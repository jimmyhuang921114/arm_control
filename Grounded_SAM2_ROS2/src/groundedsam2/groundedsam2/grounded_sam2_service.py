import sys
sys.path.append('/workspace/Grounded-SAM-2')

import rclpy
from rclpy.node import Node
from tm_robot_if.srv import GroundedSAM2Interface
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import torch
import json
import numpy as np
import pycocotools.mask as mask_util
from torchvision.ops import box_convert
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from grounding_dino.groundingdino.util.inference import load_model, predict

class GroundedSAM2Service(Node):
    def __init__(self):
        super().__init__('grounded_sam2_service')

        self.get_logger().warn(f"Initialize Grounded Sam2 Service ...")

        self.srv = self.create_service(
            GroundedSAM2Interface,
            'grounded_sam2',
            self.grounded_sam2_callback
        )

        self.sampler_checkpoint = "./Grounded-SAM-2/checkpoints/sam2.1_hiera_large.pt"
        self.model_cfg = "configs/sam2.1/sam2.1_hiera_l.yaml"
        self.sam2_model = build_sam2(self.model_cfg, self.sampler_checkpoint, device="cuda")
        self.sam2_predictor = SAM2ImagePredictor(self.sam2_model)

        self.grounding_model = load_model(
            model_config_path="./Grounded-SAM-2/grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py",
            model_checkpoint_path="./Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth",
            device="cuda"
        )

        self.bridge = CvBridge()
        self.get_logger().info(f"\033[92mInitialization completed")

    def grounded_sam2_callback(self, request, response):
        self.get_logger().info(f"Processing image with prompt: {request.prompt}")

        try:
            image_bgr = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
            depth_np = self.bridge.imgmsg_to_cv2(request.depth, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")
            return response

        h, w, _ = image_bgr.shape
        prompt = request.prompt

        # ========= 只把「畫面中間」餵給 GroundingDINO =========
        # 中間一半
        half_width = w // 2
        quarter_width = w // 4
        x0, x1 = quarter_width, quarter_width + half_width

        center_img_bgr = image_bgr[:, x0:x1]
        ch, cw = center_img_bgr.shape[:2]

        # GroundingDINO 用 RGB tensor（裁切影像）
        center_img_rgb = cv2.cvtColor(center_img_bgr, cv2.COLOR_BGR2RGB)
        center_tensor = torch.from_numpy(center_img_rgb).permute(2, 0, 1).float() / 255.0
        center_tensor = center_tensor.to("cuda")

        boxes, confidences, labels = predict(
            model=self.grounding_model,
            image=center_tensor,
            caption=prompt,
            box_threshold=request.confidence_threshold,
            text_threshold=request.confidence_threshold,
            device="cuda"
        )


        if boxes is None or len(boxes) == 0:
            self.get_logger().warn("No boxes from GroundingDINO on center crop.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        boxes = boxes * torch.Tensor([cw, ch, cw, ch]).to(boxes.device)
        input_boxes_crop_xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").cpu().numpy()

  
        input_boxes = input_boxes_crop_xyxy.copy()
        input_boxes[:, [0, 2]] += x0 


        self.sam2_predictor.set_image(image_bgr)

        if len(input_boxes) == 0:
            self.get_logger().warn("No boxes after mapping back to full image.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        masks, _, _ = self.sam2_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_boxes,
            multimask_output=False,
        )

        if masks is None or len(masks) == 0:
            self.get_logger().warn("No masks predicted.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        if masks.ndim == 4:
            masks = masks.squeeze(1)

        candidates = []


        total_area = float(h * w)
        max_area_ratio = 0.10  # 10%：過大雜訊丟掉
        min_area_ratio = 0.002  # 0.2%：過小雜訊丟掉
        min_area = total_area * min_area_ratio
        max_area = total_area * max_area_ratio
        for i, (box, mask, conf, label) in enumerate(zip(input_boxes, masks, confidences, labels)):
            ys, xs = np.where(mask > 0)

            area = float(np.sum(mask))
            
            if area < min_area or area > max_area:
                self.get_logger().debug(f"Reject by area: {area/total_area:.4f}")
                continue

            if len(xs) == 0 or len(ys) == 0:
                continue
            
            cx, cy = int(np.median(xs)), int(np.median(ys))
            if 0 <= cx < depth_np.shape[1] and 0 <= cy < depth_np.shape[0]:
                z = float(depth_np[cy, cx])
                if z == 0.0:
                    continue    

                score = cy * 10000

                candidates.append({
                    "score": score,
                    "box": box,
                    "mask": mask,
                    "label": label,
                    "conf": float(conf)
                })

        if not candidates:
            self.get_logger().warn("No valid mask candidates after scoring.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response


        best = sorted(candidates, key=lambda x: x["score"], reverse=True)[0]

        def single_mask_to_rle(mask):
            rle = mask_util.encode(np.array(mask[:, :, None], order="F", dtype="uint8"))[0]
            rle["counts"] = rle["counts"].decode("utf-8")
            return rle

        # 視覺化（可留著除錯）
        vis_image = image_bgr.copy()
        
        cv2.line(vis_image, (x0, 0), (x0, h-1), (0, 0, 0), 2)
        cv2.line(vis_image, (x1-1, 0), (x1-1, h-1), (0, 0, 0), 2)

        for c in candidates:
            color = (0, 255, 0) if c is best else (255, 0, 0)
            xmin, ymin, xmax, ymax = [int(v) for v in c["box"]]
            w_box = xmax - xmin
            h_box = ymax - ymin
            cv2.rectangle(vis_image, (xmin, ymin), (xmax, ymax), color, 2)
            cv2.putText(vis_image, f"{c['label']} {c['conf']:.2f} ({w_box}x{h_box})",
                        (xmin, max(0, ymin - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            mask_u8 = (c["mask"] > 0).astype(np.uint8)
            colored_mask = cv2.applyColorMap(mask_u8 * 255, cv2.COLORMAP_JET)
            alpha = 0.5
            for ch in range(3):
                vis_image[:, :, ch] = np.where(
                    mask_u8 == 1,
                    cv2.addWeighted(vis_image[:, :, ch], 1 - alpha, colored_mask[:, :, ch], alpha, 0),
                    vis_image[:, :, ch]
                )

        cv2.imshow("Grounded-SAM2 Result (Center-fed DINO)", vis_image)
        cv2.waitKey(1)

        # 回傳最佳
        box = best["box"]
        binary_mask = (best["mask"] > 0).astype(np.uint8) * 255

        response.bbox = box.tolist()
        response.seg = [json.dumps(single_mask_to_rle(best["mask"]))]
        response.label = [str(best["label"])]
        response.score = [float(best["conf"])]
        response.binary_image = self.bridge.cv2_to_imgmsg(binary_mask, encoding='mono8')
        response.binary_image.header = request.image.header

        return response


def main(args=None):
    rclpy.init(args=args)
    service = GroundedSAM2Service()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
