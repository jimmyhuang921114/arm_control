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

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image_tensor = torch.from_numpy(image_rgb).permute(2, 0, 1).float() / 255.0
        image_tensor = image_tensor.to("cuda")

        boxes, confidences, labels = predict(
            model=self.grounding_model,
            image=image_tensor,
            caption=prompt,
            box_threshold=request.confidence_threshold,
            text_threshold=0.2,
            device="cuda"
        )


        boxes = boxes * torch.Tensor([w, h, w, h]).to(boxes.device)
        input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").cpu().numpy()

        total_area = h * w
        min_area = total_area * 0.005  
        max_area = total_area * request.size_threshold

        filtered = []
        for box, conf, label in zip(input_boxes, confidences, labels):
            xmin, ymin, xmax, ymax = box
            w_box = xmax - xmin
            h_box = ymax - ymin
            box_area = w_box * h_box
            aspect_ratio = max(w_box / h_box, h_box / w_box)

            # 加入篩選條件（避免過大、過小、比例異常）
            if box_area < min_area or box_area > max_area:
                continue
            if aspect_ratio > 3.0:  # 過細長的也排除
                continue

            filtered.append((box, conf, label))

        input_boxes = np.array([f[0] for f in filtered])
        confidences = [f[1] for f in filtered]
        labels = [f[2] for f in filtered]

        self.sam2_predictor.set_image(image_bgr)

        if len(input_boxes) == 0:
            self.get_logger().warn("No boxes passed filtering.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        masks, _, _ = self.sam2_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_boxes,
            multimask_output=False,
        )

        if masks.ndim == 4:
            masks = masks.squeeze(1)

        if masks is None or len(masks) == 0:
            self.get_logger().warn("No masks predicted.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        candidates = []
        for i, (box, mask, conf, label) in enumerate(zip(input_boxes, masks, confidences, labels)):
            ys, xs = np.where(mask > 0)
            if len(xs) == 0 or len(ys) == 0:
                continue
            cx, cy = int(np.median(xs)), int(np.median(ys))
            if 0 <= cx < depth_np.shape[1] and 0 <= cy < depth_np.shape[0]:
                z = float(depth_np[cy, cx])
                if z == 0.0:
                    continue
                score = cx + cy - z * 1000
                candidates.append({
                    "score": score,
                    "box": box,
                    "mask": mask,
                    "label": label,
                    "conf": conf
                })

        if not candidates:
            self.get_logger().warn("No valid mask candidates.")
            response.bbox, response.seg, response.label, response.score = [], [], [], []
            return response

        best = sorted(candidates, key=lambda x: x["score"], reverse=True)[0]

        def single_mask_to_rle(mask):
            rle = mask_util.encode(np.array(mask[:, :, None], order="F", dtype="uint8"))[0]
            rle["counts"] = rle["counts"].decode("utf-8")
            return rle

        vis_image = image_bgr.copy()
        for c in candidates:
            color = (0, 255, 0) if c is best else (255, 0, 0)
            xmin, ymin, xmax, ymax = [int(v) for v in c["box"]]
            w_box = xmax - xmin
            h_box = ymax - ymin
            cv2.rectangle(vis_image, (xmin, ymin), (xmax, ymax), color, 2)
            cv2.putText(vis_image, f"{c['label']} {c['conf']:.2f} ({w_box}x{h_box})", (xmin, ymin - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            mask = (c["mask"] > 0).astype(np.uint8)
            colored_mask = cv2.applyColorMap(mask * 255, cv2.COLORMAP_JET)
            alpha = 0.5
            for ch in range(3):
                vis_image[:, :, ch] = np.where(
                    mask == 1,
                    cv2.addWeighted(vis_image[:, :, ch], 1 - alpha, colored_mask[:, :, ch], alpha, 0),
                    vis_image[:, :, ch]
                )

        cv2.imshow("Grounded-SAM2 Result", vis_image)
        cv2.waitKey(1)

        box = best["box"]
        binary_mask = (best["mask"] > 0).astype(np.uint8) * 255

        response.bbox = box.tolist()
        response.seg = [json.dumps(single_mask_to_rle(best["mask"]))]
        response.label = [str(best["label"])]
        response.score = [float(best["conf"])]

        # Optional: publish or return binary mask
        _, buffer = cv2.imencode(".png", binary_mask)
        binary_mask = (best["mask"] > 0).astype(np.uint8) * 255
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
