import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch
import numpy as np
import os
import cv2
from PIL import Image as PILImage

from segment_anything import SamPredictor, sam_model_registry
from GroundingDINO.groundingdino.util.slconfig import SLConfig
from GroundingDINO.groundingdino.models import build_model
from GroundingDINO.groundingdino.util.utils import clean_state_dict, get_phrases_from_posmap
import GroundingDINO.groundingdino.datasets.transforms as T

class GroundedSAMNode(Node):
    def __init__(self):
        super().__init__('grounded_sam_node')
        self.declare_parameter("input_image", "/path/to/image.jpg")
        self.declare_parameter("text_prompt", "object")
        self.declare_parameter("grounded_config", "")
        self.declare_parameter("grounded_checkpoint", "")
        self.declare_parameter("bert_path", "")
        self.declare_parameter("sam_checkpoint", "")
        self.declare_parameter("sam_version", "vit_h")
        self.declare_parameter("box_threshold", 0.3)
        self.declare_parameter("text_threshold", 0.25)
        self.declare_parameter("device", "cuda")

        self.bridge = CvBridge()

        # Load parameters
        self.image_path = self.get_parameter("input_image").get_parameter_value().string_value
        self.text_prompt = self.get_parameter("text_prompt").get_parameter_value().string_value
        self.config_path = self.get_parameter("grounded_config").get_parameter_value().string_value
        self.checkpoint_path = self.get_parameter("grounded_checkpoint").get_parameter_value().string_value
        self.bert_path = self.get_parameter("bert_path").get_parameter_value().string_value
        self.sam_ckpt = self.get_parameter("sam_checkpoint").get_parameter_value().string_value
        self.sam_version = self.get_parameter("sam_version").get_parameter_value().string_value
        self.box_thresh = self.get_parameter("box_threshold").get_parameter_value().double_value
        self.text_thresh = self.get_parameter("text_threshold").get_parameter_value().double_value
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.predict()

    def predict(self):
        # Load GroundingDINO model
        args = SLConfig.fromfile(self.config_path)
        args.device = self.device
        args.bert_base_uncased_path = self.bert_path
        args.text_encoder_type = "bert-base-uncased"
        model = build_model(args)
        checkpoint = torch.load(self.checkpoint_path, map_location="cpu")
        model.load_state_dict(clean_state_dict(checkpoint["model"]), strict=False)
        model.eval().to(self.device)

        # Load image
        image_pil = PILImage.open(self.image_path).convert("RGB")
        transform = T.Compose([
            T.RandomResize([800], max_size=1333),
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])
        image_tensor, _ = transform(image_pil, None)

        with torch.no_grad():
            outputs = model(image_tensor[None].to(self.device), captions=[self.text_prompt])
        logits = outputs["pred_logits"].cpu().sigmoid()[0]
        boxes = outputs["pred_boxes"].cpu()[0]

        filt_mask = logits.max(dim=1)[0] > self.box_thresh
        logits_filt = logits[filt_mask]
        boxes_filt = boxes[filt_mask]

        tokenizer = model.tokenizer
        tokenized = tokenizer(self.text_prompt)
        pred_phrases = [get_phrases_from_posmap(log > self.text_thresh, tokenized, tokenizer) for log in logits_filt]

        # Convert boxes
        W, H = image_pil.size
        for i in range(boxes_filt.size(0)):
            boxes_filt[i] = boxes_filt[i] * torch.Tensor([W, H, W, H])
            boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
            boxes_filt[i][2:] += boxes_filt[i][:2]

        # SAM prediction
        predictor = SamPredictor(sam_model_registry[self.sam_version](checkpoint=self.sam_ckpt).to(self.device))
        image_np = np.array(image_pil)
        predictor.set_image(image_np)
        transformed_boxes = predictor.transform.apply_boxes_torch(boxes_filt, image_np.shape[:2]).to(self.device)

        masks, _, _ = predictor.predict_torch(
            point_coords=None,
            point_labels=None,
            boxes=transformed_boxes,
            multimask_output=False
        )

        # Publish mask as ROS Image (optional)
        # Here we just visualize the first mask as an example
        mask_np = masks[0][0].cpu().numpy().astype(np.uint8) * 255
        ros_mask = self.bridge.cv2_to_imgmsg(mask_np, encoding="mono8")
        pub = self.create_publisher(Image, "/grounded_sam/mask", 10)
        pub.publish(ros_mask)

        self.get_logger().info("Published segmented mask from SAM")


def main(args=None):
    rclpy.init(args=args)
    node = GroundedSAMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
