# yolo_mask_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloMaskNode(Node):
    def __init__(self):
        super().__init__('yolo_mask_node')

        self.bridge = CvBridge()
        self.model = YOLO('/workspace/tm_robot/src/visuial/resource/best.pt')

        self.image_sub = self.create_subscription(
            Image,
            '/tm_robot/color_image',
            self.image_callback,
            10
        )

        self.mask_pub = self.create_publisher(Image, '/yolo/mask', 10)
        self.get_logger().info('YOLO Mask Node started.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO segmentation inference
        results = self.model(frame)
        masks = results[0].masks  # list of masks
        
        if masks is None:
            self.get_logger().warn('No mask detected.')
            return

        # Assume using first mask only
        mask_np = masks.data[0].cpu().numpy().astype(np.uint8) * 255

        mask_msg = self.bridge.cv2_to_imgmsg(mask_np, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)
        self.get_logger().info('Published 1st YOLO mask.')


def main(args=None):
    rclpy.init(args=args)
    node = YoloMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()