#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tm_robot_if.srv import OCRMatch  # 替換成你的 package name
import easyocr
import numpy as np
import cv2


class EasyOCRServiceNode(Node):
    def __init__(self):
        super().__init__('easyocr_service_node')
        self.bridge = CvBridge()

        self.reader = easyocr.Reader(['en', 'ch_sim'])

        self.srv = self.create_service(OCRMatch, 'easyocr_match', self.ocr_callback)
        self.get_logger().info("EasyOCR Service 已啟動")

    def ocr_callback(self, request, response):
        try:
            # 將 ROS Image 轉為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

            results = self.reader.readtext(cv_image)
            found = False

            for (bbox, text, prob) in results:
                if prob < 0.3:
                    continue
                self.get_logger().info(f"OCR: {text} (score={prob:.2f})")
                if request.target_text.lower() in text.lower():
                    found = True
                    break

            response.success = found
            return response

        except Exception as e:
            self.get_logger().error(f"OCR 處理失敗: {e}")
            response.success = False
            return response


def main(args=None):
    rclpy.init(args=args)
    node = EasyOCRServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
