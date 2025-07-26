#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tm_robot_if.srv import DrugIdentify
from cv_bridge import CvBridge

from openai_client import OpenAIClient
import tempfile
import os
import cv2


class DrugIdentifyService(Node):
    def __init__(self):
        super().__init__('drug_identify_service')
        self.srv = self.create_service(DrugIdentify, 'drug_identify', self.identify_callback)
        self.bridge = CvBridge()
        self.client = OpenAIClient(model="gpt-4o-mini", temperature=0.1)
        self.get_logger().info('Drug Identify Service is ready.')

    def identify_callback(self, request, response):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

            yaml_text = request.drug_yaml
            messages = [
                {"role": "system", "content": "你是一位資深藥學與影像辨識專家，請根據下列藥物的影像和藥物的資料。"},
                {"role": "user", "content": f"以下是藥物的描述（YAML）：\n```yaml\n{yaml_text}\n```"},
                {"role": "user", "content": [
                    {"type": "text", "text": "請根據下列藥物的影像和藥物的資料，判斷圖片中的藥品是否與JSON描述完全一致。如果藥物為正確的請回答'yes'，如果不正確則回答'no'，請不要提供理由或其他文字。"},
                    {"type": "image_data", "data": cv_image}
                ]}
            ]

            result = self.client.chat_completion(messages)
            response.result = result.choices[0].message.content.strip()
            self.get_logger().info(f"辨識結果：{response.result}")

        except Exception as e:
            self.get_logger().error(f"辨識過程錯誤：{str(e)}")
            response.result = f"error: {str(e)}"

        return response



def main(args=None):
    rclpy.init(args=args)
    node = DrugIdentifyService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
