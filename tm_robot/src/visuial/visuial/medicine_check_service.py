#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tm_robot_if.srv import DrugIdentify
from cv_bridge import CvBridge

from openai import OpenAI
import tempfile
import os
import cv2
import base64
import yaml
import numpy as np

from ament_index_python.packages import get_package_share_directory


class OpenAIClient:
    def __init__(self, model="gpt-4o-mini", temperature=0.1):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = model
        self.temperature = temperature

    def encode_image(self, image):
        _, buffer = cv2.imencode('.png', image)
        base64_image = base64.b64encode(buffer).decode('utf-8')
        return base64_image

    def chat_completion(self, messages):
        processed_messages = []
        for msg in messages:
            if isinstance(msg["content"], list):
                parts = []
                for part in msg["content"]:
                    if part["type"] == "text":
                        parts.append(part)
                    elif part["type"] == "image_data":
                        encoded = self.encode_image(part["data"])
                        parts.append({
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/png;base64,{encoded}"
                            }
                        })
                processed_messages.append({"role": msg["role"], "content": parts})
            else:
                processed_messages.append(msg)

        return self.client.chat.completions.create(
            model=self.model,
            messages=processed_messages,
            temperature=self.temperature
        )


class DrugIdentifyService(Node):
    def __init__(self):
        super().__init__('drug_identify_service')
        self.srv = self.create_service(DrugIdentify, 'drug_identify', self.identify_callback)
        self.bridge = CvBridge()
        self.client = OpenAIClient(model="gpt-4o", temperature=0.1)

        # 載入 med_info.yaml
        # package_path = get_package_share_directory('tm_robot_main')
        # self.info_path = os.path.join(package_path, "med_order", "med_info.yaml")
        # MED_INFO_PATH = '/workspace/tm_robot/src/tm_robot_main/tm_robot_main/med_order/med_info.yaml'
        MED_INFO_PATH = '/workspace/tm_robot/src/visuial/visuial/medicine_info2.yaml'
        with open(MED_INFO_PATH, "r", encoding="utf-8") as f:
            self.med_info = yaml.safe_load(f)

        self.get_logger().info(f"medicine info{self.med_info}")
        self.get_logger().info('Drug Identify Service is ready.')

    def identify_callback(self, request, response):
        try:
            name = request.name
            if name not in self.med_info:
                raise ValueError(f"藥物名稱 '{name}' 不存在於 med_info.yaml")

            drug_detail = self.med_info[name].get("藥物詳細資料", {})
            if not drug_detail:
                raise ValueError(f"找不到藥物 '{name}' 的詳細資料")

            # 轉成 YAML 字串
            yaml_text = yaml.dump(drug_detail, allow_unicode=True)

            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

            messages = [
                {"role": "system", "content": "你是一位資深藥學與影像辨識專家，請根據下列藥物的影像和藥物的資料。"},
                {"role": "user", "content": f"以下是藥物的描述（YAML）：\n```yaml\n{yaml_text}\n```"},
                {"role": "user", "content": [
                    {"type": "text", "text": "請根據下列藥物的影像和藥物的資料，判斷圖片中的藥品是否與YAML描述完全一致。如果藥物為正確的請回答'yes'，如果不正確則回答'no'，請不要提供理由或其他文字。"},
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
