#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tm_robot_if.srv import DrugIdentify, QueryMedicineDetail
from cv_bridge import CvBridge

from openai import OpenAI
import os
import cv2
import base64
import yaml


class OpenAIClient:
    def __init__(self, model="gpt-5-mini-2025-08-07", temperature=0.1):
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

        self.detail_info_client = self.create_client(QueryMedicineDetail, 'query_medicine_detail')

        self.get_logger().info('Drug Identify Service is ready.')

    def query_detail_info(self, med_name):
        if not self.detail_info_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("藥物詳細查詢服務不可用")

        req = QueryMedicineDetail.Request()
        req.medicine_name = med_name
        res = self.detail_info_client.call(req)

        if not res.success:
            raise ValueError(f"查詢失敗: {res.error}")

        try:
            return yaml.safe_load(res.detail_yaml)
        except Exception as e:
            raise ValueError(f"解析 YAML 失敗: {e}")

    def identify_callback(self, request, response):
        try:
            drug_detail = self.query_detail_info(request.name)
            if not drug_detail:
                raise ValueError(f"找不到藥物 '{request.name}' 的詳細資料")

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
