#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tm_robot_if.srv import DrugIdentify
from cv_bridge import CvBridge

from openai import OpenAI
import os, cv2, base64, yaml, requests
from urllib.parse import quote  # ← 新增

DEFAULT_BASE_URL = "http://localhost:8001"  # ← 新增

class OpenAIClient:
    def __init__(self, model="gpt-4o", temperature=0.1):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = model
        self.temperature = temperature

    def encode_image(self, image):
        _, buffer = cv2.imencode('.png', image)
        return base64.b64encode(buffer).decode('utf-8')

    def chat_completion(self, messages):
        processed = []
        for msg in messages:
            if isinstance(msg["content"], list):
                parts = []
                for part in msg["content"]:
                    if part["type"] == "text":
                        parts.append(part)
                    elif part["type"] == "image_data":
                        parts.append({
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{self.encode_image(part['data'])}"}
                        })
                processed.append({"role": msg["role"], "content": parts})
            else:
                processed.append(msg)
        return self.client.chat.completions.create(
            model=self.model, messages=processed, temperature=self.temperature
        )

class DrugIdentifyService(Node):
    def __init__(self):
        super().__init__('drug_identify_service')

        # 參數：直接抓後端 HTTP
        self.base_url = self.declare_parameter('base_url', DEFAULT_BASE_URL)\
            .get_parameter_value().string_value
        self.http_timeout = self.declare_parameter('http_timeout', 5.0)\
            .get_parameter_value().double_value

        self.srv = self.create_service(DrugIdentify, 'drug_identify', self.identify_callback)
        self.bridge = CvBridge()
        self.client = OpenAIClient(model="gpt-4o", temperature=0.1)

        self.get_logger().info('Drug Identify Service is ready.')

    
    def query_detail_info(self, med_name: str):
        name_q = quote((med_name or "").strip(), safe="")
        if not name_q:
            raise ValueError("藥名為空")

        url = f"{self.base_url}/api/ros2/medicine/detailed/{name_q}"
        r = requests.get(url, timeout=self.http_timeout, headers={"Accept": "application/json"})
        if r.status_code != 200:
            preview = (r.text or "")[:200].replace("\n", " ")
            raise RuntimeError(f"http {r.status_code}: {preview}")
        try:
            data = r.json() or {}
        except Exception as je:
            preview = (r.text or "")[:200].replace("\n", " ")
            raise RuntimeError(f"invalid json: {je}; body preview: {preview}")

        raw_yaml = (data.get("yaml") or data.get("content") or "").strip()
        if not raw_yaml:
            raise ValueError("詳細資料為空（yaml/content 都沒有）")
        try:
            parsed = yaml.safe_load(raw_yaml)
            self.get_logger().info(f"[drug_identify] 取得 detail YAML，長度={len(raw_yaml)}")
            return parsed
        except Exception as e:
            raise ValueError(f"解析 YAML 失敗：{e}")

    def identify_callback(self, request, response):
        try:
            self.get_logger().info(f"[drug_identify] 收到請求 name={request.name}")
            drug_detail = self.query_detail_info(request.name)

            yaml_text = yaml.dump(drug_detail, allow_unicode=True)
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

            messages = [
                {"role": "system", "content": "你是一位資深藥學與影像辨識專家，請根據下列藥物的影像和藥物的資料。"},
                {"role": "user", "content": f"以下是藥物的描述（YAML）：\n```yaml\n{yaml_text}\n```"},
                {"role": "user", "content": [
                    {"type": "text", "text":
                        "請根據下列藥物的影像和藥物的資料，判斷圖片中的藥品是否與YAML差不多 。只要有一項對了就好"
                        "如果正確請回答 'yes'，如果不正確則回答 'no'，不要提供理由或其他文字。"},
                    {"type": "image_data", "data": cv_image}
                ]}
            ]

            result = self.client.chat_completion(messages)
            response.result = (result.choices[0].message.content or "").strip()
            self.get_logger().info(f"辨識結果：{response.result}")

        except Exception as e:
            self.get_logger().error(f"辨識過程錯誤：{str(e)}")
            response.result = f"error: {str(e)}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DrugIdentifyService()
    try:
        rclpy.spin(node)   # 單執行緒 OK
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
