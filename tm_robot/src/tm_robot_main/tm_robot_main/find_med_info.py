#!/usr/bin/env python3
"""
ROS2 Medicine Query - Service Only
- /query_medicine_basic  : 回傳基礎藥物資訊（含型別欄位與 yaml）
- /query_medicine_detail : 回傳詳細藥物資訊（含型別欄位與 yaml）
資料來源：
  GET {base}/api/ros2/medicine/basic/{name}
  GET {base}/api/ros2/medicine/detailed/{name}
"""

import requests
import rclpy
from rclpy.node import Node

from tm_robot_if.srv import QueryMedicineBasic, QueryMedicineDetail

DEFAULT_BASE_URL = "http://localhost:8001"

class MedicineQueryServiceSrv(Node):
    def __init__(self):
        super().__init__('medicine_query_service_srv')


        self.base_url = self.declare_parameter('base_url', DEFAULT_BASE_URL)\
            .get_parameter_value().string_value
        self.http_timeout = self.declare_parameter('http_timeout', 3.0)\
            .get_parameter_value().double_value

        self.srv_basic = self.create_service(
            QueryMedicineBasic, 'query_medicine_basic', self._handle_basic)
        self.srv_detail = self.create_service(
            QueryMedicineDetail, 'query_medicine_detail', self._handle_detail)

        self.get_logger().info(f"[MedicineQueryServiceSrv] online | base_url={self.base_url}")

    # -------- service callbacks --------

    def _handle_basic(self, req: QueryMedicineBasic.Request, res: QueryMedicineBasic.Response):
        name = (req.medicine_name or "").strip()
        if not name:
            res.success = False
            res.error = "medicine_name is required"
            return res

        try:
            r = requests.get(
                f"{self.base_url}/api/ros2/medicine/basic/{name}",
                timeout=self.http_timeout
            )
            if r.status_code != 200:
                res.success = False
                res.error = f"http {r.status_code}"
                return res

            data = r.json() or {}
            # 填回型別欄位
            res.success = True
            res.name = data.get("name", "")
            res.position = data.get("position", "")
            res.amount = int(data.get("amount", 0))
            # 可能是 float 或字串，轉成 float
            try:
                res.confidence = float(data.get("confidence", 0.0))
            except Exception:
                res.confidence = 0.0
            res.prompt = data.get("prompt", "")
            res.yaml = data.get("yaml", "")
            res.error = ""
            self.get_logger().info(f"[basic] {res.name} pos={res.position} amt={res.amount}")
            return res

        except Exception as e:
            res.success = False
            res.error = str(e)
            return res

    def _handle_detail(self, req: QueryMedicineDetail.Request, res: QueryMedicineDetail.Response):
        name = (req.medicine_name or "").strip()
        if not name:
            res.success = False
            res.error = "medicine_name is required"
            return res

        try:
            r = requests.get(
                f"{self.base_url}/api/ros2/medicine/detailed/{name}",
                timeout=self.http_timeout
            )
            if r.status_code != 200:
                res.success = False
                res.error = f"http {r.status_code}"
                return res

            data = r.json() or {}
            res.success = True
            res.name = data.get("name", "")
            res.content = data.get("content", "")
            res.yaml = data.get("yaml", "")
            res.error = ""
            self.get_logger().info(f"[detail] {res.name} content_len={len(res.content)}")
            return res

        except Exception as e:
            res.success = False
            res.error = str(e)
            return res


def main():
    rclpy.init()
    node = MedicineQueryServiceSrv()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
