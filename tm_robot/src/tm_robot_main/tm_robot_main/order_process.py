#!/usr/bin/env python3
import time
import yaml
import requests
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tm_robot_if.srv import  CompleteOrder

DEFAULT_BASE_URL = "http://localhost:8001"

class OrderGatewaySimple(Node):
    def __init__(self):
        super().__init__('order_gateway_simple')

        self.base_url = self.declare_parameter('base_url', DEFAULT_BASE_URL).get_parameter_value().string_value
        self.long_poll_seconds = self.declare_parameter('long_poll_seconds', 1.0).get_parameter_value().double_value
        # self.long_poll_step = self.declare_parameter('long_poll_step', 1.0).get_parameter_value().double_value
        # self.http_timeout_next = self.declare_parameter('http_timeout_next', 5.0).get_parameter_value().double_value
        self.http_timeout_complete = self.declare_parameter('http_timeout_complete', 5.0).get_parameter_value().double_value

        # 狀態
        self.waiting_for_complete = False
        self.last_order_id: Optional[str] = None

        # Services
        # self.srv_next = self.create_service(GetNextOrder, 'get_next_order', self.get_next_order_cb)
        self.srv_complete = self.create_service(CompleteOrder, 'complete_order', self.complete_order_cb)

        # Topic
        self.order_publisher = self.create_publisher(
            String, 
            '/hospital/new_order', 
            10
        )
        
        self._poll_order_timer = self.create_timer(1, self._poll_order_callback)

        self.get_logger().info(f"[OrderGatewaySimple] base_url={self.base_url} | long_poll={self.long_poll_seconds}")

    # # 取下一張訂單（阻塞直到有單）
    # def get_next_order_cb(self, req: GetNextOrder.Request, res: GetNextOrder.Response):
    #     if self.waiting_for_complete:
    #         res.success = False
    #         res.error = "still waiting for complete_order"
    #         return res

    #     try:
    #         deadline = time.time() + self.long_poll_seconds
    #         while True:
    #             ok, order, order_yaml, err = self._http_fetch_next()
    #             if ok and order:
    #                 res.success = True
    #                 res.order_id = str(order.get('order_id', ''))
    #                 res.order_yaml = order_yaml
    #                 res.error = ""
    #                 self.last_order_id = res.order_id
    #                 self.waiting_for_complete = True
    #                 self.get_logger().info(f"[get_next_order] 拿到訂單 {res.order_id}")
    #                 return res

    #             if time.time() >= deadline:
    #                 res.success = False
    #                 res.order_id = ""
    #                 res.order_yaml = ""
    #                 res.error = "no order"
    #                 return res

    #             time.sleep(max(0.0, self.long_poll_step))

    #     except Exception as e:
    #         res.success = False
    #         res.order_id = ""
    #         res.order_yaml = ""
    #         res.error = str(e)
    #         self.get_logger().exception("get_next_order exception")
    #         return res

    def _push_order_to_ros2(self, order: Dict[str, Any], order_yaml: str):
        """推送訂單到ROS2"""
        order_id = order['order_id']
        patient_name = order['patient_name']
        medicine_count = len(order['medicine'])
        
        self.get_logger().info("=" * 60)
        self.get_logger().info(f" 推送新訂單: {order_id}")
        self.get_logger().info(f" 病患: {patient_name}")
        self.get_logger().info(f" 藥物數量: {medicine_count}")
        # 顯示藥物詳情
        for i, med in enumerate(order['medicine'], 1):
            self.get_logger().info(f"  {i}. {med['name']} - 位置:{med['position']} - 數量:{med['amount']}")
        
        self.get_logger().info("=" * 60)
        # 發布到ROS2 Topic
        msg = String()
        msg.data = order_yaml
        self.order_publisher.publish(msg)
        # 發布狀態
        self.get_logger().info(f" 訂單 {order_id} 已推送到 /hospital/new_order topic")


    def _poll_order_callback(self):
        """檢查並推送新訂單"""
        try:
            # 檢查系統狀態
            response = requests.get(f"{self.base_url}/api/system/status", timeout=3)
            if response.status_code != 200:
                self.get_logger().warn("醫院系統不可用")
                return
            # 拉取下一個訂單
            response = requests.get(f"{self.base_url}/api/ros2/order/next", timeout=5)
            if response.status_code == 204:
                # 沒有新訂單
                return
            elif response.status_code == 200:
                # 有新訂單
                order_data = response.json()
                order = order_data['order']
                order_yaml = order_data['yaml']
                print(f"order: {type(order)}, {order}")
                # 檢查是否為新訂單
                if order['order_id'] != self.last_order_id:
                    print(f"order: {type(order)}, {order}")
                    print(f"order yaml: {type(order_yaml)}, {order_yaml}")
                    self._push_order_to_ros2(order, order_yaml)
                    self.last_order_id = order['order_id']
                    self.waiting_for_complete = True
                    
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"網路錯誤: {e}")
        except Exception as e:
            self.get_logger().error(f"推送訂單時發生錯誤: {e}")


    def complete_order_cb(self, req: CompleteOrder.Request, res: CompleteOrder.Response):
        try:
            
            if not self.waiting_for_complete or req.order_id != self.last_order_id:
                res.success = False
                res.error = "no matching order waiting for completion"
                self.get_logger().error(f"no matching order waiting for completion,{req.order_id},{self.last_order_id},{self.waiting_for_complete}")
                return res

            status = (req.status or "success").lower()
            if status not in ("success", "failed"):
                status = "success"

            payload = {"order_id": req.order_id, "status": status}
            r = requests.post(
                f"{self.base_url}/api/ros2/order/complete",
                json=payload,
                timeout=self.http_timeout_complete
            )
            ok = (r.status_code == 200)
            res.success = ok
            res.error = "" if ok else f"http {r.status_code}: {r.text}"

            if ok:
                self.get_logger().info(f"[complete_order] 已完成 order_id={req.order_id} status={status}")
                self.waiting_for_complete = False
                self.last_order_id = None
            else:
                self.get_logger().error(f"[complete_order] 回報失敗 {res.error}")

            return res

        except Exception as e:
            res.success = False
            res.error = str(e)
            self.get_logger().exception("complete_order exception")
            return res

    # def _http_fetch_next(self) -> Tuple[bool, Optional[Dict[str, Any]], str, str]:
    #     try:
    #         r = requests.get(f"{self.base_url}/api/ros2/order/next", timeout=self.http_timeout_next)
    #     except Exception as e:
    #         return False, None, "", f"http error: {e}"

    #     if r.status_code == 204:
    #         return True, None, "", "no order"
    #     if r.status_code != 200:
    #         return False, None, "", f"http {r.status_code}: {r.text}"

    #     try:
    #         data = r.json() or {}
    #         order = data.get("order", {}) or {}
    #         order_yaml = data.get("yaml") or yaml.safe_dump(order, allow_unicode=True)
    #         return True, order, order_yaml, ""
    #     except Exception as e:
    #         return False, None, "", f"parse error: {e}"

def main():
    rclpy.init()
    node = OrderGatewaySimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
