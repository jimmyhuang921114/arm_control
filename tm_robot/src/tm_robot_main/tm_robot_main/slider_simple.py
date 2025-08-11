#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_rail_interface.srv import RailControl  # 依實際套件修改


class SliderControlNode(Node):
    def __init__(self):
        super().__init__('slider_control_node')

        # 建立 RailControl Service Client
        self.slider_client = self.create_client(RailControl, 'rail_control')

        self.get_logger().info("等待 RailControl 服務...")
        while not self.slider_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("RailControl 服務尚未啟動，持續等待...")

        self.get_logger().info("RailControl 服務已連線")

        # 依序呼叫 opt_code = 0, 1, 2
        # self.call_slider_sequence()
        self.timer = self.create_timer(1.0, self.call_slider_sequence)

    def call_slider(self, code: int):
        self.slider_finish = False
        """單次呼叫 RailControl"""
        req = RailControl.Request()
        req.opt_code = code
        req.rail_name = "arm_rail1"

        self.get_logger().info(f"呼叫 RailControl: opt_code={code}")
        future = self.slider_client.call_async(req)
        future.add_done_callback(self.slider_callback)
        # rclpy.spin_until_future_complete(self, future)            

    def slider_callback(self, future):
        self.slider_finish = True
        try:
            res = future.result()
            if res and res.result >= 0:
                self.get_logger().info(f"RailControl success")
                return True
            else:
                self.get_logger().error(f"RailControl failed")
                return False
        except:
            self.get_logger().error("RailControl 呼叫未完成")
            return False

    def wait_slider(self):
        while not self.slider_finish:
            rclpy.spin_once(self, timeout_sec=0.1)
            

    def call_slider_sequence(self):
        """依序執行 0 → 1 → 2"""
        steps = [
            (0, "初始化"),
            (1, "移動到取貨位置"),
            (2, "返回位置")
        ]
        for code, desc in steps:
            self.get_logger().info(f"開始 {desc} (opt_code={code})")
            self.call_slider(code)
            self.wait_slider()
            self.get_logger().info(f"{desc} 完成")
        self.timer.cancel()
        self.destroy_timer(self.timer)


def main(args=None):
    rclpy.init(args=args)
    node = SliderControlNode()
    node.create_rate(100)
    while rclpy.ok():
        try:
            rclpy.spin_once(node)
        except KeyboardInterrupt:
            break
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
