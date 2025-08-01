#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tm_robot_if.srv import DrugIdentify
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import sys
import os


class DrugIdentifyClient(Node):
    def __init__(self):
        super().__init__('drug_identify_client')
        self.cli = self.create_client(DrugIdentify, 'drug_identify')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('等待 drug_identify 服務啟動中...')
        self.req = DrugIdentify.Request()
        self.bridge = CvBridge()

    def send_request(self, image_path, yaml_path):
        if not os.path.exists(image_path):
            self.get_logger().error(f"找不到圖片檔案：{image_path}")
            return
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"找不到 YAML 檔案：{yaml_path}")
            return

        # 讀取圖片並轉換為 ROS Image
        image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

        # 讀取 YAML 字串
        with open(yaml_path, 'r', encoding='utf-8') as f:
            yaml_text = f.read()

        self.req.image = ros_image
        self.req.drug_yaml = yaml_text

        self.get_logger().info("發送辨識請求中...")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f"辨識結果：{future.result().result}")
        else:
            self.get_logger().error("辨識服務回應失敗")


def main():
    if len(sys.argv) < 3:
        print("用法: ros2 run dis_service test_drug_identify_client <image_path> <yaml_path>")
        return

    image_path = sys.argv[1]
    yaml_path = sys.argv[2]

    rclpy.init()
    client = DrugIdentifyClient()
    client.send_request(image_path, yaml_path)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
