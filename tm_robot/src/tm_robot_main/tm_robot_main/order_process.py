import rclpy
from rclpy.node import Node
from tm_robot_if.srv import OrderID
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class OrderProcess(Node):
    def __init__(self):
        super().__init__("order_process")
        self.srv = self.create_service(OrderID, "order_id", self.order_callback)
        self.get_logger().info("OrderProcess 啟動")

        # 自動取得 package 的 share 路徑
        package_path = get_package_share_directory("tm_robot_main")
        self.order_path = os.path.join(package_path, "med_order", "med_order.yaml")
        self.info_path = os.path.join(package_path, "med_order", "med_info.yaml")
        self.output_path = os.path.join(package_path, "med_order", "sorted_order.yaml")

    def read_order_list(self):
        with open(self.order_path, "r", encoding="utf-8") as f:
            order_data = yaml.safe_load(f)
        return order_data.get("order", [])

    def read_medicine_info(self, order_list):
        with open(self.info_path, "r", encoding="utf-8") as f:
            med_info = yaml.safe_load(f)

        shelf_counter = {}
        for name in order_list:
            info = med_info.get(name, {}).get("藥物基本資料", {})
            location = info.get("存取位置", "")
            if not location or "-" not in location:
                self.get_logger().warn(f"藥品 {name} 的存取位置格式錯誤")
                continue

            shelf, _ = location.split("-")
            shelf = int(shelf)

            shelf_counter[shelf] = shelf_counter.get(shelf, 0) + 1

        result = {
            shelf: {"數量": shelf_counter[shelf]}
            for shelf in sorted(shelf_counter.keys())
        }
        return result

    def order_callback(self, request, response):
        order_list = self.read_order_list()
        result_yaml = self.read_medicine_info(order_list)

        with open(self.output_path, "w", encoding="utf-8") as f:
            yaml.dump(result_yaml, f, allow_unicode=True)

        self.get_logger().info(f"已寫入結果到 {self.output_path}")
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = OrderProcess()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
