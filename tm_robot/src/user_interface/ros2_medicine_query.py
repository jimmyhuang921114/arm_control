#!/usr/bin/env python3
"""
ROS2 Medicine Query Service
藥物資訊查詢服務

此檔案提供：
1. 基礎藥物資訊查詢
2. 詳細藥物資訊查詢
3. ROS2 Service 和 Topic 介面
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
import requests
import json
import yaml
from typing import Dict, Any, Optional

class MedicineQueryService(Node):
    """
    藥物查詢服務
    - 提供基礎和詳細藥物資訊查詢
    - 支援 ROS2 Service 和 Topic 介面
    """
    
    def __init__(self):
        super().__init__('medicine_query_service')
        
        # 醫院系統設定
        self.hospital_base_url = 'http://localhost:8001'
        
        # 發布器
        self.basic_info_pub = self.create_publisher(
            String,
            '/hospital/medicine_basic_response',
            10
        )
        
        self.detail_info_pub = self.create_publisher(
            String,
            '/hospital/medicine_detail_response',
            10
        )
        
        # 訂閱器 - 監聽查詢請求
        self.basic_query_sub = self.create_subscription(
            String,
            '/hospital/query_medicine_basic',
            self.handle_basic_query,
            10
        )
        
        self.detail_query_sub = self.create_subscription(
            String,
            '/hospital/query_medicine_detail', 
            self.handle_detail_query,
            10
        )
        
        self.get_logger().info(" 藥物查詢服務已啟動")
        self.get_logger().info(f" 連接醫院系統: {self.hospital_base_url}")
    
    def handle_basic_query(self, msg):
        """處理基礎藥物查詢"""
        medicine_name = msg.data.strip()
        self.get_logger().info(f" 查詢基礎藥物資訊: {medicine_name}")
        
        # 查詢藥物資訊
        info = self.query_medicine_basic(medicine_name)
        if info:
            # 發布結果
            response_msg = String()
            response_msg.data = info['yaml']
            self.basic_info_pub.publish(response_msg)
            
            self.get_logger().info(f" 已發布 {medicine_name} 基礎資訊")
        else:
            # 發布錯誤資訊
            error_msg = String()
            error_data = {
                "error": "藥物不存在",
                "medicine_name": medicine_name,
                "timestamp": rclpy.clock.Clock().now().nanoseconds / 1e9
            }
            error_msg.data = yaml.safe_dump(error_data, allow_unicode=True)
            self.basic_info_pub.publish(error_msg)
            
            self.get_logger().warn(f" 找不到藥物: {medicine_name}")
    
    def handle_detail_query(self, msg):
        """處理詳細藥物查詢"""
        medicine_name = msg.data.strip()
        self.get_logger().info(f" 查詢詳細藥物資訊: {medicine_name}")
        
        # 查詢藥物詳細資訊
        info = self.query_medicine_detail(medicine_name)
        if info:
            # 發布結果
            response_msg = String()
            response_msg.data = info['yaml']
            self.detail_info_pub.publish(response_msg)
            
            self.get_logger().info(f" 已發布 {medicine_name} 詳細資訊")
        else:
            # 發布錯誤資訊
            error_msg = String()
            error_data = {
                "error": "藥物詳細資訊不存在",
                "medicine_name": medicine_name,
                "timestamp": rclpy.clock.Clock().now().nanoseconds / 1e9
            }
            error_msg.data = yaml.safe_dump(error_data, allow_unicode=True)
            self.detail_info_pub.publish(error_msg)
            
            self.get_logger().warn(f" 找不到藥物詳細資訊: {medicine_name}")
    
    def query_medicine_basic(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """查詢基礎藥物資訊"""
        try:
            response = requests.get(
                f"{self.hospital_base_url}/api/ros2/medicine/basic/{medicine_name}",
                timeout=3
            )
            if response.status_code == 200:
                data = response.json()
                self.get_logger().info(f" {medicine_name} - 位置:{data['position']} 庫存:{data['amount']}")
                return data
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"查詢基礎藥物資訊錯誤: {e}")
            return None
    
    def query_medicine_detail(self, medicine_name: str) -> Optional[Dict[str, Any]]:
        """查詢詳細藥物資訊"""
        try:
            response = requests.get(
                f"{self.hospital_base_url}/api/ros2/medicine/detailed/{medicine_name}",
                timeout=3
            )
            if response.status_code == 200:
                data = response.json()
                content_preview = data['content'][:50] + "..." if len(data['content']) > 50 else data['content']
                self.get_logger().info(f" {medicine_name} 詳細內容: {content_preview}")
                return data
            else:
                return None
        except Exception as e:
            self.get_logger().error(f"查詢詳細藥物資訊錯誤: {e}")
            return None
    
    def query_all_medicines(self):
        """查詢所有藥物列表"""
        try:
            response = requests.get(f"{self.hospital_base_url}/api/medicine/basic", timeout=5)
            if response.status_code == 200:
                medicines = response.json()
                self.get_logger().info(f" 系統中共有 {len(medicines)} 種藥物:")
                for med in medicines:
                    self.get_logger().info(f"  - {med['name']} (位置: {med['position']}, 庫存: {med['amount']})")
                return medicines
            else:
                self.get_logger().error("無法取得藥物列表")
                return []
        except Exception as e:
            self.get_logger().error(f"查詢藥物列表錯誤: {e}")
            return []


def test_queries(node: MedicineQueryService):
    """測試查詢功能"""
    print("\n" + "="*50)
    print(" 測試藥物查詢功能")
    print("="*50)
    
    # 查詢所有藥物
    medicines = node.query_all_medicines()
    
    if medicines:
        # 測試第一個藥物的基礎和詳細資訊
        test_medicine = medicines[0]['name']
        print(f"\n🔍 測試查詢: {test_medicine}")
        
        # 基礎資訊
        basic_info = node.query_medicine_basic(test_medicine)
        if basic_info:
            print(" 基礎資訊查詢成功")
        else:
            print(" 基礎資訊查詢失敗")
        
        # 詳細資訊
        detail_info = node.query_medicine_detail(test_medicine)
        if detail_info:
            print("詳細資訊查詢成功")
        else:
            print(" 詳細資訊查詢失敗")
    
    print("="*50)


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    query_service = MedicineQueryService()
    
    print("\n" + "="*60)
    print(" 藥物查詢服務")
    print("="*60)
    print(" ROS2 Topics:")
    print("  訂閱 (查詢請求):")
    print("    - /hospital/query_medicine_basic   (發送藥物名稱)")
    print("    - /hospital/query_medicine_detail  (發送藥物名稱)")
    print()
    print("  發布 (查詢結果):")
    print("    - /hospital/medicine_basic_response")
    print("    - /hospital/medicine_detail_response")
    print()
    print(" 使用範例:")
    print("  # 查詢基礎資訊")
    print("  ros2 topic pub /hospital/query_medicine_basic std_msgs/String \"data: 'Aspirin'\"")
    print()
    print("  # 監聽基礎資訊回應")
    print("  ros2 topic echo /hospital/medicine_basic_response")
    print()
    print("  # 查詢詳細資訊")
    print("  ros2 topic pub /hospital/query_medicine_detail std_msgs/String \"data: 'Aspirin'\"")
    print()
    print("  # 監聽詳細資訊回應")
    print("  ros2 topic echo /hospital/medicine_detail_response")
    print("="*60)
    
    # 執行測試
    try:
        test_queries(query_service)
    except Exception as e:
        query_service.get_logger().warn(f"測試時發生錯誤: {e}")
    
    print("\n 服務已啟動，等待查詢請求...")
    
    try:
        rclpy.spin(query_service)
    except KeyboardInterrupt:
        query_service.get_logger().info(" 查詢服務已停止")
    finally:
        query_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()