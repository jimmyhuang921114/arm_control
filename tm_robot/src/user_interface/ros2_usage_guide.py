#!/usr/bin/env python3
"""
ROS2 醫院系統使用指南和測試
使用指南 + 完整測試範例

此檔案示範如何：
1. 接收自動推送的訂單
2. 查詢基礎和詳細藥物資訊
3. 回報訂單完成狀態
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import requests
import time

class HospitalSystemDemo(Node):
    """
    醫院系統演示節點
    展示如何與醫院系統進行完整的ROS2通訊
    """
    
    def __init__(self):
        super().__init__('hospital_system_demo')
        
        # 醫院系統設定
        self.hospital_base_url = 'http://localhost:8001'
        self.current_order = None
        
        # 訂閱器 - 接收訂單和藥物資訊
        self.order_sub = self.create_subscription(
            String,
            '/hospital/new_order',
            self.handle_new_order,
            10
        )
        
        self.medicine_basic_sub = self.create_subscription(
            String,
            '/hospital/medicine_basic_response',
            self.handle_medicine_basic_response,
            10
        )
        
        self.medicine_detail_sub = self.create_subscription(
            String,
            '/hospital/medicine_detail_response',
            self.handle_medicine_detail_response,
            10
        )
        
        # 發布器 - 發送查詢請求
        self.basic_query_pub = self.create_publisher(
            String,
            '/hospital/query_medicine_basic',
            10
        )
        
        self.detail_query_pub = self.create_publisher(
            String,
            '/hospital/query_medicine_detail',
            10
        )
        
        self.get_logger().info("🏥 醫院系統演示節點已啟動")
    
    def handle_new_order(self, msg):
        """處理新訂單"""
        try:
            order_data = yaml.safe_load(msg.data)
            self.current_order = order_data
            
            order_id = order_data.get('order_id', 'Unknown')
            patient_name = order_data.get('patient_name', 'Unknown')
            medicines = order_data.get('medicine', [])
            
            self.get_logger().info("🎯 收到新訂單！")
            self.get_logger().info("=" * 50)
            self.get_logger().info(f"📋 訂單ID: {order_id}")
            self.get_logger().info(f"👤 病患: {patient_name}")
            self.get_logger().info(f"💊 藥物數量: {len(medicines)}")
            
            # 顯示藥物列表
            for i, med in enumerate(medicines, 1):
                self.get_logger().info(f"  {i}. {med['name']}")
                self.get_logger().info(f"     位置: {med['position']}")
                self.get_logger().info(f"     數量: {med['amount']}")
                self.get_logger().info(f"     提示詞: {med['prompt']}")
                self.get_logger().info(f"     信心值: {med['confidence']}")
                self.get_logger().info("")
            
            self.get_logger().info("=" * 50)
            
            # 開始處理訂單
            self.process_order(order_data)
            
        except Exception as e:
            self.get_logger().error(f"處理訂單時發生錯誤: {e}")
    
    def process_order(self, order_data):
        """處理訂單流程"""
        order_id = order_data.get('order_id')
        medicines = order_data.get('medicine', [])
        
        self.get_logger().info(f"🚀 開始處理訂單 {order_id}")
        
        # 回報開始處理
        self.report_progress(order_id, "started", "開始處理訂單")
        
        # 處理每個藥物
        for i, medicine in enumerate(medicines, 1):
            medicine_name = medicine['name']
            self.get_logger().info(f"🔍 處理藥物 {i}/{len(medicines)}: {medicine_name}")
            
            # 查詢基礎資訊
            self.query_medicine_basic(medicine_name)
            
            # 等待一下再查詢詳細資訊
            time.sleep(1)
            
            # 查詢詳細資訊
            self.query_medicine_detail(medicine_name)
            
            # 模擬處理時間
            time.sleep(2)
            
            # 回報藥物處理完成
            self.report_progress(order_id, "item_completed", f"{medicine_name} 處理完成")
        
        # 完成訂單
        self.get_logger().info(f"✅ 訂單 {order_id} 處理完成！")
        self.complete_order(order_id, "success", "所有藥物已處理完成")
    
    def query_medicine_basic(self, medicine_name: str):
        """查詢基礎藥物資訊"""
        msg = String()
        msg.data = medicine_name
        self.basic_query_pub.publish(msg)
        self.get_logger().info(f"📊 已請求 {medicine_name} 基礎資訊")
    
    def query_medicine_detail(self, medicine_name: str):
        """查詢詳細藥物資訊"""
        msg = String()
        msg.data = medicine_name
        self.detail_query_pub.publish(msg)
        self.get_logger().info(f"📝 已請求 {medicine_name} 詳細資訊")
    
    def handle_medicine_basic_response(self, msg):
        """處理基礎藥物資訊回應"""
        try:
            data = yaml.safe_load(msg.data)
            if 'error' in data:
                self.get_logger().warn(f"❌ 基礎資訊查詢失敗: {data['error']}")
            else:
                name = data.get('name', 'Unknown')
                position = data.get('position', 'Unknown')
                amount = data.get('amount', 0)
                confidence = data.get('confidence', 0)
                self.get_logger().info(f"📊 {name} 基礎資訊 - 位置:{position} 庫存:{amount} 信心值:{confidence}")
        except Exception as e:
            self.get_logger().error(f"處理基礎資訊回應錯誤: {e}")
    
    def handle_medicine_detail_response(self, msg):
        """處理詳細藥物資訊回應"""
        try:
            data = yaml.safe_load(msg.data)
            if 'error' in data:
                self.get_logger().warn(f"❌ 詳細資訊查詢失敗: {data['error']}")
            else:
                name = data.get('name', 'Unknown')
                content = data.get('content', 'No content')
                preview = content[:50] + "..." if len(content) > 50 else content
                self.get_logger().info(f"📝 {name} 詳細資訊: {preview}")
        except Exception as e:
            self.get_logger().error(f"處理詳細資訊回應錯誤: {e}")
    
    def report_progress(self, order_id: str, stage: str, message: str):
        """回報訂單進度"""
        try:
            payload = {
                "order_id": order_id,
                "stage": stage,
                "message": message
            }
            response = requests.post(
                f"{self.hospital_base_url}/api/ros2/order/progress",
                json=payload,
                timeout=3
            )
            if response.status_code == 200:
                self.get_logger().info(f"📈 進度已回報: {stage} - {message}")
        except Exception as e:
            self.get_logger().debug(f"回報進度錯誤: {e}")
    
    def complete_order(self, order_id: str, status: str = "success", details: str = ""):
        """完成訂單"""
        try:
            payload = {
                "order_id": order_id,
                "status": status,
                "details": details
            }
            response = requests.post(
                f"{self.hospital_base_url}/api/ros2/order/complete",
                json=payload,
                timeout=5
            )
            if response.status_code == 200:
                self.get_logger().info(f"✅ 訂單 {order_id} 完成狀態已回報")
                return True
            else:
                self.get_logger().error(f"❌ 回報完成失敗: {response.status_code}")
                return False
        except Exception as e:
            self.get_logger().error(f"完成訂單時發生錯誤: {e}")
            return False


def print_usage_guide():
    """顯示使用指南"""
    print("\n" + "="*80)
    print("🏥 醫院ROS2系統 - 完整使用指南")
    print("="*80)
    
    print("\n📦 系統架構:")
    print("-" * 40)
    print("1. 醫院系統 (clean_hospital_system.py) - 主系統")
    print("2. 自動推送器 (ros2_auto_pusher.py) - 自動推送訂單")
    print("3. 藥物查詢服務 (ros2_medicine_query.py) - 處理藥物查詢")
    print("4. 您的ROS2節點 - 接收訂單和查詢藥物")
    
    print("\n🚀 啟動順序:")
    print("-" * 40)
    print("1. 啟動醫院系統:")
    print("   python3 clean_hospital_system.py")
    print()
    print("2. 啟動自動推送器:")
    print("   python3 ros2_auto_pusher.py")
    print()
    print("3. 啟動藥物查詢服務:")
    print("   python3 ros2_medicine_query.py")
    print()
    print("4. 啟動您的ROS2節點 (本演示):")
    print("   python3 ros2_usage_guide.py")
    
    print("\n📡 ROS2 Topics 說明:")
    print("-" * 40)
    print("接收 (您需要訂閱):")
    print("  /hospital/new_order              - 新訂單推送")
    print("  /hospital/medicine_basic_response - 基礎藥物資訊回應")
    print("  /hospital/medicine_detail_response- 詳細藥物資訊回應")
    print()
    print("發送 (您需要發布):")
    print("  /hospital/query_medicine_basic   - 查詢基礎藥物資訊")
    print("  /hospital/query_medicine_detail  - 查詢詳細藥物資訊")
    
    print("\n🔄 工作流程:")
    print("-" * 40)
    print("1. 自動推送器檢查新處方籤")
    print("2. 推送訂單到 /hospital/new_order")
    print("3. 您的節點接收訂單")
    print("4. 查詢藥物資訊 (基礎/詳細)")
    print("5. 處理每個藥物")
    print("6. 回報處理進度")
    print("7. 完成訂單")
    
    print("\n💊 處方籤ID說明:")
    print("-" * 40)
    print("• 每筆處方籤都有唯一的ID")
    print("• 訂單ID格式: 000001, 000002, ...")
    print("• prescription_id 為原始處方籤編號")
    print("• 可在處方籤管理頁面查看: http://localhost:8001/prescription.html")
    
    print("\n🔧 手動測試指令:")
    print("-" * 40)
    print("# 監聽新訂單")
    print("ros2 topic echo /hospital/new_order")
    print()
    print("# 查詢基礎藥物資訊")
    print("ros2 topic pub /hospital/query_medicine_basic std_msgs/String \"data: 'Aspirin'\"")
    print()
    print("# 監聽基礎資訊回應")
    print("ros2 topic echo /hospital/medicine_basic_response")
    print()
    print("# 查詢詳細藥物資訊")
    print("ros2 topic pub /hospital/query_medicine_detail std_msgs/String \"data: 'Aspirin'\"")
    print()
    print("# 監聽詳細資訊回應")
    print("ros2 topic echo /hospital/medicine_detail_response")
    
    print("\n📋 API端點:")
    print("-" * 40)
    print("訂單進度回報: POST /api/ros2/order/progress")
    print("訂單完成回報: POST /api/ros2/order/complete")
    print("基礎藥物查詢: GET /api/ros2/medicine/basic/{name}")
    print("詳細藥物查詢: GET /api/ros2/medicine/detailed/{name}")
    
    print("\n🎯 重要提醒:")
    print("-" * 40)
    print("✅ 處方籤會自動有ID")
    print("✅ 基本和詳細資訊可分開查詢")
    print("✅ 詳細內容現在為可選項目")
    print("✅ 一次處理一個訂單")
    print("✅ 支援位置格式 1-2, 2-1")
    print("✅ 醫師使用下拉選擇藥物")
    
    print("="*80)


def main(args=None):
    """主函數"""
    rclpy.init(args=args)
    
    # 顯示使用指南
    print_usage_guide()
    
    # 啟動演示節點
    demo_node = HospitalSystemDemo()
    
    print("\n🎭 演示節點已啟動，等待接收訂單...")
    print("💡 請確保其他組件正在運行:")
    print("   - 醫院系統: python3 clean_hospital_system.py")
    print("   - 自動推送器: python3 ros2_auto_pusher.py")
    print("   - 藥物查詢服務: python3 ros2_medicine_query.py")
    print()
    print("🏥 然後到 http://localhost:8001/doctor.html 開立處方籤")
    print("📋 或到 http://localhost:8001/prescription.html 查看狀態")
    
    try:
        rclpy.spin(demo_node)
    except KeyboardInterrupt:
        demo_node.get_logger().info("🛑 演示節點已停止")
    finally:
        demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()