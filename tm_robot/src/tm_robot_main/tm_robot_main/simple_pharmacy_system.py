#!/usr/bin/env python3
"""
簡化版藥局系統
只保留基本的藥物識別和抓取功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tm_robot_if.srv import GroundedSAM2Interface, CaptureImage
from cv_bridge import CvBridge
import json
import time
from typing import Dict, Optional


class SimplePharmacySystem(Node):
    """
    簡化版藥局系統
    基本工作流程：訂單 → 視覺識別 → 抓取 → 完成
    """
    
    def __init__(self):
        super().__init__('simple_pharmacy_system')
        
        # 基本組件
        self.bridge = CvBridge()
        self.current_order = None
        self.processing = False
        
        # 圖像數據
        self.latest_color_image = None
        self.latest_depth_image = None
        
        # ROS通信
        self._setup_ros_communication()
        
        self.get_logger().info("簡化藥局系統啟動完成")
    
    def _setup_ros_communication(self):
        """設置ROS通信"""
        # 訂閱者
        self.color_sub = self.create_subscription(
            Image, '/tm_robot/color_image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/tm_robot/depth_image', self.depth_callback, 10)
        self.order_sub = self.create_subscription(
            String, '/pharmacy/new_order', self.order_callback, 10)
        
        # 發布者
        self.status_pub = self.create_publisher(
            String, '/pharmacy/status', 10)
        
        # 服務客戶端
        self.vision_client = self.create_client(
            GroundedSAM2Interface, 'grounded_sam2')
        self.grasp_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        
        # 等待服務
        self.get_logger().info("等待必要服務...")
        self.vision_client.wait_for_service(timeout_sec=10.0)
        self.grasp_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("服務連接完成")
    
    def color_callback(self, msg: Image):
        """彩色圖像回調"""
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg: Image):
        """深度圖像回調"""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def order_callback(self, msg: String):
        """處理新訂單"""
        if self.processing:
            self.get_logger().warn("系統忙碌中，跳過此訂單")
            return
            
        try:
            order_data = json.loads(msg.data)
            self.current_order = order_data
            self.get_logger().info("收到新訂單，開始處理")
            self._process_order()
            
        except Exception as e:
            self.get_logger().error(f"訂單處理失敗: {e}")
    
    def _process_order(self):
        """處理訂單"""
        if not self.current_order:
            return
            
        self.processing = True
        self._publish_status("processing")
        
        try:
            medicines = self.current_order.get('medicines', [])
            
            for medicine in medicines:
                medicine_name = medicine.get('name', 'medicine')
                quantity = medicine.get('quantity', 1)
                
                self.get_logger().info(f"處理藥物: {medicine_name}")
                
                # 執行藥物處理流程
                success = self._process_single_medicine(medicine_name)
                
                if success:
                    self.get_logger().info(f"藥物 {medicine_name} 處理完成")
                else:
                    self.get_logger().warn(f"藥物 {medicine_name} 處理失敗")
            
            self._publish_status("completed")
            self.get_logger().info("訂單處理完成")
            
        except Exception as e:
            self.get_logger().error(f"訂單處理出錯: {e}")
            self._publish_status("error")
        finally:
            self.processing = False
            self.current_order = None
    
    def _process_single_medicine(self, medicine_name: str) -> bool:
        """處理單個藥物"""
        try:
            # 1. 視覺識別
            self._publish_status("detecting")
            detection_result = self._detect_medicine(medicine_name)
            
            if not detection_result or not detection_result.get('success'):
                self.get_logger().warn(f"無法識別藥物: {medicine_name}")
                return False
            
            # 2. 抓取執行
            self._publish_status("grasping")
            grasp_result = self._execute_grasp(detection_result)
            
            if not grasp_result:
                self.get_logger().warn(f"抓取失敗: {medicine_name}")
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"處理藥物時出錯: {e}")
            return False
    
    def _detect_medicine(self, medicine_name: str) -> Optional[Dict]:
        """執行藥物識別"""
        if self.latest_color_image is None or self.latest_depth_image is None:
            self.get_logger().warn("沒有圖像數據")
            return None
        
        try:
            # 準備請求
            req = GroundedSAM2Interface.Request()
            req.image = self.bridge.cv2_to_imgmsg(self.latest_color_image, 'bgr8')
            req.depth = self.bridge.cv2_to_imgmsg(self.latest_depth_image, 'passthrough')
            req.prompt = medicine_name
            req.confidence_threshold = 0.3
            req.size_threshold = 0.1
            req.selection_mode = "closest"
            
            # 設置時間戳
            now = self.get_clock().now().to_msg()
            req.image.header.stamp = now
            req.image.header.frame_id = 'camera'
            req.depth.header.stamp = now
            req.depth.header.frame_id = 'camera'
            
            # 調用服務
            future = self.vision_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    return {
                        'success': True,
                        'confidence': response.score,
                        'mask': response.binary_image
                    }
            
            return {'success': False}
            
        except Exception as e:
            self.get_logger().error(f"視覺識別失敗: {e}")
            return None
    
    def _execute_grasp(self, detection_result: Dict) -> bool:
        """執行抓取"""
        try:
            if 'mask' not in detection_result:
                return False
            
            # 準備抓取請求
            req = CaptureImage.Request()
            req.mask = detection_result['mask']
            
            # 調用抓取服務
            future = self.grasp_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                return response and response.success
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"抓取執行失敗: {e}")
            return False
    
    def _publish_status(self, status: str):
        """發布系統狀態"""
        try:
            status_data = {
                'status': status,
                'timestamp': time.time()
            }
            
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"發布狀態失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    system = SimplePharmacySystem()
    
    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        pass
    finally:
        system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()