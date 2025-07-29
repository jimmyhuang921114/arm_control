#!/usr/bin/env python3
"""
簡化版藥局系統
- 使用GroundedSAM2進行物品挑選和識別
- 在特定位置使用LLM進行二次確認
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
    工作流程：訂單 → GroundedSAM2物品挑選 → 抓取 → 特定位置LLM確認 → 完成
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
        
        # 工作流程狀態
        self.current_stage = "idle"  # idle, picking, moving, confirming, completed
        
        # ROS通信
        self._setup_ros_communication()
        
        self.get_logger().info("簡化藥局系統啟動完成")
        self.get_logger().info("工作流程: GroundedSAM2物品挑選 → 抓取移動 → 特定位置LLM確認")
    
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
        self.grounded_sam2_client = self.create_client(
            GroundedSAM2Interface, 'grounded_sam2')
        self.enhanced_grasp_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        
        # 等待服務
        self.get_logger().info("等待必要服務...")
        self.grounded_sam2_client.wait_for_service(timeout_sec=10.0)
        self.enhanced_grasp_client.wait_for_service(timeout_sec=10.0)
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
        self.current_stage = "processing"
        self._publish_status("processing")
        
        try:
            medicines = self.current_order.get('medicines', [])
            
            for medicine in medicines:
                medicine_name = medicine.get('name', 'medicine')
                quantity = medicine.get('quantity', 1)
                
                self.get_logger().info(f"開始處理藥物: {medicine_name}")
                
                # 執行完整的藥物處理流程
                success = self._process_single_medicine(medicine_name, quantity)
                
                if success:
                    self.get_logger().info(f"藥物 {medicine_name} 處理完成")
                else:
                    self.get_logger().warn(f"藥物 {medicine_name} 處理失敗")
            
            self.current_stage = "completed"
            self._publish_status("completed")
            self.get_logger().info("訂單處理完成")
            
        except Exception as e:
            self.get_logger().error(f"訂單處理出錯: {e}")
            self.current_stage = "error"
            self._publish_status("error")
        finally:
            self.processing = False
            self.current_order = None
            self.current_stage = "idle"
    
    def _process_single_medicine(self, medicine_name: str, quantity: int) -> bool:
        """處理單個藥物的完整流程"""
        try:
            # 階段1: 使用GroundedSAM2進行物品挑選
            self.current_stage = "picking"
            self._publish_status("picking_with_grounded_sam2")
            self.get_logger().info(f"階段1: 使用GroundedSAM2挑選物品 - {medicine_name}")
            
            detection_result = self._pick_item_with_grounded_sam2(medicine_name)
            
            if not detection_result or not detection_result.get('success'):
                self.get_logger().warn(f"GroundedSAM2無法識別藥物: {medicine_name}")
                return False
            
            self.get_logger().info(f"GroundedSAM2成功識別藥物，置信度: {detection_result.get('confidence', 0):.3f}")
            
            # 階段2: 執行抓取和移動
            self.current_stage = "moving"
            self._publish_status("grasping_and_moving")
            self.get_logger().info(f"階段2: 抓取並移動到確認位置 - {medicine_name}")
            
            grasp_result = self._execute_grasp_and_move(detection_result)
            
            if not grasp_result:
                self.get_logger().warn(f"抓取或移動失敗: {medicine_name}")
                return False
            
            # 階段3: 在特定位置使用LLM確認
            self.current_stage = "confirming"
            self._publish_status("llm_confirmation")
            self.get_logger().info(f"階段3: 在特定位置使用LLM確認 - {medicine_name}")
            
            confirmation_result = self._confirm_with_llm_at_specific_location(medicine_name)
            
            if not confirmation_result:
                self.get_logger().warn(f"LLM確認失敗: {medicine_name}")
                return False
            
            self.get_logger().info(f"LLM確認成功: {medicine_name}")
            
            # 階段4: 完成分發
            self._publish_status("dispensing")
            self.get_logger().info(f"階段4: 完成分發 - {medicine_name}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"處理藥物時出錯: {e}")
            return False
    
    def _pick_item_with_grounded_sam2(self, medicine_name: str) -> Optional[Dict]:
        """階段1: 使用GroundedSAM2進行物品挑選"""
        if self.latest_color_image is None or self.latest_depth_image is None:
            self.get_logger().warn("沒有圖像數據用於物品挑選")
            return None
        
        try:
            self.get_logger().info(f"GroundedSAM2開始識別目標物品: {medicine_name}")
            
            # 準備GroundedSAM2請求
            req = GroundedSAM2Interface.Request()
            req.image = self.bridge.cv2_to_imgmsg(self.latest_color_image, 'bgr8')
            req.depth = self.bridge.cv2_to_imgmsg(self.latest_depth_image, 'passthrough')
            req.prompt = f"{medicine_name} medicine pill tablet"  # 增強識別prompt
            req.confidence_threshold = 0.3
            req.size_threshold = 0.1
            req.selection_mode = "closest"  # 選擇最接近的物品
            
            # 設置時間戳
            now = self.get_clock().now().to_msg()
            req.image.header.stamp = now
            req.image.header.frame_id = 'camera'
            req.depth.header.stamp = now
            req.depth.header.frame_id = 'camera'
            
            # 調用GroundedSAM2服務
            future = self.grounded_sam2_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    self.get_logger().info(
                        f"GroundedSAM2成功識別: {response.label}, 置信度: {response.score:.3f}")
                    return {
                        'success': True,
                        'confidence': response.score,
                        'label': response.label,
                        'mask': response.binary_image,
                        'bbox': response.bbox
                    }
                else:
                    self.get_logger().warn("GroundedSAM2識別失敗")
            
            return {'success': False}
            
        except Exception as e:
            self.get_logger().error(f"GroundedSAM2物品挑選失敗: {e}")
            return None
    
    def _execute_grasp_and_move(self, detection_result: Dict) -> bool:
        """階段2: 執行抓取並移動到確認位置"""
        try:
            if 'mask' not in detection_result:
                self.get_logger().error("缺少遮罩數據，無法執行抓取")
                return False
            
            self.get_logger().info("開始執行抓取動作")
            
            # 使用增強抓取檢測
            req = CaptureImage.Request()
            req.mask = detection_result['mask']
            
            # 調用抓取服務
            future = self.enhanced_grasp_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    self.get_logger().info("抓取成功，移動到LLM確認位置")
                    
                    # 這裡可以添加移動到特定位置的代碼
                    # 例如：移動到固定的LLM確認工作站
                    time.sleep(2)  # 模擬移動時間
                    
                    return True
                else:
                    self.get_logger().warn("抓取失敗")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"抓取和移動失敗: {e}")
            return False
    
    def _confirm_with_llm_at_specific_location(self, medicine_name: str) -> bool:
        """階段3: 在特定位置使用LLM進行確認"""
        try:
            self.get_logger().info(f"在確認位置使用LLM驗證藥物: {medicine_name}")
            
            # 在特定位置重新拍攝圖像用於LLM確認
            # 這裡可以觸發相機在確認位置拍攝新的圖像
            
            # 模擬LLM確認過程
            # 實際實現時，這裡會調用LLM藥物識別系統
            time.sleep(3)  # 模擬LLM處理時間
            
            # 模擬確認結果
            llm_confidence = 0.9  # 實際來自LLM系統
            
            if llm_confidence > 0.8:
                self.get_logger().info(f"LLM確認成功: {medicine_name}, 置信度: {llm_confidence:.3f}")
                return True
            else:
                self.get_logger().warn(f"LLM確認置信度不足: {medicine_name}, 置信度: {llm_confidence:.3f}")
                return False
            
        except Exception as e:
            self.get_logger().error(f"LLM確認失敗: {e}")
            return False
    
    def _publish_status(self, status: str):
        """發布系統狀態"""
        try:
            status_data = {
                'status': status,
                'stage': self.current_stage,
                'timestamp': time.time()
            }
            
            if self.current_order:
                status_data['order_id'] = self.current_order.get('order_id', 'unknown')
            
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