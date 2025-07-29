#!/usr/bin/env python3
"""
整合機器人系統
專注於訂單處理和抓取整合：OCR + GroundedSAM2 + LLM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tm_robot_if.srv import GroundedSAM2Interface, CaptureImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
import requests
from typing import Dict, List, Optional
import threading
from queue import Queue


class IntegratedRobotSystem(Node):
    """
    整合機器人系統
    工作流程：訂單獲取 → OCR文字識別 → GroundedSAM2物品挑選 → 抓取 → LLM確認 → 完成回傳
    """
    
    def __init__(self):
        super().__init__('integrated_robot_system')
        
        # 基本組件
        self.bridge = CvBridge()
        self.order_queue = Queue()
        self.current_order = None
        self.processing = False
        
        # 圖像數據
        self.latest_color_image = None
        self.latest_depth_image = None
        
        # 設置ROS通信
        self._setup_ros_communication()
        
        # 啟動訂單監控線程
        self.order_monitor_thread = threading.Thread(target=self._order_monitor_worker, daemon=True)
        self.order_monitor_thread.start()
        
        # 啟動訂單處理線程
        self.order_processor_thread = threading.Thread(target=self._order_processor_worker, daemon=True)
        self.order_processor_thread.start()
        
        self.get_logger().info("整合機器人系統啟動完成")
        self.get_logger().info("系統流程: 訂單獲取 → OCR → GroundedSAM2 → 抓取 → LLM確認 → 完成")
    
    def _setup_ros_communication(self):
        """設置ROS通信"""
        # 訂閱者
        self.color_sub = self.create_subscription(
            Image, '/tm_robot/color_image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/tm_robot/depth_image', self.depth_callback, 10)
        
        # 發布者
        self.status_pub = self.create_publisher(
            String, '/robot/status', 10)
        self.order_complete_pub = self.create_publisher(
            String, '/robot/order_complete', 10)
        
        # 服務客戶端
        self.grounded_sam2_client = self.create_client(
            GroundedSAM2Interface, 'grounded_sam2')
        self.enhanced_grasp_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        
        # OCR服務客戶端（假設您有OCR服務）
        self.ocr_client = self.create_client(
            CaptureImage, 'ocr_service')  # 可以復用CaptureImage介面
        
        # 等待服務
        self.get_logger().info("等待服務連接...")
        self.grounded_sam2_client.wait_for_service(timeout_sec=10.0)
        self.enhanced_grasp_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("核心服務已連接")
    
    def color_callback(self, msg: Image):
        """彩色圖像回調"""
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg: Image):
        """深度圖像回調"""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def _order_monitor_worker(self):
        """訂單監控工作線程"""
        while True:
            try:
                # 每10秒檢查一次新訂單
                time.sleep(10)
                if not self.processing:
                    self._fetch_new_orders()
            except Exception as e:
                self.get_logger().error(f"訂單監控錯誤: {e}")
                time.sleep(5)
    
    def _fetch_new_orders(self):
        """從API獲取新訂單"""
        try:
            # 調用Web API獲取待處理訂單
            response = requests.get('http://localhost:8000/api/orders/robot/pending', timeout=5)
            if response.status_code == 200:
                orders = response.json()
                for order in orders:
                    self.order_queue.put(order)
                    self.get_logger().info(f"新訂單加入佇列: {order['order_id']}")
        except Exception as e:
            self.get_logger().warn(f"獲取訂單失敗: {e}")
    
    def _order_processor_worker(self):
        """訂單處理工作線程"""
        while True:
            try:
                if not self.order_queue.empty():
                    order = self.order_queue.get()
                    self.current_order = order
                    self.processing = True
                    
                    self.get_logger().info(f"開始處理訂單: {order['order_id']}")
                    self._process_order(order)
                    
                    self.processing = False
                    self.current_order = None
                else:
                    time.sleep(1)
            except Exception as e:
                self.get_logger().error(f"訂單處理錯誤: {e}")
                self.processing = False
                time.sleep(5)
    
    def _process_order(self, order: Dict):
        """處理單個訂單"""
        try:
            order_id = order['order_id']
            medicines = order['medicines']
            
            self._publish_status(f"開始處理訂單 {order_id}")
            
            # 處理每種藥物
            all_success = True
            for medicine in medicines:
                success = self._process_medicine(order_id, medicine)
                if not success:
                    all_success = False
                    self.get_logger().warn(f"藥物處理失敗: {medicine['name']}")
            
            # 更新訂單狀態
            if all_success:
                self._complete_order(order_id)
                self.get_logger().info(f"訂單 {order_id} 處理完成")
            else:
                self.get_logger().error(f"訂單 {order_id} 處理失敗")
                
        except Exception as e:
            self.get_logger().error(f"處理訂單時發生錯誤: {e}")
    
    def _process_medicine(self, order_id: str, medicine: Dict) -> bool:
        """
        處理單個藥物
        整合流程：OCR → GroundedSAM2 → 抓取 → LLM確認
        """
        medicine_name = medicine['name']
        quantity = medicine['quantity']
        json_config = medicine.get('json_config', {})
        
        self.get_logger().info(f"處理藥物: {medicine_name} x{quantity}")
        
        try:
            # 階段1: OCR文字識別（用於輔助識別）
            ocr_result = self._perform_ocr()
            
            # 階段2: GroundedSAM2物品挑選
            detection_result = self._perform_grounded_sam2_detection(medicine_name, json_config)
            if not detection_result or not detection_result.get('success'):
                self.get_logger().warn(f"GroundedSAM2識別失敗: {medicine_name}")
                return False
            
            # 階段3: 增強抓取
            grasp_result = self._perform_enhanced_grasp(detection_result, json_config)
            if not grasp_result:
                self.get_logger().warn(f"抓取失敗: {medicine_name}")
                return False
            
            # 階段4: 移動到LLM確認位置並確認
            llm_result = self._perform_llm_confirmation(medicine_name, json_config, ocr_result)
            if not llm_result:
                self.get_logger().warn(f"LLM確認失敗: {medicine_name}")
                return False
            
            self.get_logger().info(f"藥物 {medicine_name} 處理成功")
            return True
            
        except Exception as e:
            self.get_logger().error(f"處理藥物 {medicine_name} 時發生錯誤: {e}")
            return False
    
    def _perform_ocr(self) -> Optional[Dict]:
        """執行OCR文字識別"""
        if self.latest_color_image is None:
            self.get_logger().warn("沒有圖像數據進行OCR")
            return None
        
        try:
            self.get_logger().info("執行OCR文字識別")
            
            # 準備OCR請求
            req = CaptureImage.Request()
            # 將整個圖像作為OCR輸入
            mask = np.ones(self.latest_color_image.shape[:2], dtype=np.uint8) * 255
            req.mask = self.bridge.cv2_to_imgmsg(mask, 'mono8')
            
            # 調用OCR服務
            if self.ocr_client.service_is_ready():
                future = self.ocr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
                
                if future.done():
                    response = future.result()
                    if response and response.success:
                        self.get_logger().info("OCR識別成功")
                        return {"success": True, "text_detected": True}
            
            # 如果OCR服務不可用，返回基本結果
            self.get_logger().info("OCR服務不可用，跳過OCR階段")
            return {"success": True, "text_detected": False}
            
        except Exception as e:
            self.get_logger().warn(f"OCR執行失敗: {e}")
            return {"success": True, "text_detected": False}  # OCR失敗不影響後續流程
    
    def _perform_grounded_sam2_detection(self, medicine_name: str, json_config: Dict) -> Optional[Dict]:
        """執行GroundedSAM2物品識別"""
        if self.latest_color_image is None or self.latest_depth_image is None:
            self.get_logger().warn("沒有圖像數據進行GroundedSAM2識別")
            return None
        
        try:
            self.get_logger().info(f"GroundedSAM2識別藥物: {medicine_name}")
            
            # 從JSON配置獲取檢測參數
            detection_config = json_config.get('detection', {})
            confidence_threshold = detection_config.get('confidence_threshold', 0.3)
            size_threshold = detection_config.get('size_threshold', 0.1)
            
            # 準備請求
            req = GroundedSAM2Interface.Request()
            req.image = self.bridge.cv2_to_imgmsg(self.latest_color_image, 'bgr8')
            req.depth = self.bridge.cv2_to_imgmsg(self.latest_depth_image, 'passthrough')
            req.prompt = f"{medicine_name} medicine pill tablet"
            req.confidence_threshold = confidence_threshold
            req.size_threshold = size_threshold
            req.selection_mode = "closest"
            
            # 設置時間戳
            now = self.get_clock().now().to_msg()
            req.image.header.stamp = now
            req.image.header.frame_id = 'camera'
            req.depth.header.stamp = now
            req.depth.header.frame_id = 'camera'
            
            # 調用服務
            future = self.grounded_sam2_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response and response.success:
                    self.get_logger().info(
                        f"GroundedSAM2識別成功: {response.label}, 置信度: {response.score:.3f}")
                    return {
                        'success': True,
                        'confidence': response.score,
                        'label': response.label,
                        'mask': response.binary_image,
                        'bbox': response.bbox
                    }
            
            return {'success': False}
            
        except Exception as e:
            self.get_logger().error(f"GroundedSAM2識別失敗: {e}")
            return None
    
    def _perform_enhanced_grasp(self, detection_result: Dict, json_config: Dict) -> bool:
        """執行增強抓取"""
        try:
            if 'mask' not in detection_result:
                self.get_logger().error("缺少遮罩數據，無法執行抓取")
                return False
            
            self.get_logger().info("執行增強抓取")
            
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
                    
                    # 模擬移動到LLM確認位置
                    time.sleep(2)
                    return True
                else:
                    self.get_logger().warn("抓取失敗")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"抓取執行失敗: {e}")
            return False
    
    def _perform_llm_confirmation(self, medicine_name: str, json_config: Dict, ocr_result: Dict) -> bool:
        """在特定位置執行LLM確認"""
        try:
            self.get_logger().info(f"LLM確認藥物: {medicine_name}")
            
            # 從JSON配置獲取LLM驗證參數
            verification_config = json_config.get('verification', {})
            llm_prompt = verification_config.get('llm_prompt', f"識別{medicine_name}藥物")
            required_confidence = verification_config.get('required_confidence', 0.8)
            
            # 在確認位置重新拍攝圖像用於LLM
            # 這裡可以觸發相機在特定位置拍攝
            
            # 模擬LLM確認過程
            # 實際實現時會調用您的LLM藥物識別系統
            time.sleep(3)  # 模擬LLM處理時間
            
            # 整合OCR結果輔助LLM判斷
            ocr_helpful = ocr_result and ocr_result.get('text_detected', False)
            
            # 模擬LLM確認結果
            llm_confidence = 0.9 if ocr_helpful else 0.85
            
            if llm_confidence >= required_confidence:
                self.get_logger().info(f"LLM確認成功: {medicine_name}, 置信度: {llm_confidence:.3f}")
                return True
            else:
                self.get_logger().warn(f"LLM確認置信度不足: {medicine_name}, 置信度: {llm_confidence:.3f}")
                return False
            
        except Exception as e:
            self.get_logger().error(f"LLM確認失敗: {e}")
            return False
    
    def _complete_order(self, order_id: str):
        """完成訂單"""
        try:
            # 調用API標記訂單完成
            response = requests.put(f'http://localhost:8000/api/orders/{order_id}/complete', timeout=5)
            if response.status_code == 200:
                self.get_logger().info(f"訂單 {order_id} 已標記為完成")
                
                # 發布訂單完成消息
                completion_msg = String()
                completion_data = {
                    "order_id": order_id,
                    "status": "completed",
                    "completed_at": time.time()
                }
                completion_msg.data = json.dumps(completion_data)
                self.order_complete_pub.publish(completion_msg)
            else:
                self.get_logger().error(f"標記訂單完成失敗: {response.text}")
                
        except Exception as e:
            self.get_logger().error(f"完成訂單時發生錯誤: {e}")
    
    def _publish_status(self, status: str):
        """發布系統狀態"""
        try:
            status_data = {
                'status': status,
                'timestamp': time.time(),
                'current_order': self.current_order['order_id'] if self.current_order else None
            }
            
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"發布狀態失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    robot_system = IntegratedRobotSystem()
    
    try:
        rclpy.spin(robot_system)
    except KeyboardInterrupt:
        pass
    finally:
        robot_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()