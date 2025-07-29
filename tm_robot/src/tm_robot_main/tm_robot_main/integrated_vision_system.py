#!/usr/bin/env python3
"""
整合視覺識別系統
- 主要使用GroundedSAM2進行藥物識別和分割
- LLM僅用於二次確認
- 整合增強抓取檢測
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from tm_robot_if.srv import GroundedSAM2Interface, CaptureImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import logging


class IntegratedVisionSystem(Node):
    """
    整合視覺識別系統
    負責藥物識別、定位和抓取規劃的完整流程
    """
    
    def __init__(self):
        super().__init__('integrated_vision_system')
        
        # 初始化CV橋接器
        self.bridge = CvBridge()
        
        # 載入配置
        self.config = self._load_config()
        
        # 狀態變量
        self.current_order = None
        self.detected_medicines = []
        self.current_stage = "idle"  # idle, detecting, confirming, grasping
        self.confirmation_required = False
        
        # 圖像緩存
        self.latest_color_image = None
        self.latest_depth_image = None
        self.latest_detection_result = None
        
        # ROS訂閱者
        self.color_sub = self.create_subscription(
            Image, '/tm_robot/color_image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/tm_robot/depth_image', self.depth_callback, 10)
        self.order_sub = self.create_subscription(
            String, '/pharmacy/new_order', self.order_callback, 10)
        
        # ROS發布者
        self.detection_pub = self.create_publisher(
            String, '/vision/detection_result', 10)
        self.status_pub = self.create_publisher(
            String, '/vision/status', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/vision/target_pose', 10)
        
        # 服務客戶端
        self.grounded_sam2_client = self.create_client(
            GroundedSAM2Interface, 'grounded_sam2')
        self.enhanced_grasp_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        
        # 服務提供者
        self.detect_service = self.create_service(
            GroundedSAM2Interface, 'vision/detect_medicine', 
            self.detect_medicine_callback)
        
        # 等待服務可用
        self._wait_for_services()
        
        # 創建定時器進行週期性檢查
        self.check_timer = self.create_timer(1.0, self.periodic_check)
        
        self.get_logger().info("整合視覺識別系統初始化完成")
    
    def _load_config(self) -> Dict:
        """載入配置文件"""
        try:
            config_path = Path(__file__).parent / "vision_config.yaml"
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            else:
                return self._get_default_config()
        except Exception as e:
            self.get_logger().warn(f"載入配置失敗，使用默認配置: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """獲取默認配置"""
        return {
            'detection': {
                'confidence_threshold': 0.3,
                'size_threshold': 0.1,
                'selection_mode': "all",
                'max_detections': 5
            },
            'confirmation': {
                'enable_llm_confirmation': True,
                'confirmation_threshold': 0.8,
                'max_retry_attempts': 3
            },
            'medicine_database': {
                'database_path': "/workspace/extra_package/llm_drug_identification_system/drug_database/",
                'default_medicine': "no000001.yaml"
            },
            'workflow': {
                'auto_proceed': True,
                'wait_timeout': 30.0,
                'retry_on_failure': True
            }
        }
    
    def _wait_for_services(self):
        """等待所需服務可用"""
        services = [
            (self.grounded_sam2_client, 'grounded_sam2'),
            (self.enhanced_grasp_client, 'enhanced_grab_detect')
        ]
        
        for client, service_name in services:
            self.get_logger().info(f"等待服務: {service_name}")
            if not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().warn(f"服務 {service_name} 不可用，系統功能可能受限")
            else:
                self.get_logger().info(f"服務 {service_name} 已連接")
    
    def color_callback(self, msg: Image):
        """彩色圖像回調"""
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg: Image):
        """深度圖像回調"""
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def order_callback(self, msg: String):
        """訂單回調"""
        try:
            order_data = json.loads(msg.data)
            self.current_order = order_data
            self.get_logger().info(f"收到新訂單: {order_data.get('order_id', 'Unknown')}")
            self._start_detection_workflow()
        except json.JSONDecodeError as e:
            self.get_logger().error(f"訂單數據解析失敗: {e}")
    
    def detect_medicine_callback(self, request, response):
        """藥物檢測服務回調"""
        try:
            # 執行藥物檢測
            detection_result = self._perform_detection(
                prompt=request.prompt,
                confidence_threshold=request.confidence_threshold,
                size_threshold=request.size_threshold,
                selection_mode=request.selection_mode
            )
            
            if detection_result:
                response.success = True
                response.label = detection_result.get('label', '')
                response.score = detection_result.get('score', 0.0)
                response.bbox = detection_result.get('bbox', [])
                if 'binary_image' in detection_result:
                    response.binary_image = detection_result['binary_image']
                
                self.get_logger().info(f"檢測成功: {response.label} (置信度: {response.score:.3f})")
            else:
                response.success = False
                self.get_logger().warn("藥物檢測失敗")
                
        except Exception as e:
            self.get_logger().error(f"檢測服務處理失敗: {e}")
            response.success = False
        
        return response
    
    def _start_detection_workflow(self):
        """開始檢測工作流程"""
        if not self.current_order:
            self.get_logger().warn("沒有有效訂單，無法開始檢測")
            return
        
        self.current_stage = "detecting"
        self.detected_medicines = []
        
        # 發布狀態更新
        self._publish_status("開始藥物檢測")
        
        # 獲取訂單中的藥物列表
        medicines_to_detect = self.current_order.get('medicines', [])
        
        if not medicines_to_detect:
            self.get_logger().warn("訂單中沒有藥物信息")
            return
        
        # 依次檢測每種藥物
        for medicine in medicines_to_detect:
            self._detect_single_medicine(medicine)
    
    def _detect_single_medicine(self, medicine_info: Dict):
        """檢測單個藥物"""
        if not self.latest_color_image is None and not self.latest_depth_image is None:
            self.get_logger().warn("沒有可用的圖像數據")
            return
        
        medicine_name = medicine_info.get('name', 'medicine')
        quantity = medicine_info.get('quantity', 1)
        
        self.get_logger().info(f"檢測藥物: {medicine_name} (需要數量: {quantity})")
        
        # 使用GroundedSAM2進行主要檢測
        detection_result = self._perform_detection(
            prompt=medicine_name,
            confidence_threshold=self.config['detection']['confidence_threshold'],
            size_threshold=self.config['detection']['size_threshold'],
            selection_mode=self.config['detection']['selection_mode']
        )
        
        if detection_result and detection_result['success']:
            # 檢測成功，準備進行確認（如果需要）
            medicine_detection = {
                'medicine_info': medicine_info,
                'detection_result': detection_result,
                'status': 'detected',
                'timestamp': time.time()
            }
            
            # 檢查是否需要LLM二次確認
            if (self.config['confirmation']['enable_llm_confirmation'] and 
                detection_result['score'] < self.config['confirmation']['confirmation_threshold']):
                
                self.get_logger().info(f"檢測置信度較低({detection_result['score']:.3f})，啟動LLM二次確認")
                self._perform_llm_confirmation(medicine_detection)
            else:
                # 直接接受檢測結果
                medicine_detection['status'] = 'confirmed'
                self.detected_medicines.append(medicine_detection)
                self._proceed_to_grasping(medicine_detection)
        else:
            self.get_logger().warn(f"未能檢測到藥物: {medicine_name}")
    
    def _perform_detection(self, prompt: str, confidence_threshold: float, 
                          size_threshold: float, selection_mode: str) -> Optional[Dict]:
        """執行GroundedSAM2檢測"""
        try:
            if self.latest_color_image is None or self.latest_depth_image is None:
                self.get_logger().warn("沒有可用的圖像數據")
                return None
            
            # 準備請求
            req = GroundedSAM2Interface.Request()
            req.image = self.bridge.cv2_to_imgmsg(self.latest_color_image, 'bgr8')
            req.depth = self.bridge.cv2_to_imgmsg(self.latest_depth_image, 'passthrough')
            req.prompt = prompt
            req.confidence_threshold = confidence_threshold
            req.size_threshold = size_threshold
            req.selection_mode = selection_mode
            
            # 設置標頭
            now = self.get_clock().now().to_msg()
            req.image.header.stamp = now
            req.image.header.frame_id = 'camera'
            req.depth.header.stamp = now
            req.depth.header.frame_id = 'camera'
            
            # 呼叫服務
            future = self.grounded_sam2_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response and hasattr(response, 'success') and response.success:
                    return {
                        'success': True,
                        'label': response.label,
                        'score': response.score,
                        'bbox': list(response.bbox),
                        'binary_image': response.binary_image,
                        'detection_time': time.time()
                    }
                else:
                    self.get_logger().warn("GroundedSAM2檢測失敗")
                    return {'success': False}
            else:
                self.get_logger().error("GroundedSAM2服務調用超時")
                return None
                
        except Exception as e:
            self.get_logger().error(f"檢測執行失敗: {e}")
            return None
    
    def _perform_llm_confirmation(self, medicine_detection: Dict):
        """執行LLM二次確認"""
        try:
            medicine_info = medicine_detection['medicine_info']
            medicine_name = medicine_info.get('name', 'unknown')
            
            self.get_logger().info(f"開始LLM確認: {medicine_name}")
            
            # 這裡可以調用LLM服務進行確認
            # 由於您已有LLM藥物識別系統，可以整合使用
            confirmation_result = self._call_llm_service(medicine_detection)
            
            if confirmation_result and confirmation_result.get('confirmed', False):
                medicine_detection['status'] = 'confirmed'
                medicine_detection['llm_confidence'] = confirmation_result.get('confidence', 0.0)
                self.detected_medicines.append(medicine_detection)
                self._proceed_to_grasping(medicine_detection)
                
                self.get_logger().info(f"LLM確認成功: {medicine_name}")
            else:
                self.get_logger().warn(f"LLM確認失敗: {medicine_name}")
                # 可以選擇重新檢測或標記為失敗
                
        except Exception as e:
            self.get_logger().error(f"LLM確認過程失敗: {e}")
    
    def _call_llm_service(self, medicine_detection: Dict) -> Optional[Dict]:
        """調用LLM服務進行確認"""
        try:
            # 這裡整合現有的LLM藥物識別系統
            # 可以將檢測到的圖像區域傳送給LLM系統進行確認
            
            # 模擬LLM確認結果（實際實現時需要調用真實的LLM服務）
            # 您可以在這裡整合 extra_package/llm_drug_identification_system
            
            detection_result = medicine_detection['detection_result']
            score = detection_result.get('score', 0.0)
            
            # 簡化的確認邏輯（實際應該調用LLM）
            if score > 0.5:
                return {
                    'confirmed': True,
                    'confidence': min(score + 0.2, 1.0),
                    'method': 'llm_confirmation'
                }
            else:
                return {
                    'confirmed': False,
                    'confidence': score,
                    'method': 'llm_confirmation'
                }
                
        except Exception as e:
            self.get_logger().error(f"LLM服務調用失敗: {e}")
            return None
    
    def _proceed_to_grasping(self, medicine_detection: Dict):
        """進行抓取階段"""
        try:
            detection_result = medicine_detection['detection_result']
            
            if 'binary_image' not in detection_result:
                self.get_logger().error("沒有可用的遮罩圖像進行抓取")
                return
            
            self.current_stage = "grasping"
            self._publish_status("開始抓取規劃")
            
            # 調用增強抓取檢測
            req = CaptureImage.Request()
            req.mask = detection_result['binary_image']
            
            future = self.enhanced_grasp_client.call_async(req)
            future.add_done_callback(
                lambda f: self._handle_grasp_result(f, medicine_detection))
            
        except Exception as e:
            self.get_logger().error(f"抓取規劃失敗: {e}")
    
    def _handle_grasp_result(self, future, medicine_detection: Dict):
        """處理抓取結果"""
        try:
            if future.done():
                response = future.result()
                if response and response.success:
                    medicine_detection['status'] = 'ready_for_grasp'
                    medicine_detection['grasp_time'] = time.time()
                    
                    self.get_logger().info(
                        f"抓取規劃完成: {medicine_detection['medicine_info']['name']}")
                    
                    # 發布檢測結果
                    self._publish_detection_result(medicine_detection)
                    
                else:
                    self.get_logger().warn("抓取規劃失敗")
                    medicine_detection['status'] = 'grasp_failed'
            else:
                self.get_logger().error("抓取規劃服務調用失敗")
                
        except Exception as e:
            self.get_logger().error(f"處理抓取結果失敗: {e}")
    
    def _publish_detection_result(self, medicine_detection: Dict):
        """發布檢測結果"""
        try:
            result_data = {
                'medicine_name': medicine_detection['medicine_info']['name'],
                'detection_score': medicine_detection['detection_result']['score'],
                'status': medicine_detection['status'],
                'timestamp': medicine_detection['timestamp'],
                'bbox': medicine_detection['detection_result']['bbox']
            }
            
            if 'llm_confidence' in medicine_detection:
                result_data['llm_confidence'] = medicine_detection['llm_confidence']
            
            msg = String()
            msg.data = json.dumps(result_data)
            self.detection_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"發布檢測結果失敗: {e}")
    
    def _publish_status(self, status: str):
        """發布系統狀態"""
        try:
            status_data = {
                'stage': self.current_stage,
                'status': status,
                'timestamp': time.time(),
                'order_id': self.current_order.get('order_id', '') if self.current_order else ''
            }
            
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"發布狀態失敗: {e}")
    
    def periodic_check(self):
        """週期性檢查系統狀態"""
        # 檢查是否有停滯的任務
        if self.current_stage != "idle":
            # 可以添加超時檢查等邏輯
            pass
    
    def get_detection_summary(self) -> Dict:
        """獲取檢測摘要"""
        return {
            'total_detected': len(self.detected_medicines),
            'confirmed': len([m for m in self.detected_medicines if m['status'] == 'confirmed']),
            'ready_for_grasp': len([m for m in self.detected_medicines if m['status'] == 'ready_for_grasp']),
            'current_stage': self.current_stage,
            'order_id': self.current_order.get('order_id', '') if self.current_order else ''
        }


def main(args=None):
    rclpy.init(args=args)
    
    vision_system = IntegratedVisionSystem()
    
    try:
        rclpy.spin(vision_system)
    except KeyboardInterrupt:
        pass
    finally:
        vision_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()