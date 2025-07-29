import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped, Point, Quaternion, Vector3
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from tm_robot_if.srv import PoseSrv, CaptureImage
import numpy as np
import cv2
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from scipy import ndimage
from sklearn.cluster import DBSCAN
import logging
import yaml
from pathlib import Path
from .smart_object_classifier import SmartObjectClassifier

class EnhancedSmallObjectGraspNode(Node):
    """
    增強型細小物件抓取檢測節點
    專門處理藥物等細小物件的吸取點和姿態檢測
    """
    
    def __init__(self, config_path: str = None):
        super().__init__('enhanced_small_object_grasp_node')
        self.bridge = CvBridge()
        
        # 載入配置文件
        if config_path is None:
            config_path = Path(__file__).parent / "small_object_config.yaml"
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        # 初始化智能分類器
        self.object_classifier = SmartObjectClassifier(config_path)
        
        # 相機內參
        camera_config = self.config['camera']
        intrinsic = camera_config['intrinsic']
        
        self.K = np.array([[intrinsic['fx'], 0.0, intrinsic['cx']],
                           [0.0, intrinsic['fy'], intrinsic['cy']],
                           [0.0, 0.0, 1.0]])
        
        self.dist_coeffs = np.array(camera_config['distortion'])
        
        self.intrinsic = intrinsic
        
        # 動態物件參數（將由分類器確定）
        self.current_object_params = self.config['detection']['general']
        self.current_grasp_strategy = None
        
        # 狀態變量
        self.latest_depth = None
        self.latest_color = None
        self.latest_mask = None
        self.pose_history = []
        self.confidence_threshold = self.config['scoring']['min_acceptance_score']
        
        # ROS設置
        self.depth_sub = self.create_subscription(
            Image, '/tm_robot/depth_image', self.depth_callback, 10)
        self.color_sub = self.create_subscription(
            Image, '/tm_robot/color_image', self.color_callback, 10)
        
        self.grasp_service = self.create_service(
            CaptureImage, 'enhanced_grab_detect', self.handle_grasp_request)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/enhanced_grasp_pose', 10)
        self.br = TransformBroadcaster(self)
        self.pose_client = self.create_client(PoseSrv, 'thing_pose')
        
        self.get_logger().info("Enhanced Small Object Grasp Node initialized")
    
    def depth_callback(self, msg):
        """深度圖像回調"""
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    def color_callback(self, msg):
        """彩色圖像回調"""
        self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def handle_grasp_request(self, request, response):
        """處理抓取請求"""
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            self.get_logger().info("接收到mask，開始進行細小物件抓取分析")
            
            success, confidence = self.analyze_and_generate_grasp_pose()
            
            response.success = success
            if success:
                self.get_logger().info(f"抓取分析成功，置信度: {confidence:.3f}")
            else:
                self.get_logger().warn("抓取分析失敗")
                
        except Exception as e:
            self.get_logger().error(f"處理抓取請求失敗: {str(e)}")
            response.success = False
        
        return response
    
    def analyze_and_generate_grasp_pose(self):
        """分析並生成抓取姿態"""
        if not self._validate_inputs():
            return False, 0.0
        
        try:
            # 1. 前處理和校正
            depth_corrected, mask_corrected = self._preprocess_images()
            
            # 2. 智能物件分類和策略選擇
            classification_result = self.object_classifier.classify_object(
                mask_corrected, depth_corrected, self.latest_color)
            
            self.current_object_params = classification_result['parameters']
            self.current_grasp_strategy = classification_result['strategy']
            
            self.get_logger().info(
                f"物件分類: {classification_result['object_type']}, "
                f"置信度: {classification_result['confidence']:.3f}")
            
            # 3. 物件分析
            object_info = self._analyze_object_properties(mask_corrected, depth_corrected)
            if object_info is None:
                return False, 0.0
            
            # 添加分類信息到物件信息中
            object_info['classification'] = classification_result
            
            # 4. 生成候選抓取點（基於智能策略）
            grasp_candidates = self._generate_intelligent_grasp_candidates(
                object_info, mask_corrected, depth_corrected)
            if not grasp_candidates:
                return False, 0.0
            
            # 5. 選擇最佳抓取點
            best_grasp = self._select_best_grasp(grasp_candidates, object_info)
            if best_grasp is None:
                return False, 0.0
            
            # 6. 生成最終姿態並發布
            success = self._generate_final_pose(best_grasp, object_info)
            
            return success, best_grasp['confidence']
            
        except Exception as e:
            self.get_logger().error(f"抓取分析過程錯誤: {str(e)}")
            return False, 0.0
    
    def _validate_inputs(self):
        """驗證輸入數據"""
        if self.latest_depth is None:
            self.get_logger().warn("缺少深度圖像")
            return False
        if self.latest_mask is None:
            self.get_logger().warn("缺少物件mask")
            return False
        return True
    
    def _preprocess_images(self):
        """圖像前處理和相機校正"""
        # 深度圖像去畸變
        depth_corrected = cv2.undistort(
            self.latest_depth.astype(np.float32), self.K, self.dist_coeffs)
        
        # Mask去畸變
        mask_corrected = cv2.undistort(
            self.latest_mask, self.K, self.dist_coeffs)
        
        # 深度圖像濾波降噪
        depth_corrected = cv2.bilateralFilter(
            depth_corrected, 5, 50, 50)
        
        # Mask形態學操作
        kernel = np.ones((3,3), np.uint8)
        mask_corrected = cv2.morphologyEx(
            mask_corrected, cv2.MORPH_CLOSE, kernel)
        mask_corrected = cv2.morphologyEx(
            mask_corrected, cv2.MORPH_OPEN, kernel)
        
        return depth_corrected, mask_corrected
    
    def _analyze_object_properties(self, mask, depth):
        """分析物件屬性"""
        # 確保mask為二值圖像
        _, mask_binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        
        # 計算物件面積
        object_area = np.sum(mask_binary > 0)
        min_area = self.current_object_params.get('min_area', 100)
        max_area = self.current_object_params.get('max_area', 5000)
        
        if object_area < min_area:
            self.get_logger().warn(f"物件面積太小: {object_area} < {min_area}")
            return None
        
        if object_area > max_area:
            self.get_logger().warn(f"物件面積太大: {object_area} > {max_area}")
            return None
        
        # 獲取物件輪廓
        contours, _ = cv2.findContours(mask_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn("未找到有效輪廓")
            return None
        
        # 選擇最大輪廓
        main_contour = max(contours, key=cv2.contourArea)
        
        # 計算幾何特徵
        moments = cv2.moments(main_contour)
        if moments['m00'] == 0:
            return None
        
        # 質心
        centroid_2d = (int(moments['m10'] / moments['m00']), 
                      int(moments['m01'] / moments['m00']))
        
        # 邊界框
        bbox = cv2.boundingRect(main_contour)
        
        # 橢圓擬合
        if len(main_contour) >= 5:
            ellipse = cv2.fitEllipse(main_contour)
        else:
            ellipse = None
        
        # 計算3D點雲
        mask_3d = mask_binary > 0
        depth_valid = depth > 0
        valid_mask = mask_3d & depth_valid
        
        min_depth_points = self.current_object_params.get('min_depth_points', 20)
        if np.sum(valid_mask) < min_depth_points:
            self.get_logger().warn(f"有效深度點太少: {np.sum(valid_mask)}")
            return None
        
        # 轉換到3D空間
        points_3d = self._image_to_3d_points(valid_mask, depth)
        
        return {
            'area': object_area,
            'centroid_2d': centroid_2d,
            'bbox': bbox,
            'contour': main_contour,
            'ellipse': ellipse,
            'points_3d': points_3d,
            'mask_3d': valid_mask,
            'depth_stats': {
                'mean': np.mean(depth[valid_mask]),
                'std': np.std(depth[valid_mask]),
                'min': np.min(depth[valid_mask]),
                'max': np.max(depth[valid_mask])
            }
        }
    
    def _image_to_3d_points(self, mask, depth):
        """將圖像坐標轉換為3D點"""
        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']
        
        # 獲取像素坐標
        ys, xs = np.where(mask)
        zs = depth[mask] * 0.001  # 轉換為米
        
        # 投影到3D
        x3d = (xs - cx) * zs / fx
        y3d = (ys - cy) * zs / fy
        
        return np.column_stack((x3d, y3d, zs))
    
    def _generate_intelligent_grasp_candidates(self, object_info, mask, depth):
        """基於智能分類策略生成抓取候選點"""
        candidates = []
        
        if self.current_grasp_strategy is None:
            return self._generate_grasp_candidates(object_info, mask, depth)
        
        # 獲取推薦的主要方法
        primary_methods = self.current_grasp_strategy.get('primary_methods', ['centroid'])
        backup_methods = self.current_grasp_strategy.get('backup_methods', ['centroid'])
        
        # 根據推薦策略生成候選點
        for method in primary_methods:
            if method == 'centroid':
                candidate = self._generate_centroid_grasp(object_info)
            elif method == 'highest_point':
                candidate = self._generate_highest_point_grasp(object_info)
            elif method == 'stable_region':
                candidates.extend(self._generate_stable_region_grasps(object_info, mask))
                continue
            elif method == 'ellipse_axis':
                if object_info.get('ellipse'):
                    candidates.extend(self._generate_ellipse_based_grasps(object_info))
                continue
            else:
                continue
            
            if candidate:
                # 根據策略調整參數
                suction_offset = self.current_grasp_strategy.get('suction_offset', 0.005)
                candidate['position'][2] += suction_offset - 0.005  # 調整偏移
                candidates.append(candidate)
        
        # 如果主要方法失敗，嘗試備用方法
        if not candidates:
            for method in backup_methods:
                if method == 'centroid':
                    candidate = self._generate_centroid_grasp(object_info)
                    if candidate:
                        candidates.append(candidate)
                        break
        
        # 考慮特殊情況
        special_considerations = self.current_grasp_strategy.get('special_considerations', [])
        if 'very_thin' in special_considerations:
            # 對於非常薄的物件，增加額外的偏移
            for candidate in candidates:
                candidate['position'][2] += 0.002  # 額外2mm偏移
        
        return candidates
    
    def _generate_grasp_candidates(self, object_info, mask, depth):
        """生成抓取候選點"""
        candidates = []
        
        # 方法1: 基於質心的抓取
        centroid_grasp = self._generate_centroid_grasp(object_info)
        if centroid_grasp:
            candidates.append(centroid_grasp)
        
        # 方法2: 基於最高點的抓取（適合細小物件）
        highest_point_grasp = self._generate_highest_point_grasp(object_info)
        if highest_point_grasp:
            candidates.append(highest_point_grasp)
        
        # 方法3: 基於穩定區域的抓取
        stable_grasps = self._generate_stable_region_grasps(object_info, mask)
        candidates.extend(stable_grasps)
        
        # 方法4: 基於橢圓長軸的抓取
        if object_info['ellipse']:
            ellipse_grasps = self._generate_ellipse_based_grasps(object_info)
            candidates.extend(ellipse_grasps)
        
        return candidates
    
    def _generate_centroid_grasp(self, object_info):
        """基於質心生成抓取點"""
        try:
            points_3d = object_info['points_3d']
            centroid_3d = np.mean(points_3d, axis=0)
            
            # 添加吸嘴偏移
            grasp_point = centroid_3d.copy()
            suction_offset = self.current_grasp_strategy.get('suction_offset', 0.005) if self.current_grasp_strategy else 0.005
            grasp_point[2] += suction_offset
            
            # 簡單的向下吸取姿態
            orientation = self._compute_downward_orientation()
            
            return {
                'type': 'centroid',
                'position': grasp_point,
                'orientation': orientation,
                'confidence': 0.7,
                'approach_vector': np.array([0, 0, -1])
            }
        except Exception as e:
            self.get_logger().warn(f"質心抓取生成失敗: {e}")
            return None
    
    def _generate_highest_point_grasp(self, object_info):
        """基於最高點生成抓取點"""
        try:
            points_3d = object_info['points_3d']
            # 找到Z值最大的點（最接近相機）
            highest_idx = np.argmin(points_3d[:, 2])
            highest_point = points_3d[highest_idx]
            
            # 添加偏移
            grasp_point = highest_point.copy()
            suction_offset = self.current_grasp_strategy.get('suction_offset', 0.005) if self.current_grasp_strategy else 0.005
            grasp_point[2] += suction_offset
            
            # 計算局部法向量
            orientation = self._compute_local_normal_orientation(
                points_3d, highest_idx)
            
            return {
                'type': 'highest_point',
                'position': grasp_point,
                'orientation': orientation,
                'confidence': 0.8,
                'approach_vector': np.array([0, 0, -1])
            }
        except Exception as e:
            self.get_logger().warn(f"最高點抓取生成失敗: {e}")
            return None
    
    def _generate_stable_region_grasps(self, object_info, mask):
        """基於穩定區域生成多個抓取點"""
        candidates = []
        try:
            # 使用形態學操作找到穩定區域
            kernel = np.ones((5,5), np.uint8)
            eroded = cv2.erode(mask, kernel, iterations=1)
            
            if np.sum(eroded > 0) < 10:
                return candidates
            
            # 找到穩定區域的中心點
            stable_points = np.where(eroded > 0)
            if len(stable_points[0]) == 0:
                return candidates
            
            # 聚類找到多個穩定區域
            pixel_coords = np.column_stack((stable_points[1], stable_points[0]))
            if len(pixel_coords) > 5:
                clustering = DBSCAN(eps=10, min_samples=3).fit(pixel_coords)
                labels = clustering.labels_
                
                for label in set(labels):
                    if label == -1:  # 噪聲點
                        continue
                    
                    cluster_points = pixel_coords[labels == label]
                    center = np.mean(cluster_points, axis=0).astype(int)
                    
                    # 轉換到3D
                    if (0 <= center[1] < mask.shape[0] and 
                        0 <= center[0] < mask.shape[1]):
                        
                        grasp_3d = self._pixel_to_3d(center, object_info)
                        if grasp_3d is not None:
                            candidates.append({
                                'type': 'stable_region',
                                'position': grasp_3d,
                                'orientation': self._compute_downward_orientation(),
                                'confidence': 0.6,
                                'approach_vector': np.array([0, 0, -1])
                            })
            
        except Exception as e:
            self.get_logger().warn(f"穩定區域抓取生成失敗: {e}")
        
        return candidates
    
    def _generate_ellipse_based_grasps(self, object_info):
        """基於橢圓擬合生成抓取點"""
        candidates = []
        try:
            ellipse = object_info['ellipse']
            center, (major_axis, minor_axis), angle = ellipse
            
            # 在橢圓長軸和短軸方向生成抓取點
            for axis_angle in [angle, angle + 90]:
                rad = np.radians(axis_angle)
                
                # 沿軸方向偏移
                offset_x = 0.3 * major_axis * np.cos(rad)
                offset_y = 0.3 * major_axis * np.sin(rad)
                
                grasp_pixel = (int(center[0] + offset_x), 
                              int(center[1] + offset_y))
                
                grasp_3d = self._pixel_to_3d(grasp_pixel, object_info)
                if grasp_3d is not None:
                    candidates.append({
                        'type': 'ellipse_axis',
                        'position': grasp_3d,
                        'orientation': self._compute_axis_aligned_orientation(axis_angle),
                        'confidence': 0.65,
                        'approach_vector': np.array([0, 0, -1])
                    })
                    
        except Exception as e:
            self.get_logger().warn(f"橢圓抓取生成失敗: {e}")
        
        return candidates
    
    def _pixel_to_3d(self, pixel, object_info):
        """將像素坐標轉換為3D點"""
        try:
            x, y = pixel
            if not (0 <= y < self.latest_depth.shape[0] and 
                   0 <= x < self.latest_depth.shape[1]):
                return None
            
            depth_val = self.latest_depth[y, x] * 0.001
            if depth_val <= 0:
                return None
            
            fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
            cx, cy = self.intrinsic['cx'], self.intrinsic['cy']
            
            x3d = (x - cx) * depth_val / fx
            y3d = (y - cy) * depth_val / fy
            suction_offset = self.current_grasp_strategy.get('suction_offset', 0.005) if self.current_grasp_strategy else 0.005
            z3d = depth_val + suction_offset
            
            return np.array([x3d, y3d, z3d])
            
        except Exception:
            return None
    
    def _compute_downward_orientation(self):
        """計算向下的抓取姿態"""
        # 向下抓取的四元數 (相機坐標系)
        rotation = R.from_euler('xyz', [np.pi, 0, 0])
        return rotation.as_quat()
    
    def _compute_local_normal_orientation(self, points_3d, center_idx, radius=0.005):
        """計算局部法向量姿態"""
        try:
            center_point = points_3d[center_idx]
            
            # 找到鄰近點
            distances = np.linalg.norm(points_3d - center_point, axis=1)
            neighbors = points_3d[distances < radius]
            
            if len(neighbors) < 3:
                return self._compute_downward_orientation()
            
            # 主成分分析找到主要方向
            centered = neighbors - center_point
            covariance = np.cov(centered.T)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            
            # 最小特徵值對應的向量作為法向量
            normal = eigenvectors[:, 0]
            
            # 確保法向量向上
            if normal[2] > 0:
                normal = -normal
            
            # 構建旋轉矩陣
            z_axis = normal / np.linalg.norm(normal)
            x_temp = np.array([1, 0, 0])
            if abs(np.dot(z_axis, x_temp)) > 0.9:
                x_temp = np.array([0, 1, 0])
            
            y_axis = np.cross(z_axis, x_temp)
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, z_axis)
            
            rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
            rotation = R.from_matrix(rotation_matrix)
            
            return rotation.as_quat()
            
        except Exception:
            return self._compute_downward_orientation()
    
    def _compute_axis_aligned_orientation(self, angle_deg):
        """計算沿軸對齊的姿態"""
        try:
            base_rotation = R.from_euler('xyz', [np.pi, 0, 0])
            axis_rotation = R.from_euler('z', np.radians(angle_deg))
            combined = base_rotation * axis_rotation
            return combined.as_quat()
        except Exception:
            return self._compute_downward_orientation()
    
    def _select_best_grasp(self, candidates, object_info):
        """選擇最佳抓取點"""
        if not candidates:
            return None
        
        # 為每個候選點計算得分
        for candidate in candidates:
            score = self._evaluate_grasp_candidate(candidate, object_info)
            candidate['final_score'] = score
        
        # 選擇得分最高的候選點
        best_candidate = max(candidates, key=lambda x: x['final_score'])
        
        # 只有當置信度足夠高時才返回
        if best_candidate['final_score'] > 0.5:
            return best_candidate
        
        return None
    
    def _evaluate_grasp_candidate(self, candidate, object_info):
        """評估抓取候選點"""
        score = candidate['confidence']
        
        # 距離質心的懲罰
        points_3d = object_info['points_3d']
        centroid = np.mean(points_3d, axis=0)
        distance_to_centroid = np.linalg.norm(
            candidate['position'] - centroid)
        
        # 距離越近分數越高
        distance_score = max(0, 1.0 - distance_to_centroid / 0.02)
        score *= distance_score
        
        # 高度獎勵（較高的點更容易抓取）
        relative_height = candidate['position'][2] - np.min(points_3d[:, 2])
        height_score = min(1.0, relative_height / 0.01)
        score *= (0.8 + 0.2 * height_score)
        
        # 類型獎勵
        type_bonus = {
            'highest_point': 1.2,
            'centroid': 1.0,
            'stable_region': 0.9,
            'ellipse_axis': 0.8
        }
        score *= type_bonus.get(candidate['type'], 1.0)
        
        return score
    
    def _generate_final_pose(self, best_grasp, object_info):
        """生成並發布最終抓取姿態"""
        try:
            # 創建姿態消息
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'camera_depth_optical_frame'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            
            # 設置位置
            position = best_grasp['position']
            pose_msg.pose.position = Point(
                x=float(position[0]),
                y=float(position[1]), 
                z=float(position[2])
            )
            
            # 設置姿態
            quat = best_grasp['orientation']
            pose_msg.pose.orientation = Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3])
            )
            
            # 發布姿態
            self.pose_pub.publish(pose_msg)
            
            # 發布TF
            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = 'enhanced_grasp_pose'
            tf_msg.transform.translation = Vector3(
                x=position[0], y=position[1], z=position[2])
            tf_msg.transform.rotation = pose_msg.pose.orientation
            self.br.sendTransform(tf_msg)
            
            # 記錄信息
            rotation = R.from_quat(quat)
            rpy = rotation.as_euler('xyz', degrees=True)
            
            self.get_logger().info(
                f"[Enhanced Grasp Pose - {best_grasp['type']}]\n"
                f"  Position: x={position[0]:.4f}, y={position[1]:.4f}, z={position[2]:.4f}\n"
                f"  RPY: roll={rpy[0]:.1f}°, pitch={rpy[1]:.1f}°, yaw={rpy[2]:.1f}°\n"
                f"  Score: {best_grasp['final_score']:.3f}"
            )
            
            # 可選：發送到機械手臂控制服務
            self._send_to_robot_service(pose_msg.pose)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"生成最終姿態失敗: {e}")
            return False
    
    def _send_to_robot_service(self, pose):
        """發送姿態到機械手臂服務"""
        try:
            if not self.pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("機械手臂服務不可用")
                return
            
            req = PoseSrv.Request()
            req.pose = pose
            
            future = self.pose_client.call_async(req)
            future.add_done_callback(self._handle_robot_response)
            
        except Exception as e:
            self.get_logger().error(f"發送到機械手臂服務失敗: {e}")
    
    def _handle_robot_response(self, future):
        """處理機械手臂服務響應"""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("姿態已成功發送到機械手臂")
            else:
                self.get_logger().warn("機械手臂服務返回失敗")
        except Exception as e:
            self.get_logger().error(f"處理機械手臂響應失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSmallObjectGraspNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()