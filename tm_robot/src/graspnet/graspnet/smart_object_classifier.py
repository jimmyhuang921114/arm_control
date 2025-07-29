import cv2
import numpy as np
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import logging
from scipy import ndimage
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
import joblib

class SmartObjectClassifier:
    """
    智能物件分類器
    根據視覺特徵自動識別藥物類型並選擇最佳抓取策略
    """
    
    def __init__(self, config_path: str = None):
        """
        初始化分類器
        
        Args:
            config_path: 配置文件路徑
        """
        # 載入配置
        if config_path is None:
            config_path = Path(__file__).parent / "small_object_config.yaml"
        
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)
        
        # 設置日誌
        logging.basicConfig(level=getattr(logging, self.config['debug']['log_level']))
        self.logger = logging.getLogger(__name__)
        
        # 載入預訓練模型（如果存在）
        self.classifier = None
        self.scaler = None
        self._load_pretrained_model()
        
        # 物件類型定義
        self.object_types = {
            'pill': 0,      # 藥丸
            'capsule': 1,   # 膠囊
            'tablet': 2,    # 藥片
            'small_bottle': 3  # 小瓶
        }
        
        self.logger.info("Smart Object Classifier initialized")
    
    def classify_object(self, mask: np.ndarray, depth: np.ndarray, 
                       color_image: np.ndarray = None) -> Dict:
        """
        分類物件並返回推薦的抓取策略
        
        Args:
            mask: 物件遮罩
            depth: 深度圖像
            color_image: 彩色圖像（可選）
            
        Returns:
            包含物件類型和抓取策略的字典
        """
        try:
            # 提取特徵
            features = self._extract_features(mask, depth, color_image)
            if features is None:
                return self._get_default_strategy()
            
            # 分類物件
            object_type = self._classify_by_features(features)
            
            # 獲取對應的抓取策略
            strategy = self._get_grasp_strategy(object_type, features)
            
            return {
                'object_type': object_type,
                'confidence': features.get('classification_confidence', 0.7),
                'features': features,
                'strategy': strategy,
                'parameters': self._get_type_specific_parameters(object_type)
            }
            
        except Exception as e:
            self.logger.error(f"物件分類失敗: {e}")
            return self._get_default_strategy()
    
    def _extract_features(self, mask: np.ndarray, depth: np.ndarray, 
                         color_image: np.ndarray = None) -> Optional[Dict]:
        """提取物件特徵"""
        try:
            # 確保mask為二值圖像
            _, mask_binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
            
            # 基本幾何特徵
            geometric_features = self._extract_geometric_features(mask_binary)
            if geometric_features is None:
                return None
            
            # 深度特徵
            depth_features = self._extract_depth_features(mask_binary, depth)
            
            # 形狀特徵
            shape_features = self._extract_shape_features(mask_binary)
            
            # 紋理特徵（如果有彩色圖像）
            texture_features = {}
            if color_image is not None:
                texture_features = self._extract_texture_features(
                    mask_binary, color_image)
            
            # 合併所有特徵
            features = {
                **geometric_features,
                **depth_features,
                **shape_features,
                **texture_features
            }
            
            return features
            
        except Exception as e:
            self.logger.error(f"特徵提取失敗: {e}")
            return None
    
    def _extract_geometric_features(self, mask: np.ndarray) -> Optional[Dict]:
        """提取幾何特徵"""
        try:
            # 找到輪廓
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return None
            
            # 選擇最大輪廓
            main_contour = max(contours, key=cv2.contourArea)
            
            # 基本測量
            area = cv2.contourArea(main_contour)
            perimeter = cv2.arcLength(main_contour, True)
            
            # 邊界框
            bbox = cv2.boundingRect(main_contour)
            bbox_width, bbox_height = bbox[2], bbox[3]
            bbox_area = bbox_width * bbox_height
            
            # 最小外接矩形
            min_rect = cv2.minAreaRect(main_contour)
            min_rect_area = min_rect[1][0] * min_rect[1][1]
            
            # 橢圓擬合
            if len(main_contour) >= 5:
                ellipse = cv2.fitEllipse(main_contour)
                ellipse_area = np.pi * ellipse[1][0] * ellipse[1][1] / 4
                ellipse_ratio = max(ellipse[1]) / min(ellipse[1])
            else:
                ellipse_area = area
                ellipse_ratio = 1.0
            
            features = {
                'area': area,
                'perimeter': perimeter,
                'bbox_width': bbox_width,
                'bbox_height': bbox_height,
                'bbox_area': bbox_area,
                'min_rect_area': min_rect_area,
                'ellipse_area': ellipse_area,
                'ellipse_ratio': ellipse_ratio,
                
                # 形狀比率
                'aspect_ratio': bbox_width / max(bbox_height, 1),
                'extent': area / max(bbox_area, 1),
                'solidity': area / max(cv2.contourArea(cv2.convexHull(main_contour)), 1),
                'compactness': (4 * np.pi * area) / max(perimeter * perimeter, 1),
                'rectangularity': area / max(min_rect_area, 1),
                'ellipticity': area / max(ellipse_area, 1),
            }
            
            return features
            
        except Exception as e:
            self.logger.error(f"幾何特徵提取失敗: {e}")
            return None
    
    def _extract_depth_features(self, mask: np.ndarray, depth: np.ndarray) -> Dict:
        """提取深度特徵"""
        try:
            # 獲取物件區域的深度值
            valid_mask = (mask > 0) & (depth > 0)
            if not np.any(valid_mask):
                return {}
            
            object_depths = depth[valid_mask] * 0.001  # 轉換為米
            
            # 基本統計
            depth_stats = {
                'depth_mean': np.mean(object_depths),
                'depth_std': np.std(object_depths),
                'depth_min': np.min(object_depths),
                'depth_max': np.max(object_depths),
                'depth_range': np.max(object_depths) - np.min(object_depths),
                'depth_median': np.median(object_depths),
            }
            
            # 高度估算
            estimated_height = depth_stats['depth_range']
            depth_stats['estimated_height'] = estimated_height
            
            # 深度變化模式
            depth_gradient = np.gradient(object_depths)
            depth_stats['depth_gradient_std'] = np.std(depth_gradient)
            
            # 表面粗糙度
            if len(object_depths) > 10:
                depth_stats['surface_roughness'] = np.std(
                    object_depths - np.mean(object_depths))
            else:
                depth_stats['surface_roughness'] = 0.0
            
            return depth_stats
            
        except Exception as e:
            self.logger.error(f"深度特徵提取失敗: {e}")
            return {}
    
    def _extract_shape_features(self, mask: np.ndarray) -> Dict:
        """提取形狀特徵"""
        try:
            # 找到輪廓
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                return {}
            
            main_contour = max(contours, key=cv2.contourArea)
            
            # Hu矩
            moments = cv2.moments(main_contour)
            hu_moments = cv2.HuMoments(moments).flatten()
            
            # 輪廓近似
            epsilon = 0.02 * cv2.arcLength(main_contour, True)
            approx = cv2.approxPolyDP(main_contour, epsilon, True)
            
            # 凸包分析
            hull = cv2.convexHull(main_contour)
            hull_area = cv2.contourArea(hull)
            
            # 最小外接圓
            (_, _), radius = cv2.minEnclosingCircle(main_contour)
            circle_area = np.pi * radius * radius
            
            shape_features = {
                'num_vertices': len(approx),
                'convex_hull_area': hull_area,
                'min_circle_radius': radius,
                'min_circle_area': circle_area,
                'circularity': cv2.contourArea(main_contour) / max(circle_area, 1),
                'convexity': cv2.contourArea(main_contour) / max(hull_area, 1),
            }
            
            # 添加Hu矩作為特徵
            for i, hu in enumerate(hu_moments):
                if not np.isnan(hu) and np.isfinite(hu):
                    shape_features[f'hu_moment_{i}'] = -np.sign(hu) * np.log10(np.abs(hu))
                else:
                    shape_features[f'hu_moment_{i}'] = 0.0
            
            return shape_features
            
        except Exception as e:
            self.logger.error(f"形狀特徵提取失敗: {e}")
            return {}
    
    def _extract_texture_features(self, mask: np.ndarray, 
                                 color_image: np.ndarray) -> Dict:
        """提取紋理特徵"""
        try:
            # 轉換為灰階
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            
            # 獲取物件區域
            object_region = gray[mask > 0]
            if len(object_region) == 0:
                return {}
            
            # 基本統計
            texture_features = {
                'intensity_mean': np.mean(object_region),
                'intensity_std': np.std(object_region),
                'intensity_range': np.max(object_region) - np.min(object_region),
            }
            
            # 局部二值模式 (簡化版)
            if mask.shape[0] > 10 and mask.shape[1] > 10:
                # 獲取物件的最小外接矩形區域
                coords = np.where(mask > 0)
                if len(coords[0]) > 0:
                    y_min, y_max = np.min(coords[0]), np.max(coords[0])
                    x_min, x_max = np.min(coords[1]), np.max(coords[1])
                    
                    roi = gray[y_min:y_max+1, x_min:x_max+1]
                    roi_mask = mask[y_min:y_max+1, x_min:x_max+1]
                    
                    if roi.size > 0:
                        # 簡單的紋理測量
                        texture_features['local_variance'] = np.var(
                            roi[roi_mask > 0])
            
            return texture_features
            
        except Exception as e:
            self.logger.error(f"紋理特徵提取失敗: {e}")
            return {}
    
    def _classify_by_features(self, features: Dict) -> str:
        """根據特徵分類物件"""
        try:
            # 如果有預訓練模型，使用機器學習分類
            if self.classifier is not None and self.scaler is not None:
                return self._ml_classify(features)
            
            # 否則使用基於規則的分類
            return self._rule_based_classify(features)
            
        except Exception as e:
            self.logger.error(f"物件分類失敗: {e}")
            return 'pill'  # 默認為藥丸
    
    def _rule_based_classify(self, features: Dict) -> str:
        """基於規則的分類"""
        try:
            area = features.get('area', 0)
            aspect_ratio = features.get('aspect_ratio', 1.0)
            ellipse_ratio = features.get('ellipse_ratio', 1.0)
            estimated_height = features.get('estimated_height', 0)
            circularity = features.get('circularity', 0)
            
            # 小瓶子：大面積且高
            if (area > 3000 and estimated_height > 0.015):
                return 'small_bottle'
            
            # 膠囊：長寬比較大，橢圓比高
            elif (aspect_ratio > 1.8 or ellipse_ratio > 2.0) and area > 150:
                return 'capsule'
            
            # 藥片：較平但不是很圓
            elif (estimated_height < 0.008 and circularity < 0.8 and 
                  area > 200):
                return 'tablet'
            
            # 藥丸：較圓，中等大小
            else:
                return 'pill'
                
        except Exception as e:
            self.logger.error(f"規則分類失敗: {e}")
            return 'pill'
    
    def _ml_classify(self, features: Dict) -> str:
        """使用機器學習模型分類"""
        try:
            # 準備特徵向量
            feature_vector = self._prepare_feature_vector(features)
            if feature_vector is None:
                return self._rule_based_classify(features)
            
            # 標準化
            feature_scaled = self.scaler.transform([feature_vector])
            
            # 預測
            prediction = self.classifier.predict(feature_scaled)[0]
            probabilities = self.classifier.predict_proba(feature_scaled)[0]
            
            # 獲取類型名稱
            type_names = list(self.object_types.keys())
            predicted_type = type_names[prediction]
            
            # 更新置信度
            confidence = np.max(probabilities)
            features['classification_confidence'] = confidence
            
            self.logger.debug(f"ML分類結果: {predicted_type}, 置信度: {confidence:.3f}")
            
            return predicted_type
            
        except Exception as e:
            self.logger.error(f"ML分類失敗: {e}")
            return self._rule_based_classify(features)
    
    def _prepare_feature_vector(self, features: Dict) -> Optional[np.ndarray]:
        """準備機器學習特徵向量"""
        try:
            # 定義使用的特徵列表
            feature_names = [
                'area', 'aspect_ratio', 'ellipse_ratio', 'estimated_height',
                'circularity', 'convexity', 'solidity', 'extent',
                'depth_range', 'surface_roughness'
            ]
            
            # 提取特徵值
            feature_vector = []
            for name in feature_names:
                value = features.get(name, 0.0)
                if np.isnan(value) or not np.isfinite(value):
                    value = 0.0
                feature_vector.append(value)
            
            return np.array(feature_vector)
            
        except Exception as e:
            self.logger.error(f"特徵向量準備失敗: {e}")
            return None
    
    def _get_grasp_strategy(self, object_type: str, features: Dict) -> Dict:
        """獲取抓取策略"""
        try:
            # 從配置中獲取物件特定參數
            type_config = self.config['object_specific'].get(object_type, {})
            
            # 獲取推薦策略
            preferred_strategies = type_config.get('preferred_strategies', 
                                                 ['centroid', 'highest_point'])
            
            # 根據物件特徵調整策略
            strategy = {
                'primary_methods': preferred_strategies,
                'backup_methods': ['centroid'],  # 總是包含備用方案
                'suction_offset': type_config.get('suction_offset', 0.005),
                'approach_tolerance': type_config.get('approach_angle_tolerance', 20.0),
                'special_considerations': []
            }
            
            # 基於特徵的特殊考慮
            if features.get('aspect_ratio', 1.0) > 2.0:
                strategy['special_considerations'].append('long_object')
                strategy['primary_methods'] = ['stable_region', 'ellipse_axis']
            
            if features.get('estimated_height', 0) < 0.003:
                strategy['special_considerations'].append('very_thin')
                strategy['suction_offset'] = max(0.003, strategy['suction_offset'])
            
            if features.get('circularity', 0) > 0.9:
                strategy['special_considerations'].append('circular')
                strategy['primary_methods'] = ['centroid', 'highest_point']
            
            return strategy
            
        except Exception as e:
            self.logger.error(f"抓取策略獲取失敗: {e}")
            return self._get_default_strategy()['strategy']
    
    def _get_type_specific_parameters(self, object_type: str) -> Dict:
        """獲取類型特定參數"""
        try:
            # 從配置中獲取檢測參數
            detection_params = self.config['detection']['object_types'].get(
                object_type, self.config['detection']['general'])
            
            # 從配置中獲取物件特定參數
            specific_params = self.config['object_specific'].get(object_type, {})
            
            return {
                **detection_params,
                **specific_params
            }
            
        except Exception as e:
            self.logger.error(f"參數獲取失敗: {e}")
            return self.config['detection']['general']
    
    def _get_default_strategy(self) -> Dict:
        """獲取默認策略"""
        return {
            'object_type': 'pill',
            'confidence': 0.5,
            'features': {},
            'strategy': {
                'primary_methods': ['centroid', 'highest_point'],
                'backup_methods': ['centroid'],
                'suction_offset': 0.005,
                'approach_tolerance': 20.0,
                'special_considerations': []
            },
            'parameters': self.config['detection']['general']
        }
    
    def _load_pretrained_model(self):
        """載入預訓練模型"""
        try:
            model_dir = Path(__file__).parent / "models"
            classifier_path = model_dir / "object_classifier.joblib"
            scaler_path = model_dir / "feature_scaler.joblib"
            
            if classifier_path.exists() and scaler_path.exists():
                self.classifier = joblib.load(classifier_path)
                self.scaler = joblib.load(scaler_path)
                self.logger.info("已載入預訓練模型")
            else:
                self.logger.info("未找到預訓練模型，將使用基於規則的分類")
                
        except Exception as e:
            self.logger.warning(f"載入預訓練模型失敗: {e}")
            self.classifier = None
            self.scaler = None
    
    def train_classifier(self, training_data: List[Dict]):
        """訓練分類器"""
        try:
            if len(training_data) < 10:
                self.logger.warning("訓練數據太少，無法訓練有效模型")
                return False
            
            # 準備訓練數據
            X = []
            y = []
            
            for sample in training_data:
                features = sample['features']
                object_type = sample['object_type']
                
                feature_vector = self._prepare_feature_vector(features)
                if feature_vector is not None:
                    X.append(feature_vector)
                    y.append(self.object_types[object_type])
            
            if len(X) < 5:
                self.logger.warning("有效訓練樣本太少")
                return False
            
            X = np.array(X)
            y = np.array(y)
            
            # 標準化特徵
            self.scaler = StandardScaler()
            X_scaled = self.scaler.fit_transform(X)
            
            # 訓練分類器
            self.classifier = RandomForestClassifier(
                n_estimators=100, random_state=42)
            self.classifier.fit(X_scaled, y)
            
            # 保存模型
            self._save_model()
            
            self.logger.info(f"成功訓練分類器，使用 {len(X)} 個樣本")
            return True
            
        except Exception as e:
            self.logger.error(f"訓練分類器失敗: {e}")
            return False
    
    def _save_model(self):
        """保存模型"""
        try:
            model_dir = Path(__file__).parent / "models"
            model_dir.mkdir(exist_ok=True)
            
            classifier_path = model_dir / "object_classifier.joblib"
            scaler_path = model_dir / "feature_scaler.joblib"
            
            joblib.dump(self.classifier, classifier_path)
            joblib.dump(self.scaler, scaler_path)
            
            self.logger.info("模型已保存")
            
        except Exception as e:
            self.logger.error(f"保存模型失敗: {e}")


# 輔助函數
def create_training_sample(mask: np.ndarray, depth: np.ndarray, 
                          object_type: str, color_image: np.ndarray = None) -> Dict:
    """
    創建訓練樣本
    
    Args:
        mask: 物件遮罩
        depth: 深度圖像
        object_type: 物件類型
        color_image: 彩色圖像（可選）
        
    Returns:
        訓練樣本字典
    """
    classifier = SmartObjectClassifier()
    features = classifier._extract_features(mask, depth, color_image)
    
    return {
        'features': features,
        'object_type': object_type,
        'timestamp': np.datetime64('now')
    }