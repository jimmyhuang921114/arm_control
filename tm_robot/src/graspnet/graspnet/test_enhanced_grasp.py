#!/usr/bin/env python3
"""
增強型細小物件抓取系統測試腳本
用於驗證各種抓取策略和物件分類的準確性
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from tm_robot_if.srv import CaptureImage
from cv_bridge import CvBridge
import time
import argparse
from pathlib import Path
import yaml
import matplotlib.pyplot as plt
from typing import List, Dict, Tuple
import json

class EnhancedGraspTester(Node):
    """增強抓取系統測試器"""
    
    def __init__(self):
        super().__init__('enhanced_grasp_tester')
        
        self.bridge = CvBridge()
        self.test_results = []
        
        # 創建服務客戶端
        self.enhanced_grab_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        
        # 等待服務可用
        self.get_logger().info("等待增強抓取服務...")
        self.enhanced_grab_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info("增強抓取測試器初始化完成")
    
    def test_with_synthetic_data(self, test_cases: List[Dict]):
        """使用合成數據進行測試"""
        self.get_logger().info(f"開始進行 {len(test_cases)} 個測試案例")
        
        for i, test_case in enumerate(test_cases):
            self.get_logger().info(f"執行測試案例 {i+1}/{len(test_cases)}: {test_case['name']}")
            
            # 生成測試數據
            mask, depth = self._generate_test_data(test_case)
            
            # 執行測試
            success, result = self._run_single_test(mask, depth, test_case)
            
            # 記錄結果
            test_result = {
                'test_id': i + 1,
                'name': test_case['name'],
                'object_type': test_case['object_type'],
                'success': success,
                'result': result,
                'timestamp': time.time()
            }
            
            self.test_results.append(test_result)
            
            # 短暫延遲
            time.sleep(0.5)
        
        # 生成測試報告
        self._generate_test_report()
    
    def test_with_real_images(self, image_dir: str):
        """使用真實圖像進行測試"""
        image_path = Path(image_dir)
        if not image_path.exists():
            self.get_logger().error(f"圖像目錄不存在: {image_dir}")
            return
        
        # 查找測試圖像
        color_images = list(image_path.glob("*_color.png")) + list(image_path.glob("*_color.jpg"))
        
        self.get_logger().info(f"找到 {len(color_images)} 個測試圖像")
        
        for color_img_path in color_images:
            # 尋找對應的深度圖像和遮罩
            base_name = color_img_path.stem.replace('_color', '')
            depth_img_path = color_img_path.parent / f"{base_name}_depth.png"
            mask_img_path = color_img_path.parent / f"{base_name}_mask.png"
            
            if not depth_img_path.exists() or not mask_img_path.exists():
                self.get_logger().warn(f"缺少深度或遮罩圖像: {base_name}")
                continue
            
            # 載入圖像
            try:
                mask = cv2.imread(str(mask_img_path), cv2.IMREAD_GRAYSCALE)
                depth = cv2.imread(str(depth_img_path), cv2.IMREAD_UNCHANGED)
                
                self.get_logger().info(f"測試圖像: {base_name}")
                
                # 執行測試
                success, result = self._run_single_test(mask, depth, {
                    'name': base_name,
                    'object_type': 'unknown'
                })
                
                # 記錄結果
                test_result = {
                    'test_id': len(self.test_results) + 1,
                    'name': base_name,
                    'image_path': str(color_img_path),
                    'success': success,
                    'result': result,
                    'timestamp': time.time()
                }
                
                self.test_results.append(test_result)
                
            except Exception as e:
                self.get_logger().error(f"處理圖像 {base_name} 時出錯: {e}")
        
        # 生成測試報告
        self._generate_test_report()
    
    def _generate_test_data(self, test_case: Dict) -> Tuple[np.ndarray, np.ndarray]:
        """生成合成測試數據"""
        width, height = 640, 480
        
        # 創建基礎深度圖像 (模擬桌面)
        depth = np.full((height, width), 500, dtype=np.uint16)  # 50cm深度
        
        # 創建遮罩
        mask = np.zeros((height, width), dtype=np.uint8)
        
        # 根據物件類型生成不同的形狀
        center_x, center_y = width // 2, height // 2
        
        if test_case['object_type'] == 'pill':
            # 圓形藥丸
            radius = np.random.randint(15, 25)
            cv2.circle(mask, (center_x, center_y), radius, 255, -1)
            # 添加高度變化
            cv2.circle(depth, (center_x, center_y), radius, 495, -1)
            
        elif test_case['object_type'] == 'capsule':
            # 橢圓形膠囊
            axes = (np.random.randint(30, 40), np.random.randint(15, 20))
            angle = np.random.randint(0, 180)
            cv2.ellipse(mask, (center_x, center_y), axes, angle, 0, 360, 255, -1)
            cv2.ellipse(depth, (center_x, center_y), axes, angle, 0, 360, 492, -1)
            
        elif test_case['object_type'] == 'tablet':
            # 矩形藥片
            width_obj = np.random.randint(25, 35)
            height_obj = np.random.randint(20, 30)
            x1, y1 = center_x - width_obj//2, center_y - height_obj//2
            x2, y2 = center_x + width_obj//2, center_y + height_obj//2
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            cv2.rectangle(depth, (x1, y1), (x2, y2), 497, -1)
            
        elif test_case['object_type'] == 'small_bottle':
            # 小瓶子 (矩形)
            width_obj = np.random.randint(40, 60)
            height_obj = np.random.randint(80, 120)
            x1, y1 = center_x - width_obj//2, center_y - height_obj//2
            x2, y2 = center_x + width_obj//2, center_y + height_obj//2
            cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
            cv2.rectangle(depth, (x1, y1), (x2, y2), 475, -1)  # 更高的物件
        
        # 添加噪聲
        noise = np.random.normal(0, 2, depth.shape).astype(np.int16)
        depth = np.clip(depth.astype(np.int16) + noise, 0, 65535).astype(np.uint16)
        
        return mask, depth
    
    def _run_single_test(self, mask: np.ndarray, depth: np.ndarray, 
                        test_case: Dict) -> Tuple[bool, Dict]:
        """執行單個測試"""
        try:
            # 準備服務請求
            req = CaptureImage.Request()
            req.mask = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            
            # 發送請求
            future = self.enhanced_grab_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                
                result = {
                    'service_success': response.success,
                    'test_case': test_case,
                    'mask_area': np.sum(mask > 0),
                    'depth_stats': {
                        'mean': float(np.mean(depth[mask > 0])) if np.any(mask > 0) else 0,
                        'std': float(np.std(depth[mask > 0])) if np.any(mask > 0) else 0,
                    }
                }
                
                return response.success, result
            else:
                self.get_logger().error("服務調用超時")
                return False, {'error': 'timeout'}
                
        except Exception as e:
            self.get_logger().error(f"測試執行失敗: {e}")
            return False, {'error': str(e)}
    
    def _generate_test_report(self):
        """生成測試報告"""
        if not self.test_results:
            self.get_logger().warn("沒有測試結果可報告")
            return
        
        # 計算統計信息
        total_tests = len(self.test_results)
        successful_tests = sum(1 for r in self.test_results if r['success'])
        success_rate = successful_tests / total_tests * 100
        
        # 按物件類型分組統計
        type_stats = {}
        for result in self.test_results:
            obj_type = result.get('object_type', 'unknown')
            if obj_type not in type_stats:
                type_stats[obj_type] = {'total': 0, 'success': 0}
            type_stats[obj_type]['total'] += 1
            if result['success']:
                type_stats[obj_type]['success'] += 1
        
        # 生成報告
        report = {
            'summary': {
                'total_tests': total_tests,
                'successful_tests': successful_tests,
                'success_rate': success_rate,
                'test_timestamp': time.time()
            },
            'type_statistics': {
                obj_type: {
                    'total': stats['total'],
                    'success': stats['success'],
                    'success_rate': stats['success'] / stats['total'] * 100
                }
                for obj_type, stats in type_stats.items()
            },
            'detailed_results': self.test_results
        }
        
        # 保存報告
        report_path = Path(f"enhanced_grasp_test_report_{int(time.time())}.json")
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        # 打印摘要
        self.get_logger().info("=" * 60)
        self.get_logger().info("增強抓取系統測試報告")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"總測試數: {total_tests}")
        self.get_logger().info(f"成功測試數: {successful_tests}")
        self.get_logger().info(f"成功率: {success_rate:.1f}%")
        self.get_logger().info("")
        
        self.get_logger().info("按物件類型統計:")
        for obj_type, stats in report['type_statistics'].items():
            self.get_logger().info(
                f"  {obj_type}: {stats['success']}/{stats['total']} "
                f"({stats['success_rate']:.1f}%)")
        
        self.get_logger().info(f"\n詳細報告已保存至: {report_path}")
        
        # 生成可視化圖表
        self._generate_visualization(report)
    
    def _generate_visualization(self, report: Dict):
        """生成測試結果可視化"""
        try:
            import matplotlib.pyplot as plt
            
            # 創建圖表
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
            
            # 成功率餅圖
            success_data = [
                report['summary']['successful_tests'],
                report['summary']['total_tests'] - report['summary']['successful_tests']
            ]
            ax1.pie(success_data, labels=['成功', '失敗'], autopct='%1.1f%%', 
                   colors=['#2ecc71', '#e74c3c'])
            ax1.set_title('總體成功率')
            
            # 按類型成功率柱狀圖
            types = list(report['type_statistics'].keys())
            success_rates = [report['type_statistics'][t]['success_rate'] for t in types]
            
            ax2.bar(types, success_rates, color='#3498db')
            ax2.set_ylabel('成功率 (%)')
            ax2.set_title('按物件類型成功率')
            ax2.set_ylim(0, 100)
            
            # 旋轉標籤以便閱讀
            plt.setp(ax2.get_xticklabels(), rotation=45, ha='right')
            
            plt.tight_layout()
            
            # 保存圖表
            chart_path = f"enhanced_grasp_test_chart_{int(time.time())}.png"
            plt.savefig(chart_path, dpi=300, bbox_inches='tight')
            
            self.get_logger().info(f"測試圖表已保存至: {chart_path}")
            
        except ImportError:
            self.get_logger().warn("matplotlib不可用，跳過圖表生成")
        except Exception as e:
            self.get_logger().error(f"生成圖表失敗: {e}")


def create_default_test_cases() -> List[Dict]:
    """創建默認測試案例"""
    return [
        {
            'name': '小圓形藥丸測試',
            'object_type': 'pill',
            'description': '測試圓形藥丸的抓取檢測'
        },
        {
            'name': '橢圓形膠囊測試',
            'object_type': 'capsule',
            'description': '測試長條形膠囊的抓取檢測'
        },
        {
            'name': '方形藥片測試',
            'object_type': 'tablet',
            'description': '測試扁平藥片的抓取檢測'
        },
        {
            'name': '小瓶子測試',
            'object_type': 'small_bottle',
            'description': '測試小藥瓶的抓取檢測'
        },
        # 邊界案例
        {
            'name': '極小物件測試',
            'object_type': 'pill',
            'description': '測試極小物件的檢測限制'
        },
        {
            'name': '細長物件測試',
            'object_type': 'capsule',
            'description': '測試細長物件的抓取策略'
        }
    ]


def main():
    parser = argparse.ArgumentParser(description='增強抓取系統測試工具')
    parser.add_argument('--mode', choices=['synthetic', 'real'], default='synthetic',
                       help='測試模式：synthetic (合成數據) 或 real (真實圖像)')
    parser.add_argument('--image-dir', type=str,
                       help='真實圖像測試的圖像目錄路徑')
    parser.add_argument('--num-tests', type=int, default=10,
                       help='合成數據測試的案例數量')
    
    args = parser.parse_args()
    
    rclpy.init()
    tester = EnhancedGraspTester()
    
    try:
        if args.mode == 'synthetic':
            # 合成數據測試
            test_cases = create_default_test_cases()
            
            # 重複測試案例以達到指定數量
            if args.num_tests > len(test_cases):
                multiplier = args.num_tests // len(test_cases) + 1
                test_cases = (test_cases * multiplier)[:args.num_tests]
            else:
                test_cases = test_cases[:args.num_tests]
            
            tester.test_with_synthetic_data(test_cases)
            
        elif args.mode == 'real':
            # 真實圖像測試
            if not args.image_dir:
                print("錯誤：真實圖像測試需要指定 --image-dir 參數")
                return
            
            tester.test_with_real_images(args.image_dir)
        
        tester.get_logger().info("測試完成！")
        
    except KeyboardInterrupt:
        tester.get_logger().info("測試被用戶中斷")
    except Exception as e:
        tester.get_logger().error(f"測試執行失敗: {e}")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()