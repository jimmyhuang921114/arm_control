# 增強型細小物件抓取系統

## 概述

這是一個專門針對藥物等細小物件設計的智能抓取系統，具備以下特色：

- **智能物件分類**：自動識別藥丸、膠囊、藥片、小瓶等不同類型
- **多策略抓取**：根據物件特性選擇最適合的抓取方法
- **精確吸取點檢測**：專為細小物件優化的吸取點和姿態計算
- **配置化參數**：支援不同物件類型的參數調整
- **完整測試框架**：提供自動化測試和性能評估

## 系統架構

```
視覺輸入 → 物件分類 → 特徵分析 → 策略選擇 → 抓取點生成 → 姿態計算 → 執行控制
    ↓           ↓          ↓          ↓           ↓           ↓          ↓
  RGB-D      智能分類    幾何特徵    抓取策略    候選點評估   姿態優化    機械手臂
```

## 主要組件

### 1. 增強抓取檢測節點 (`EnhancedSmallObjectGraspNode`)

主要功能：
- 接收視覺輸入（彩色圖像、深度圖像、物件遮罩）
- 執行智能物件分類
- 生成抓取候選點
- 計算最佳抓取姿態
- 發布抓取指令

### 2. 智能物件分類器 (`SmartObjectClassifier`)

特點：
- 基於幾何、深度、形狀、紋理特徵的多維分析
- 支援機器學習和基於規則的分類方法
- 自動選擇最適合的抓取策略
- 可訓練和更新分類模型

### 3. 配置系統

- **`small_object_config.yaml`**：完整的系統配置文件
- 支援不同物件類型的個別參數設置
- 可調整的抓取策略和安全參數

## 安裝和設置

### 1. 依賴項安裝

```bash
# ROS 2 依賴
sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros

# Python 依賴
pip install opencv-python numpy scipy scikit-learn joblib pyyaml
```

### 2. 編譯

```bash
cd /workspace/tm_robot
colcon build --packages-select graspnet
source install/setup.bash
```

### 3. 配置

編輯配置文件 `src/graspnet/graspnet/small_object_config.yaml`：

```yaml
# 相機參數 - 根據實際相機校正結果調整
camera:
  intrinsic:
    fx: 904.8729050868374
    fy: 903.3201754368574
    cx: 634.3937317400505
    cy: 369.0644726085734

# 物件檢測參數 - 根據實際物件大小調整
detection:
  object_types:
    pills:
      min_area: 80      # 最小像素面積
      max_area: 2000    # 最大像素面積
      min_depth_points: 15
```

## 使用方法

### 1. 啟動系統

```bash
# 啟動完整系統
ros2 launch graspnet enhanced_grasp_system.launch.py

# 啟動調試模式
ros2 launch graspnet enhanced_grasp_system.launch.py debug_mode:=true

# 使用自定義配置
ros2 launch graspnet enhanced_grasp_system.launch.py config_file:=/path/to/config.yaml
```

### 2. 服務調用

```bash
# 測試增強抓取服務
ros2 service call /enhanced_grab_detect tm_robot_if/srv/CaptureImage "{mask: ...}"
```

### 3. 整合到主控制系統

在主控制程序中調用：

```python
# 調用增強抓取檢測
self.call_enhanced_grab()
```

## 抓取策略

### 1. 基於質心的抓取
- **適用**：規則形狀的物件（藥丸、藥片）
- **特點**：穩定可靠，適合大多數情況
- **參數**：`confidence_base: 0.7`

### 2. 基於最高點的抓取
- **適用**：有明顯高點的物件（膠囊、小瓶）
- **特點**：抓取成功率高，適合細小物件
- **參數**：`confidence_base: 0.8`

### 3. 基於穩定區域的抓取
- **適用**：不規則形狀的物件
- **特點**：多候選點，提高成功率
- **參數**：`clustering_eps: 10`

### 4. 基於橢圓軸的抓取
- **適用**：長條形物件（膠囊）
- **特點**：考慮物件方向性
- **參數**：`axis_offset_ratio: 0.3`

## 物件分類

### 支援的物件類型

1. **藥丸 (pill)**
   - 圓形或近圓形
   - 相對較小
   - 推薦策略：質心、最高點

2. **膠囊 (capsule)**
   - 橢圓形或長條形
   - 長寬比 > 1.8
   - 推薦策略：穩定區域、最高點

3. **藥片 (tablet)**
   - 方形或矩形
   - 相對較平
   - 推薦策略：質心、橢圓軸

4. **小瓶 (small_bottle)**
   - 大面積且較高
   - 推薦策略：最高點、穩定區域

### 分類特徵

- **幾何特徵**：面積、周長、長寬比、密實度
- **深度特徵**：高度、表面粗糙度、深度變化
- **形狀特徵**：圓度、凸性、Hu矩
- **紋理特徵**：亮度統計、局部變化

## 測試和驗證

### 1. 自動化測試

```bash
# 合成數據測試
ros2 run graspnet test_enhanced_grasp --mode synthetic --num-tests 20

# 真實圖像測試
ros2 run graspnet test_enhanced_grasp --mode real --image-dir /path/to/test/images
```

### 2. 測試數據格式

真實圖像測試需要以下文件：
```
test_images/
├── sample1_color.png    # 彩色圖像
├── sample1_depth.png    # 深度圖像
├── sample1_mask.png     # 物件遮罩
├── sample2_color.png
├── sample2_depth.png
└── sample2_mask.png
```

### 3. 性能評估

測試結果包括：
- 總體成功率
- 按物件類型分類成功率
- 詳細的測試報告和可視化圖表

## 參數調優

### 1. 檢測參數調整

根據實際應用場景調整物件檢測閾值：

```yaml
detection:
  general:
    min_object_area: 80      # 調整最小檢測面積
    max_object_area: 8000    # 調整最大檢測面積
    min_depth_points: 15     # 調整最少深度點數
```

### 2. 抓取策略調整

調整不同策略的權重和參數：

```yaml
candidate_generation:
  highest_point:
    confidence_base: 0.8     # 調整基礎置信度
    weight: 1.2              # 調整策略權重
```

### 3. 安全參數設置

設置工作空間限制和安全參數：

```yaml
safety:
  workspace_limits:
    x_min: -0.15             # 工作空間X軸最小值
    x_max: 0.15              # 工作空間X軸最大值
    z_min: 0.02              # 工作空間Z軸最小值
    z_max: 0.50              # 工作空間Z軸最大值
```

## 故障排除

### 常見問題

1. **服務無法連接**
   - 檢查節點是否正常啟動
   - 確認服務名稱正確
   - 檢查網絡連接

2. **物件分類錯誤**
   - 檢查配置文件參數
   - 重新訓練分類模型
   - 調整分類規則

3. **抓取失敗**
   - 檢查工作空間限制
   - 調整吸取偏移量
   - 驗證相機校正參數

4. **性能問題**
   - 啟用性能監控
   - 檢查系統資源使用
   - 調整處理頻率

### 調試技巧

1. **啟用調試模式**
   ```bash
   ros2 launch graspnet enhanced_grasp_system.launch.py debug_mode:=true
   ```

2. **查看詳細日誌**
   ```bash
   ros2 run graspnet enhanced_small_object_grasp --ros-args --log-level DEBUG
   ```

3. **可視化結果**
   - 使用 RViz 查看抓取姿態
   - 保存調試圖像
   - 分析測試報告

## 進階功能

### 1. 自定義物件類型

添加新的物件類型：

1. 在配置文件中添加新類型定義
2. 更新分類器的物件類型字典
3. 實現對應的抓取策略
4. 準備訓練數據

### 2. 機器學習模型訓練

```python
from smart_object_classifier import SmartObjectClassifier, create_training_sample

# 創建訓練樣本
training_data = []
for mask, depth, object_type in training_set:
    sample = create_training_sample(mask, depth, object_type)
    training_data.append(sample)

# 訓練分類器
classifier = SmartObjectClassifier()
classifier.train_classifier(training_data)
```

### 3. 實時性能監控

系統提供完整的性能監控功能：
- 處理時間統計
- 成功率追蹤
- 資源使用監控
- 自動異常檢測

## 技術支援

如有問題或需要技術支援，請：

1. 查看系統日誌了解詳細錯誤信息
2. 運行測試腳本驗證系統功能
3. 檢查配置文件參數設置
4. 參考故障排除指南

---

**版本**: 1.0  
**更新日期**: 2024年12月  
**兼容性**: ROS 2 Humble, Ubuntu 22.04