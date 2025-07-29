# 完整藥局自動化系統

## 系統概述

這是一個完整的藥局自動化分揀系統，從訂單接收、藥物識別、精確抓取到最終分發的端到端解決方案。系統採用先進的視覺識別技術和智能抓取算法，專門針對細小藥物進行優化，並具備完整的安全性和保密性保障。

### 🎯 核心特色

- **智能藥物識別**: 以GroundedSAM2為主，LLM作為二次確認
- **精確細小物件抓取**: 專門針對藥物等細小物件優化的抓取算法
- **端到端工作流程**: 從訂單到分發的完整自動化流程
- **高度安全保密**: 完整的認證、加密和審計機制
- **實時監控**: 全面的系統狀態監控和性能分析
- **容錯設計**: 完善的錯誤處理和自動恢復機制

## 系統架構

```
┌─────────────────────────────────────────────────────────────┐
│                    藥局自動化系統架構                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  訂單接收   │  │  用戶界面   │  │  系統監控   │         │
│  │   模組      │  │    模組     │  │    模組     │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│           │               │               │                │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │              系統總協調器                                │ │
│  │        (PharmacySystemOrchestrator)                   │ │
│  └─────────────────────────────────────────────────────────┘ │
│           │               │               │                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │  視覺識別   │  │  增強抓取   │  │  機械手臂   │         │
│  │   系統      │  │   檢測      │  │   控制      │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│           │               │               │                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐         │
│  │ GroundedSAM2│  │ 智能分類器  │  │  資料庫     │         │
│  └─────────────┘  └─────────────┘  └─────────────┘         │
│                                                           │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │                LLM二次確認系統                           │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 主要組件

### 1. 系統總協調器 (`PharmacySystemOrchestrator`)
- **功能**: 統一管理和協調所有子系統
- **特點**: 
  - 分階段任務處理流程
  - 優先級管理和任務調度
  - 完整的安全認證和審計
  - 實時性能監控和告警

### 2. 整合視覺識別系統 (`IntegratedVisionSystem`)
- **主要技術**: GroundedSAM2
- **二次確認**: LLM藥物識別系統
- **功能**: 
  - 自動藥物檢測和分割
  - 低置信度情況下的LLM確認
  - 多類型藥物支援

### 3. 增強抓取檢測系統 (`EnhancedSmallObjectGraspNode`)
- **專長**: 細小物件精確抓取
- **技術**: 
  - 智能物件分類器
  - 多策略抓取候選點生成
  - 基於物件特徵的最佳抓取點選擇

### 4. 智能物件分類器 (`SmartObjectClassifier`)
- **功能**: 自動識別藥物類型
- **支援類型**: 藥丸、膠囊、藥片、小瓶
- **特徵**: 幾何、深度、形狀、紋理多維分析

## 工作流程

### 完整處理流程

```
訂單接收 → 數據驗證 → 任務創建 → 視覺識別 → 抓取規劃 → 
執行抓取 → 二次確認 → 包裝分發 → 完成記錄
```

### 詳細階段說明

1. **訂單處理階段**
   - 接收和驗證訂單數據
   - 創建對應的藥物任務
   - 優先級分配和調度

2. **視覺識別階段**
   - GroundedSAM2主要檢測
   - 藥物定位和分割
   - 置信度評估

3. **確認階段**（條件觸發）
   - 低置信度情況下啟動
   - LLM二次確認
   - 人工干預選項

4. **抓取階段**
   - 智能物件分類
   - 多策略抓取點生成
   - 最佳抓取方案選擇
   - 精確執行抓取

5. **完成階段**
   - 包裝和標籤
   - 品質檢查
   - 分發準備

## 安裝部署

### 1. 系統要求

```bash
# 作業系統
Ubuntu 22.04 LTS

# ROS 2
ROS 2 Humble

# Python 依賴
pip install opencv-python numpy scipy scikit-learn joblib pyyaml
pip install torch torchvision torchaudio
pip install fastapi uvicorn

# 硬體要求
- CPU: 8核心以上
- RAM: 16GB以上  
- GPU: NVIDIA RTX 3060以上（用於GroundedSAM2）
- 儲存: 100GB以上可用空間
```

### 2. 編譯系統

```bash
# 進入工作空間
cd /workspace

# 編譯所有包
colcon build --packages-select tm_robot_main tm_robot_if graspnet groundedsam2 dis_service

# 載入環境
source install/setup.bash
```

### 3. 模型下載

```bash
# GroundedSAM2模型
cd /workspace/Grounded-SAM-2
wget https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt -P checkpoints/

# GroundingDINO模型  
wget https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha2/groundingdino_swint_ogc.pth -P gdino_checkpoints/
```

## 使用方法

### 1. 啟動完整系統

```bash
# 標準啟動
ros2 launch tm_robot complete_pharmacy_system.launch.py

# 調試模式啟動
ros2 launch tm_robot complete_pharmacy_system.launch.py debug_mode:=true

# 模擬環境啟動
ros2 launch tm_robot complete_pharmacy_system.launch.py use_simulation:=true

# 自定義配置啟動
ros2 launch tm_robot complete_pharmacy_system.launch.py config_file:=/path/to/config.yaml
```

### 2. 系統監控

```bash
# 查看系統狀態
ros2 topic echo /pharmacy/system_status

# 查看性能指標
ros2 topic echo /pharmacy/metrics

# 檢查各組件狀態
ros2 node list
ros2 service list
```

### 3. 發送測試訂單

```bash
# 發送測試訂單
ros2 topic pub /pharmacy/new_order std_msgs/String '{
  "data": "{
    \"order_id\": \"ORD001\",
    \"patient_id\": \"P001\",
    \"prescription_id\": \"RX001\",
    \"priority\": 2,
    \"medicines\": [
      {\"name\": \"paracetamol\", \"quantity\": 2},
      {\"name\": \"aspirin\", \"quantity\": 1}
    ]
  }"
}'
```

### 4. 系統控制

```bash
# 暫停系統
ros2 service call /pharmacy/system_control std_msgs/srv/String '{
  "data": "{\"command\": \"pause\"}"
}'

# 恢復系統
ros2 service call /pharmacy/system_control std_msgs/srv/String '{
  "data": "{\"command\": \"resume\"}"
}'

# 緊急停止
ros2 topic pub /emergency_stop std_msgs/Bool '{data: true}'
```

## 配置說明

### 1. 主要配置文件

- `system_config.yaml`: 系統總配置
- `vision_config.yaml`: 視覺識別配置  
- `small_object_config.yaml`: 細小物件抓取配置

### 2. 關鍵參數調整

```yaml
# 視覺識別閾值
detection:
  confidence_threshold: 0.3    # 主檢測閾值
  confirmation_threshold: 0.7  # LLM確認觸發閾值

# 抓取參數
grasp_strategies:
  suction:
    offset_distance: 0.005     # 吸嘴偏移距離
    
# 安全設置
security:
  enable_authentication: true  # 啟用認證
  audit_logging: true         # 審計日誌
```

## 安全與保密

### 1. 資料安全

- **加密存儲**: 所有敏感數據採用AES-256加密
- **匿名化**: 患者資料自動匿名化處理
- **訪問控制**: 基於角色的權限管理
- **審計日誌**: 完整的操作記錄和追蹤

### 2. 系統安全

- **認證機制**: 多層次用戶認證
- **網路安全**: 安全通信協議
- **入侵檢測**: 異常行為監控
- **備份恢復**: 自動資料備份和恢復

### 3. 合規要求

- **HIPAA合規**: 醫療資料隱私保護
- **FDA規範**: 醫療設備安全標準
- **國際標準**: ISO 27001資訊安全

## 監控與維護

### 1. 實時監控

- **系統狀態**: 即時組件健康監控
- **性能指標**: 處理時間、成功率、錯誤率
- **資源使用**: CPU、記憶體、磁盤使用率
- **告警系統**: 異常自動通知

### 2. 日誌管理

```bash
# 系統日誌位置
/tmp/pharmacy_system.log      # 主系統日誌
/tmp/pharmacy_errors.log      # 錯誤日誌  
/tmp/pharmacy_audit.log       # 審計日誌
/tmp/pharmacy_performance.log # 性能日誌
```

### 3. 維護程序

- **定期備份**: 每日資料庫備份
- **性能分析**: 週期性性能評估
- **軟體更新**: 安全更新和功能升級
- **硬體檢查**: 定期設備健康檢查

## 故障排除

### 1. 常見問題

| 問題 | 可能原因 | 解決方案 |
|------|----------|----------|
| 視覺識別失敗 | 光照不足、相機故障 | 檢查照明和相機連接 |
| 抓取失敗 | 位置偏差、吸力不足 | 重新校正和調整參數 |
| 系統響應慢 | 資源不足、網路延遲 | 檢查系統資源和網路 |
| 認證失敗 | 憑證過期、權限不足 | 更新憑證和檢查權限 |

### 2. 診斷工具

```bash
# 系統診斷
ros2 run diagnostic_aggregator aggregator_node

# 性能分析
ros2 topic hz /pharmacy/system_status

# 錯誤檢查  
journalctl -u pharmacy-system -f
```

## API文檔

### 1. ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pharmacy/new_order` | String | 新訂單接收 |
| `/pharmacy/system_status` | String | 系統狀態廣播 |
| `/pharmacy/metrics` | String | 性能指標 |
| `/emergency_stop` | Bool | 緊急停止 |

### 2. ROS 2 Services

| Service | Type | Description |
|---------|------|-------------|
| `/pharmacy/system_control` | String | 系統控制 |
| `/vision/detect_medicine` | GroundedSAM2Interface | 藥物檢測 |
| `/enhanced_grab_detect` | CaptureImage | 增強抓取 |

### 3. Web API

```bash
# 系統狀態
GET http://localhost:8000/api/system/status

# 訂單管理
POST http://localhost:8000/api/orders
GET http://localhost:8000/api/orders/{order_id}

# 藥物管理
GET http://localhost:8000/api/medicines
POST http://localhost:8000/api/medicines
```

## 性能指標

### 1. 目標性能

- **處理時間**: 單個藥物 < 60秒
- **準確率**: 藥物識別 > 95%
- **成功率**: 抓取成功 > 90%
- **可用性**: 系統正常運行 > 99%

### 2. 監控指標

- **吞吐量**: 每小時處理訂單數
- **錯誤率**: 各階段失敗比例
- **響應時間**: 系統響應延遲
- **資源使用**: 系統資源消耗

## 開發與擴展

### 1. 添加新藥物類型

```python
# 在智能分類器中添加新類型
self.object_types = {
    'pill': 0,
    'capsule': 1, 
    'tablet': 2,
    'small_bottle': 3,
    'new_medicine_type': 4  # 新增類型
}
```

### 2. 自定義抓取策略

```python
# 實現新的抓取方法
def _generate_custom_grasp(self, object_info):
    # 自定義抓取邏輯
    pass
```

### 3. 集成外部系統

```yaml
# 配置外部API
integration:
  external_systems:
    hospital_info_system:
      enable: true
      endpoint: "https://his.hospital.com/api"
```

## 支援與聯繫

### 技術支援

- **文檔**: 參考各組件的詳細README
- **日誌**: 查看系統日誌了解詳細錯誤
- **監控**: 使用內建監控工具診斷問題

### 系統更新

定期檢查更新：
- GroundedSAM2模型更新
- 安全補丁
- 功能改進

---

**版本**: 1.0  
**最後更新**: 2024年12月  
**兼容性**: ROS 2 Humble, Ubuntu 22.04  
**許可證**: 專有軟體，保密項目