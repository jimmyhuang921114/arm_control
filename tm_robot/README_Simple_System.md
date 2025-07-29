# 簡化版藥局系統

## 系統概述

這是一個簡化版的藥局自動化系統，專注於核心功能：

### 🎯 核心工作流程

```
訂單接收 → GroundedSAM2物品挑選 → 抓取移動 → 特定位置LLM確認 → 完成分發
```

### 🔧 主要組件

1. **GroundedSAM2**: 用於物品挑選和識別
2. **增強抓取系統**: 針對細小物件優化的抓取
3. **LLM確認系統**: 在特定位置進行二次確認
4. **網頁界面**: 現有的醫院系統界面

## 使用方法

### 啟動系統

```bash
# 基本啟動
ros2 launch tm_robot simple_pharmacy_system.launch.py

# 模擬環境
ros2 launch tm_robot simple_pharmacy_system.launch.py use_simulation:=true

# 不啟用網頁界面
ros2 launch tm_robot simple_pharmacy_system.launch.py enable_gui:=false
```

### 發送訂單

```bash
ros2 topic pub /pharmacy/new_order std_msgs/String '{
  "data": "{
    \"order_id\": \"ORD001\",
    \"medicines\": [
      {\"name\": \"paracetamol\", \"quantity\": 1}
    ]
  }"
}'
```

### 監控狀態

```bash
# 查看系統狀態
ros2 topic echo /pharmacy/status

# 查看所有節點
ros2 node list

# 查看所有服務
ros2 service list
```

## 系統特點

### ✅ 保留的核心功能
- GroundedSAM2物品識別
- 增強抓取檢測
- 現有網頁界面
- LLM確認（僅在特定位置）

### ❌ 移除的複雜功能
- 複雜的安全認證系統
- 詳細的審計日誌
- 性能監控系統
- 資料庫備份機制
- 複雜的錯誤處理

## 網頁界面

系統會自動啟動現有的網頁界面：
- **URL**: http://localhost:8000
- **功能**: 藥物管理、處方管理、醫師介面

現有頁面：
- `Medicine.html` - 藥物管理
- `Prescription.html` - 處方管理  
- `doctor.html` - 醫師介面
- `background.html` - 背景頁面

## 技術要求

### 最低系統要求
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- 8GB RAM
- NVIDIA GPU（用於GroundedSAM2）

### 依賴安裝
```bash
pip install opencv-python numpy scipy scikit-learn
pip install torch torchvision torchaudio
pip install fastapi uvicorn
```

## 系統架構

```
┌─────────────────────────────────────────┐
│           簡化藥局系統                    │
├─────────────────────────────────────────┤
│  ┌───────────┐    ┌───────────┐         │
│  │ 網頁界面   │    │ 訂單處理  │         │
│  └───────────┘    └───────────┘         │
│           │              │              │
│  ┌─────────────────────────────────────┐  │
│  │      簡化藥局系統控制器              │  │
│  └─────────────────────────────────────┘  │
│           │              │              │
│  ┌───────────────┐  ┌─────────────────┐  │
│  │ GroundedSAM2  │  │  增強抓取檢測    │  │
│  │  物品挑選     │  │      系統       │  │
│  └───────────────┘  └─────────────────┘  │
│                                        │
│  ┌─────────────────────────────────────┐  │
│  │      LLM確認系統（特定位置）         │  │
│  └─────────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

## 故障排除

### 常見問題

1. **GroundedSAM2服務無法啟動**
   ```bash
   # 檢查模型文件
   ls /workspace/Grounded-SAM-2/checkpoints/
   
   # 檢查GPU
   nvidia-smi
   ```

2. **網頁界面無法訪問**
   ```bash
   # 檢查端口
   netstat -tulpn | grep 8000
   
   # 手動啟動
   cd /workspace/tm_robot/src/user_interface
   python3 -m uvicorn main:app --host 0.0.0.0 --port 8000
   ```

3. **相機連接問題**
   ```bash
   # 檢查相機設備
   lsusb | grep Intel
   
   # 檢查RealSense
   rs-enumerate-devices
   ```

### 日誌查看

```bash
# ROS節點日誌
ros2 log info

# 系統日誌
journalctl -f

# 網頁界面日誌
# 查看終端輸出
```

## 開發說明

### 檔案結構
```
tm_robot/
├── src/tm_robot_main/tm_robot_main/
│   ├── simple_pharmacy_system.py      # 主控制系統
│   └── main_control.py                # 機械手臂控制
├── src/user_interface/                # 網頁界面
│   ├── main.py                        # FastAPI應用
│   ├── static/html/                   # HTML模板
│   └── route/                         # API路由
├── src/graspnet/graspnet/             # 抓取檢測
└── launch/
    └── simple_pharmacy_system.launch.py  # 啟動腳本
```

### 自定義修改

1. **修改藥物識別參數**
   ```python
   # 在 simple_pharmacy_system.py 中
   req.confidence_threshold = 0.3  # 調整置信度
   req.prompt = f"{medicine_name} medicine pill tablet"  # 修改提示詞
   ```

2. **調整抓取參數**
   ```yaml
   # 在 small_object_config.yaml 中
   detection:
     min_area: 100
     max_area: 5000
   ```

3. **修改網頁界面**
   ```python
   # 在 user_interface/main.py 中添加新路由
   @app.get("/new_page.html")
   async def new_page(request: Request):
       return templates.TemplateResponse("new_page.html", {"request": request})
   ```

---

**版本**: 簡化版 1.0  
**更新**: 2024年12月  
**兼容**: ROS 2 Humble, Ubuntu 22.04