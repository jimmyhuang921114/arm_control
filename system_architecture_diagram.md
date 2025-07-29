# 藥局自動化系統 - 架構圖與流程圖

## 🏗️ 整體系統架構圖

```mermaid
graph TB
    subgraph "前端層 Frontend Layer"
        WEB[藥局訂單管理網頁<br/>orders.html]
        UI[病人搜尋界面<br/>藥物選擇界面<br/>訂單追蹤界面]
    end

    subgraph "API層 API Layer"
        API[FastAPI 服務器<br/>main.py]
        ORDER_API[訂單API<br/>routes_orders.py]
        PATIENT_API[病人API<br/>routes_medicine.py]
        MED_API[藥物API<br/>routes_prescription.py]
    end

    subgraph "資料庫層 Database Layer"
        DB[(SQLite 資料庫)]
        PATIENT_TBL[Patient 病人表]
        MEDICINE_TBL[Medicine 藥物表]
        ORDER_TBL[Order 訂單表]
        ORDER_ITEM_TBL[OrderItem 訂單項目表]
        JSON_CONFIG_TBL[MedicineJsonConfig<br/>JSON配置表]
        LOG_TBL[ProcessingLog 處理日誌表]
    end

    subgraph "機器人控制層 Robot Control Layer"
        INTEGRATED_ROBOT[整合機器人系統<br/>integrated_robot_system.py]
        ORDER_MONITOR[訂單監控線程]
        ORDER_PROCESSOR[訂單處理線程]
    end

    subgraph "視覺處理層 Vision Processing Layer"
        OCR_SERVICE[OCR文字識別服務]
        GROUNDED_SAM2[GroundedSAM2服務<br/>物品識別與分割]
        LLM_SERVICE[LLM確認服務<br/>藥物二次確認]
    end

    subgraph "抓取控制層 Grasp Control Layer"
        ENHANCED_GRASP[增強抓取服務<br/>enhanced_small_object_grasp.py]
        SMART_CLASSIFIER[智能物件分類器<br/>smart_object_classifier.py]
        GRASP_CONFIG[抓取配置<br/>small_object_config.yaml]
    end

    subgraph "硬體層 Hardware Layer"
        TM_ROBOT[TM Robot 機械手臂]
        CAMERA[RGB-D 相機]
        SUCTION[吸盤系統]
    end

    subgraph "ROS2通信層 ROS2 Communication"
        ROS2_BRIDGE[ROS2 Bridge]
        TOPICS[ROS2 Topics]
        SERVICES[ROS2 Services]
    end

    %% 連接關係
    WEB --> API
    UI --> API
    API --> ORDER_API
    API --> PATIENT_API
    API --> MED_API
    
    ORDER_API --> DB
    PATIENT_API --> DB
    MED_API --> DB
    
    DB --> PATIENT_TBL
    DB --> MEDICINE_TBL
    DB --> ORDER_TBL
    DB --> ORDER_ITEM_TBL
    DB --> JSON_CONFIG_TBL
    DB --> LOG_TBL
    
    ORDER_API -.->|HTTP請求| INTEGRATED_ROBOT
    INTEGRATED_ROBOT --> ORDER_MONITOR
    INTEGRATED_ROBOT --> ORDER_PROCESSOR
    
    ORDER_PROCESSOR --> OCR_SERVICE
    ORDER_PROCESSOR --> GROUNDED_SAM2
    ORDER_PROCESSOR --> LLM_SERVICE
    ORDER_PROCESSOR --> ENHANCED_GRASP
    
    ENHANCED_GRASP --> SMART_CLASSIFIER
    ENHANCED_GRASP --> GRASP_CONFIG
    
    INTEGRATED_ROBOT --> ROS2_BRIDGE
    OCR_SERVICE --> ROS2_BRIDGE
    GROUNDED_SAM2 --> ROS2_BRIDGE
    LLM_SERVICE --> ROS2_BRIDGE
    ENHANCED_GRASP --> ROS2_BRIDGE
    
    ROS2_BRIDGE --> TOPICS
    ROS2_BRIDGE --> SERVICES
    
    TOPICS --> TM_ROBOT
    SERVICES --> TM_ROBOT
    TOPICS --> CAMERA
    SERVICES --> SUCTION
```

## 📋 完整業務流程圖

```mermaid
flowchart TD
    START([系統啟動]) --> INIT[初始化資料庫<br/>載入JSON配置]
    
    subgraph "網頁端操作 Web Operations"
        SEARCH_PATIENT[搜尋病人<br/>輸入姓名/病歷號]
        SELECT_PATIENT[選擇病人]
        ADD_MEDICINE[添加藥物<br/>選擇藥物種類和數量]
        CREATE_ORDER[創建訂單]
    end
    
    subgraph "資料庫操作 Database Operations"
        SAVE_ORDER[儲存訂單到資料庫]
        LINK_JSON[關聯藥物JSON配置]
        SET_PENDING[設置訂單狀態為pending]
    end
    
    subgraph "機器人系統 Robot System"
        FETCH_ORDER[機器人獲取待處理訂單]
        EXTRACT_JSON[提取藥物JSON配置]
        START_PROCESS[開始處理訂單]
    end
    
    subgraph "OCR階段 OCR Stage"
        CAPTURE_OCR[拍攝圖像進行OCR]
        OCR_ANALYSIS[OCR文字識別分析]
        OCR_RESULT[獲得OCR結果<br/>輔助後續識別]
    end
    
    subgraph "GroundedSAM2階段 GroundedSAM2 Stage"
        CAPTURE_VISION[拍攝RGB-D圖像]
        GROUNDED_DETECT[GroundedSAM2物品檢測]
        GENERATE_MASK[生成物品遮罩]
        VALIDATE_DETECTION{檢測成功?}
    end
    
    subgraph "抓取階段 Grasp Stage"
        ANALYZE_OBJECT[分析物件特性]
        PLAN_GRASP[規劃抓取策略]
        EXECUTE_GRASP[執行抓取動作]
        MOVE_TO_CONFIRM[移動到確認位置]
        GRASP_SUCCESS{抓取成功?}
    end
    
    subgraph "LLM確認階段 LLM Confirmation Stage"
        CAPTURE_CONFIRM[在確認位置拍攝]
        LLM_VERIFY[LLM藥物識別確認]
        COMBINE_RESULTS[結合OCR和視覺結果]
        LLM_SUCCESS{確認成功?}
    end
    
    subgraph "完成階段 Completion Stage"
        DISPENSE[完成藥物分發]
        UPDATE_STATUS[更新訂單狀態為completed]
        LOG_PROCESS[記錄處理日誌]
        NOTIFY_COMPLETE[通知前端訂單完成]
    end
    
    INIT --> SEARCH_PATIENT
    SEARCH_PATIENT --> SELECT_PATIENT
    SELECT_PATIENT --> ADD_MEDICINE
    ADD_MEDICINE --> CREATE_ORDER
    CREATE_ORDER --> SAVE_ORDER
    SAVE_ORDER --> LINK_JSON
    LINK_JSON --> SET_PENDING
    SET_PENDING --> FETCH_ORDER
    
    FETCH_ORDER --> EXTRACT_JSON
    EXTRACT_JSON --> START_PROCESS
    START_PROCESS --> CAPTURE_OCR
    
    CAPTURE_OCR --> OCR_ANALYSIS
    OCR_ANALYSIS --> OCR_RESULT
    OCR_RESULT --> CAPTURE_VISION
    
    CAPTURE_VISION --> GROUNDED_DETECT
    GROUNDED_DETECT --> GENERATE_MASK
    GENERATE_MASK --> VALIDATE_DETECTION
    
    VALIDATE_DETECTION -->|成功| ANALYZE_OBJECT
    VALIDATE_DETECTION -->|失敗| RETRY_DETECT[重試檢測]
    RETRY_DETECT --> CAPTURE_VISION
    
    ANALYZE_OBJECT --> PLAN_GRASP
    PLAN_GRASP --> EXECUTE_GRASP
    EXECUTE_GRASP --> GRASP_SUCCESS
    
    GRASP_SUCCESS -->|成功| MOVE_TO_CONFIRM
    GRASP_SUCCESS -->|失敗| RETRY_GRASP[重試抓取]
    RETRY_GRASP --> PLAN_GRASP
    
    MOVE_TO_CONFIRM --> CAPTURE_CONFIRM
    CAPTURE_CONFIRM --> LLM_VERIFY
    LLM_VERIFY --> COMBINE_RESULTS
    COMBINE_RESULTS --> LLM_SUCCESS
    
    LLM_SUCCESS -->|確認成功| DISPENSE
    LLM_SUCCESS -->|確認失敗| RETRY_LLM[重新確認]
    RETRY_LLM --> CAPTURE_CONFIRM
    
    DISPENSE --> UPDATE_STATUS
    UPDATE_STATUS --> LOG_PROCESS
    LOG_PROCESS --> NOTIFY_COMPLETE
    NOTIFY_COMPLETE --> END([處理完成])
```

## 🔄 數據流向圖

```mermaid
graph LR
    subgraph "輸入數據 Input Data"
        PATIENT_INFO[病人資訊]
        MEDICINE_LIST[藥物清單]
        DOCTOR_INFO[醫師資訊]
    end
    
    subgraph "JSON配置流 JSON Config Flow"
        JSON_DB[(預存JSON配置)]
        JSON_EXTRACT[提取配置]
        JSON_PARAMS[檢測參數<br/>抓取參數<br/>確認參數]
    end
    
    subgraph "圖像數據流 Image Data Flow"
        RGB_CAMERA[RGB相機]
        DEPTH_CAMERA[深度相機]
        OCR_IMAGE[OCR圖像]
        DETECT_IMAGE[檢測圖像]
        CONFIRM_IMAGE[確認圖像]
    end
    
    subgraph "處理結果流 Processing Results Flow"
        OCR_TEXT[OCR文字結果]
        SAM2_MASK[SAM2遮罩結果]
        GRASP_POSE[抓取姿態]
        LLM_CONFIDENCE[LLM置信度]
    end
    
    subgraph "輸出數據 Output Data"
        ORDER_STATUS[訂單狀態]
        PROCESS_LOG[處理日誌]
        ROBOT_ACTION[機器人動作]
    end
    
    PATIENT_INFO --> JSON_DB
    MEDICINE_LIST --> JSON_DB
    JSON_DB --> JSON_EXTRACT
    JSON_EXTRACT --> JSON_PARAMS
    
    RGB_CAMERA --> OCR_IMAGE
    RGB_CAMERA --> DETECT_IMAGE
    RGB_CAMERA --> CONFIRM_IMAGE
    DEPTH_CAMERA --> DETECT_IMAGE
    
    OCR_IMAGE --> OCR_TEXT
    DETECT_IMAGE --> SAM2_MASK
    SAM2_MASK --> GRASP_POSE
    CONFIRM_IMAGE --> LLM_CONFIDENCE
    
    JSON_PARAMS --> GRASP_POSE
    JSON_PARAMS --> LLM_CONFIDENCE
    
    OCR_TEXT --> ORDER_STATUS
    GRASP_POSE --> ROBOT_ACTION
    LLM_CONFIDENCE --> ORDER_STATUS
    ROBOT_ACTION --> PROCESS_LOG
```

## 🎯 核心組件通信圖

```mermaid
sequenceDiagram
    participant Web as 網頁界面
    participant API as FastAPI
    participant DB as 資料庫
    participant Robot as 機器人系統
    participant OCR as OCR服務
    participant SAM2 as GroundedSAM2
    participant Grasp as 抓取服務
    participant LLM as LLM服務
    
    Web->>API: 創建訂單請求
    API->>DB: 儲存訂單和JSON配置
    API->>Robot: 發送訂單通知
    
    Robot->>API: 獲取待處理訂單
    API->>DB: 查詢訂單詳情
    DB-->>API: 返回訂單+JSON配置
    API-->>Robot: 訂單詳情
    
    Robot->>OCR: 請求OCR識別
    OCR-->>Robot: OCR結果
    
    Robot->>SAM2: 請求物品檢測
    SAM2-->>Robot: 檢測遮罩
    
    Robot->>Grasp: 請求抓取執行
    Grasp-->>Robot: 抓取結果
    
    Robot->>LLM: 請求確認識別
    LLM-->>Robot: 確認結果
    
    Robot->>API: 更新訂單狀態
    API->>DB: 標記訂單完成
    API-->>Web: 通知訂單完成
```

## 📁 文件組織架構

```
tm_robot/
├── src/
│   ├── user_interface/           # 網頁界面層
│   │   ├── main.py              # FastAPI主應用
│   │   ├── route/
│   │   │   ├── routes_orders.py # 訂單API路由
│   │   │   ├── routes_medicine.py
│   │   │   └── routes_prescription.py
│   │   ├── database/
│   │   │   ├── pharmacy_models.py    # 資料庫模型
│   │   │   ├── pharmacy_db.py        # 資料庫服務
│   │   │   ├── medicine_db.py
│   │   │   └── prescription_db.py
│   │   └── static/html/
│   │       └── orders.html       # 訂單管理頁面
│   │
│   ├── tm_robot_main/           # 機器人控制層
│   │   └── tm_robot_main/
│   │       ├── integrated_robot_system.py  # 整合機器人系統
│   │       ├── simple_pharmacy_system.py   # 簡化系統
│   │       └── main_control.py             # 主控制節點
│   │
│   └── graspnet/                # 抓取控制層
│       ├── graspnet/
│       │   ├── enhanced_small_object_grasp.py  # 增強抓取
│       │   ├── smart_object_classifier.py      # 智能分類器
│       │   └── small_object_config.yaml        # 抓取配置
│       └── launch/
│           ├── simple_pharmacy_system.launch.py
│           └── complete_pharmacy_system.launch.py
│
└── extra_package/               # 額外服務
    └── llm_drug_identification_system/
        └── main.py              # LLM藥物識別
```

## 🔧 技術棧說明

### 前端技術
- **HTML/CSS/JavaScript**: 網頁界面
- **FastAPI**: 後端API框架
- **Jinja2**: 模板引擎

### 資料庫技術
- **SQLite**: 輕量級資料庫
- **SQLModel**: ORM框架
- **JSON**: 配置存儲格式

### 機器人技術
- **ROS2**: 機器人作業系統
- **OpenCV**: 圖像處理
- **Open3D**: 3D點雲處理

### 人工智能技術
- **GroundedSAM2**: 物品檢測與分割
- **OCR**: 文字識別
- **LLM**: 大語言模型確認
- **機器學習**: 物件分類和抓取策略

這個架構圖清楚展示了從網頁訂單管理到機器人執行的完整流程，以及各個組件之間的詳細交互關係。