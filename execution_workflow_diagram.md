# 藥局系統執行流程圖

## 🚀 簡化執行流程

```mermaid
graph TD
    A[啟動系統] --> B[網頁創建訂單]
    B --> C[選擇病人]
    C --> D[添加藥物]
    D --> E[提交訂單]
    E --> F[自動儲存到資料庫]
    F --> G[機器人系統獲取訂單]
    G --> H[提取藥物JSON配置]
    H --> I[開始處理流程]
    
    I --> J[OCR文字識別]
    J --> K[GroundedSAM2物品檢測]
    K --> L[增強抓取執行]
    L --> M[移動到確認位置]
    M --> N[LLM二次確認]
    N --> O[完成訂單]
    O --> P[更新狀態為completed]
    P --> Q[通知網頁端完成]
```

## 📱 用戶操作步驟

```mermaid
journey
    title 藥局系統操作流程
    section 網頁操作
      打開網頁: 5: 使用者
      搜尋病人: 4: 使用者
      選擇病人: 5: 使用者
      添加藥物: 4: 使用者
      確認訂單: 5: 使用者
    section 自動處理
      儲存訂單: 5: 系統
      機器人接收: 5: 機器人
      OCR識別: 4: 機器人
      物品檢測: 5: 機器人
      執行抓取: 4: 機器人
      LLM確認: 5: 機器人
    section 完成
      更新狀態: 5: 系統
      通知完成: 5: 使用者
```

## 🔧 核心技術整合流程

```mermaid
flowchart LR
    subgraph "前端 Frontend"
        A[訂單管理頁面]
    end
    
    subgraph "後端 Backend"
        B[FastAPI]
        C[SQLite DB]
    end
    
    subgraph "機器人 Robot"
        D[訂單監控]
        E[OCR服務]
        F[GroundedSAM2]
        G[增強抓取]
        H[LLM確認]
    end
    
    A --> B
    B --> C
    C -.->|JSON配置| D
    D --> E
    E --> F
    F --> G
    G --> H
    H --> B
    B --> A
    
    style A fill:#e1f5fe
    style B fill:#f3e5f5
    style C fill:#fff3e0
    style D fill:#e8f5e8
    style E fill:#fff9c4
    style F fill:#ffecb3
    style G fill:#fce4ec
    style H fill:#e3f2fd
```

## 📊 數據流轉示意圖

```mermaid
sankey-beta
  web界面,資料庫,100
  資料庫,機器人系統,100
  機器人系統,OCR服務,25
  機器人系統,GroundedSAM2,50
  機器人系統,增強抓取,75
  機器人系統,LLM確認,25
  OCR服務,整合結果,25
  GroundedSAM2,整合結果,50
  增強抓取,整合結果,75
  LLM確認,整合結果,25
  整合結果,訂單完成,175
  訂單完成,web界面,175
```

## 🎯 關鍵決策點

```mermaid
flowchart TD
    START[開始處理訂單] --> CHECK1{OCR識別成功?}
    CHECK1 -->|是| CHECK2{GroundedSAM2檢測成功?}
    CHECK1 -->|否| CONTINUE1[繼續進行，不依賴OCR]
    CONTINUE1 --> CHECK2
    
    CHECK2 -->|是| CHECK3{抓取執行成功?}
    CHECK2 -->|否| RETRY1[重試檢測最多3次]
    RETRY1 --> CHECK2
    
    CHECK3 -->|是| CHECK4{LLM確認成功?}
    CHECK3 -->|否| RETRY2[重試抓取最多3次]
    RETRY2 --> CHECK3
    
    CHECK4 -->|是| SUCCESS[訂單完成]
    CHECK4 -->|否| RETRY3[重新確認最多3次]
    RETRY3 --> CHECK4
    
    style SUCCESS fill:#c8e6c9
    style RETRY1 fill:#ffcdd2
    style RETRY2 fill:#ffcdd2
    style RETRY3 fill:#ffcdd2
```

## 🏃‍♂️ 快速啟動指令

```bash
# 1. 啟動完整系統
ros2 launch graspnet simple_pharmacy_system.launch.py

# 2. 單獨啟動組件
ros2 run tm_robot_main integrated_robot_system

# 3. 啟動網頁界面
cd tm_robot/src/user_interface
python main.py

# 4. 訪問網頁
http://localhost:8000
```

## 📋 系統狀態監控

```mermaid
gitgraph
    commit id: "系統啟動"
    commit id: "資料庫初始化"
    commit id: "服務連接"
    branch order-processing
    commit id: "接收訂單"
    commit id: "OCR識別"
    commit id: "SAM2檢測"
    commit id: "執行抓取"
    commit id: "LLM確認"
    checkout main
    merge order-processing
    commit id: "訂單完成"
    commit id: "狀態更新"
```

## 🔍 故障診斷流程

```mermaid
flowchart TD
    ERROR[系統錯誤] --> TYPE{錯誤類型}
    
    TYPE -->|網頁| WEB_ERROR[檢查FastAPI服務]
    TYPE -->|資料庫| DB_ERROR[檢查SQLite連接]
    TYPE -->|機器人| ROBOT_ERROR[檢查ROS2服務]
    TYPE -->|視覺| VISION_ERROR[檢查相機連接]
    
    WEB_ERROR --> WEB_FIX[重啟API服務]
    DB_ERROR --> DB_FIX[檢查資料庫檔案]
    ROBOT_ERROR --> ROBOT_FIX[檢查ROS2節點]
    VISION_ERROR --> VISION_FIX[檢查攝像頭驅動]
    
    WEB_FIX --> TEST[測試功能]
    DB_FIX --> TEST
    ROBOT_FIX --> TEST
    VISION_FIX --> TEST
    
    TEST --> PASS{測試通過?}
    PASS -->|是| SOLVED[問題解決]
    PASS -->|否| CONTACT[聯繫技術支援]
```