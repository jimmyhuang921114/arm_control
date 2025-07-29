from sqlmodel import SQLModel, Field, Relationship
from typing import Optional, List
from datetime import datetime
import json


class Patient(SQLModel, table=True):
    """病人資料表"""
    id: Optional[int] = Field(default=None, primary_key=True)
    patient_id: str = Field(unique=True, index=True)  # 病歷號
    name: str
    age: Optional[int] = None
    gender: Optional[str] = None
    medical_record: Optional[str] = None  # 病歷摘要
    allergies: Optional[str] = None  # 過敏史
    created_at: datetime = Field(default_factory=datetime.now)
    
    # 關聯
    orders: List["Order"] = Relationship(back_populates="patient")


class Medicine(SQLModel, table=True):
    """藥物基本資料表"""
    id: Optional[int] = Field(default=None, primary_key=True)
    medicine_code: str = Field(unique=True, index=True)  # 藥物代碼
    name: str = Field(index=True)  # 藥物名稱
    description: Optional[str] = None  # 藥物描述
    manufacturer: Optional[str] = None  # 製造商
    category: Optional[str] = None  # 藥物分類
    unit: str = Field(default="片")  # 單位（片、粒、ml等）
    created_at: datetime = Field(default_factory=datetime.now)
    
    # 關聯
    json_configs: List["MedicineJsonConfig"] = Relationship(back_populates="medicine")
    order_items: List["OrderItem"] = Relationship(back_populates="medicine")


class MedicineJsonConfig(SQLModel, table=True):
    """藥物JSON配置表 - 預先儲存的配置"""
    id: Optional[int] = Field(default=None, primary_key=True)
    medicine_id: int = Field(foreign_key="medicine.id", index=True)
    config_name: str  # 配置名稱（如：標準配置、特殊處理等）
    json_data: str  # JSON配置內容
    is_default: bool = Field(default=False)  # 是否為預設配置
    description: Optional[str] = None  # 配置說明
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    
    # 關聯
    medicine: Medicine = Relationship(back_populates="json_configs")
    
    def get_json_data(self) -> dict:
        """解析JSON數據"""
        try:
            return json.loads(self.json_data)
        except:
            return {}
    
    def set_json_data(self, data: dict):
        """設置JSON數據"""
        self.json_data = json.dumps(data, ensure_ascii=False, indent=2)


class Order(SQLModel, table=True):
    """訂單表"""
    id: Optional[int] = Field(default=None, primary_key=True)
    order_id: str = Field(unique=True, index=True)  # 訂單號
    patient_id: int = Field(foreign_key="patient.id", index=True)
    status: str = Field(default="pending", index=True)  # pending, processing, completed, failed, cancelled
    priority: int = Field(default=1)  # 優先級 1-5
    doctor_name: Optional[str] = None  # 開立醫師
    department: Optional[str] = None  # 科別
    notes: Optional[str] = None  # 訂單備註
    created_at: datetime = Field(default_factory=datetime.now)
    processed_at: Optional[datetime] = None  # 開始處理時間
    completed_at: Optional[datetime] = None  # 完成時間
    
    # 關聯
    patient: Patient = Relationship(back_populates="orders")
    order_items: List["OrderItem"] = Relationship(back_populates="order")


class OrderItem(SQLModel, table=True):
    """訂單項目表"""
    id: Optional[int] = Field(default=None, primary_key=True)
    order_id: int = Field(foreign_key="order.id", index=True)
    medicine_id: int = Field(foreign_key="medicine.id", index=True)
    quantity: int  # 數量
    dosage: Optional[str] = None  # 劑量說明
    frequency: Optional[str] = None  # 服用頻率
    duration: Optional[str] = None  # 療程天數
    special_instructions: Optional[str] = None  # 特殊指示
    status: str = Field(default="pending")  # pending, picked, verified, completed, failed
    json_config_id: Optional[int] = Field(foreign_key="medicinejsonconfig.id")  # 使用的JSON配置
    picked_at: Optional[datetime] = None  # 挑選完成時間
    verified_at: Optional[datetime] = None  # 驗證完成時間
    
    # 關聯
    order: Order = Relationship(back_populates="order_items")
    medicine: Medicine = Relationship(back_populates="order_items")
    
    def get_medicine_json_config(self) -> Optional[dict]:
        """獲取該藥物的JSON配置"""
        # 如果有指定的配置，使用指定的
        if self.json_config_id:
            from database.pharmacy_db import get_session
            with get_session() as session:
                config = session.get(MedicineJsonConfig, self.json_config_id)
                if config:
                    return config.get_json_data()
        
        # 否則使用藥物的預設配置
        from database.pharmacy_db import get_session
        with get_session() as session:
            default_config = session.query(MedicineJsonConfig).filter(
                MedicineJsonConfig.medicine_id == self.medicine_id,
                MedicineJsonConfig.is_default == True
            ).first()
            
            if default_config:
                return default_config.get_json_data()
        
        return None


class ProcessingLog(SQLModel, table=True):
    """處理日誌表"""
    id: Optional[int] = Field(default=None, primary_key=True)
    order_id: str = Field(index=True)
    order_item_id: Optional[int] = Field(foreign_key="orderitem.id")
    stage: str  # 處理階段：picking, grasping, verifying, completing
    status: str  # success, failed, in_progress
    message: Optional[str] = None  # 詳細訊息
    confidence_score: Optional[float] = None  # 置信度分數
    processing_time: Optional[float] = None  # 處理時間（秒）
    created_at: datetime = Field(default_factory=datetime.now)


# 預定義的JSON配置範例
SAMPLE_JSON_CONFIGS = {
    "paracetamol": {
        "detection": {
            "confidence_threshold": 0.4,
            "size_threshold": 0.05,
            "color_range": "white",
            "shape": "round"
        },
        "grasp": {
            "strategy": "suction",
            "suction_offset": 0.003,
            "approach_angle": 90,
            "safety_margin": 0.002
        },
        "verification": {
            "llm_prompt": "識別白色圓形普拿疼藥片",
            "required_confidence": 0.8,
            "backup_verification": True
        },
        "handling": {
            "fragile": False,
            "temperature_sensitive": False,
            "light_sensitive": False,
            "special_storage": None
        }
    },
    "aspirin": {
        "detection": {
            "confidence_threshold": 0.35,
            "size_threshold": 0.06,
            "color_range": "white",
            "shape": "round"
        },
        "grasp": {
            "strategy": "suction",
            "suction_offset": 0.004,
            "approach_angle": 90,
            "safety_margin": 0.002
        },
        "verification": {
            "llm_prompt": "識別白色圓形阿斯匹靈藥片",
            "required_confidence": 0.8,
            "backup_verification": True
        },
        "handling": {
            "fragile": False,
            "temperature_sensitive": False,
            "light_sensitive": True,
            "special_storage": "避光保存"
        }
    },
    "ibuprofen": {
        "detection": {
            "confidence_threshold": 0.4,
            "size_threshold": 0.07,
            "color_range": "orange",
            "shape": "oval"
        },
        "grasp": {
            "strategy": "suction",
            "suction_offset": 0.005,
            "approach_angle": 85,
            "safety_margin": 0.003
        },
        "verification": {
            "llm_prompt": "識別橙色橢圓形布洛芬藥片",
            "required_confidence": 0.85,
            "backup_verification": True
        },
        "handling": {
            "fragile": False,
            "temperature_sensitive": False,
            "light_sensitive": False,
            "special_storage": None
        }
    }
}