from sqlmodel import Session, SQLModel, create_engine, select
from .pharmacy_models import (
    Patient, Medicine, MedicineJsonConfig, Order, OrderItem, 
    ProcessingLog, SAMPLE_JSON_CONFIGS
)
from typing import List, Optional
import json

# 資料庫連接
DATABASE_URL = "sqlite:///./pharmacy_system.db"
engine = create_engine(DATABASE_URL, echo=False)

def init_pharmacy_db():
    """初始化藥局資料庫"""
    SQLModel.metadata.create_all(engine)
    
    # 初始化範例數據
    with Session(engine) as session:
        # 檢查是否已有數據
        existing_medicines = session.exec(select(Medicine)).first()
        if not existing_medicines:
            _init_sample_data(session)

def get_session():
    """獲取資料庫會話"""
    return Session(engine)

def _init_sample_data(session: Session):
    """初始化範例數據"""
    
    # 1. 創建範例病人
    patients = [
        Patient(
            patient_id="P001",
            name="張小明",
            age=35,
            gender="男",
            medical_record="高血壓患者",
            allergies="青黴素過敏"
        ),
        Patient(
            patient_id="P002", 
            name="李小華",
            age=42,
            gender="女",
            medical_record="糖尿病患者",
            allergies="無"
        )
    ]
    
    for patient in patients:
        session.add(patient)
    
    # 2. 創建範例藥物
    medicines = [
        Medicine(
            medicine_code="MED001",
            name="paracetamol",
            description="普拿疼 500mg",
            manufacturer="廠商A",
            category="解熱鎮痛劑",
            unit="片"
        ),
        Medicine(
            medicine_code="MED002",
            name="aspirin",
            description="阿斯匹靈 100mg",
            manufacturer="廠商B", 
            category="抗血小板劑",
            unit="片"
        ),
        Medicine(
            medicine_code="MED003",
            name="ibuprofen",
            description="布洛芬 400mg",
            manufacturer="廠商C",
            category="非類固醇消炎藥",
            unit="片"
        )
    ]
    
    for medicine in medicines:
        session.add(medicine)
    
    session.commit()
    
    # 3. 創建JSON配置
    for medicine in medicines:
        if medicine.name in SAMPLE_JSON_CONFIGS:
            config = MedicineJsonConfig(
                medicine_id=medicine.id,
                config_name="標準配置",
                json_data=json.dumps(SAMPLE_JSON_CONFIGS[medicine.name], ensure_ascii=False, indent=2),
                is_default=True,
                description=f"{medicine.name}的標準處理配置"
            )
            session.add(config)
    
    session.commit()
    print("範例數據初始化完成")

class PharmacyService:
    """藥局服務類"""
    
    @staticmethod
    def get_patient_by_id(patient_id: str) -> Optional[Patient]:
        """根據病歷號查詢病人"""
        with get_session() as session:
            statement = select(Patient).where(Patient.patient_id == patient_id)
            return session.exec(statement).first()
    
    @staticmethod
    def search_patients(keyword: str) -> List[Patient]:
        """搜尋病人"""
        with get_session() as session:
            statement = select(Patient).where(
                (Patient.name.contains(keyword)) | 
                (Patient.patient_id.contains(keyword))
            )
            return list(session.exec(statement))
    
    @staticmethod
    def get_all_medicines() -> List[Medicine]:
        """獲取所有藥物"""
        with get_session() as session:
            statement = select(Medicine)
            return list(session.exec(statement))
    
    @staticmethod
    def get_medicine_by_name(name: str) -> Optional[Medicine]:
        """根據名稱獲取藥物"""
        with get_session() as session:
            statement = select(Medicine).where(Medicine.name == name)
            return session.exec(statement).first()
    
    @staticmethod
    def get_medicine_json_config(medicine_id: int, config_id: Optional[int] = None) -> Optional[dict]:
        """獲取藥物的JSON配置"""
        with get_session() as session:
            if config_id:
                # 使用指定配置
                config = session.get(MedicineJsonConfig, config_id)
            else:
                # 使用預設配置
                statement = select(MedicineJsonConfig).where(
                    MedicineJsonConfig.medicine_id == medicine_id,
                    MedicineJsonConfig.is_default == True
                )
                config = session.exec(statement).first()
            
            if config:
                return config.get_json_data()
            return None
    
    @staticmethod
    def create_order(patient_id: str, medicines: List[dict], **kwargs) -> str:
        """
        創建訂單
        medicines: [{"medicine_name": "paracetamol", "quantity": 2, "dosage": "500mg"}]
        """
        with get_session() as session:
            # 查詢病人
            patient = PharmacyService.get_patient_by_id(patient_id)
            if not patient:
                raise ValueError(f"找不到病人: {patient_id}")
            
            # 生成訂單號
            import uuid
            order_id = f"ORD{uuid.uuid4().hex[:8].upper()}"
            
            # 創建訂單
            order = Order(
                order_id=order_id,
                patient_id=patient.id,
                doctor_name=kwargs.get('doctor_name'),
                department=kwargs.get('department'),
                notes=kwargs.get('notes'),
                priority=kwargs.get('priority', 1)
            )
            session.add(order)
            session.commit()
            
            # 創建訂單項目
            for med_info in medicines:
                medicine = PharmacyService.get_medicine_by_name(med_info['medicine_name'])
                if not medicine:
                    raise ValueError(f"找不到藥物: {med_info['medicine_name']}")
                
                order_item = OrderItem(
                    order_id=order.id,
                    medicine_id=medicine.id,
                    quantity=med_info['quantity'],
                    dosage=med_info.get('dosage'),
                    frequency=med_info.get('frequency'),
                    duration=med_info.get('duration'),
                    special_instructions=med_info.get('special_instructions')
                )
                session.add(order_item)
            
            session.commit()
            return order_id
    
    @staticmethod
    def get_order_by_id(order_id: str) -> Optional[Order]:
        """根據訂單號獲取訂單"""
        with get_session() as session:
            statement = select(Order).where(Order.order_id == order_id)
            return session.exec(statement).first()
    
    @staticmethod
    def get_pending_orders() -> List[Order]:
        """獲取待處理訂單"""
        with get_session() as session:
            statement = select(Order).where(Order.status == "pending").order_by(Order.priority.desc(), Order.created_at)
            return list(session.exec(statement))
    
    @staticmethod
    def get_order_with_items_and_configs(order_id: str) -> Optional[dict]:
        """獲取訂單及其藥物和JSON配置"""
        with get_session() as session:
            # 獲取訂單
            order_statement = select(Order).where(Order.order_id == order_id)
            order = session.exec(order_statement).first()
            if not order:
                return None
            
            # 獲取訂單項目
            items_statement = select(OrderItem).where(OrderItem.order_id == order.id)
            order_items = list(session.exec(items_statement))
            
            # 組裝完整訂單信息
            order_data = {
                "order_id": order.order_id,
                "patient_id": order.patient.patient_id,
                "patient_name": order.patient.name,
                "status": order.status,
                "priority": order.priority,
                "doctor_name": order.doctor_name,
                "department": order.department,
                "created_at": order.created_at.isoformat(),
                "medicines": []
            }
            
            for item in order_items:
                # 獲取藥物信息
                medicine = session.get(Medicine, item.medicine_id)
                
                # 獲取JSON配置
                json_config = item.get_medicine_json_config()
                
                medicine_data = {
                    "name": medicine.name,
                    "description": medicine.description,
                    "quantity": item.quantity,
                    "dosage": item.dosage,
                    "frequency": item.frequency,
                    "duration": item.duration,
                    "special_instructions": item.special_instructions,
                    "status": item.status,
                    "json_config": json_config
                }
                
                order_data["medicines"].append(medicine_data)
            
            return order_data
    
    @staticmethod
    def update_order_status(order_id: str, status: str):
        """更新訂單狀態"""
        with get_session() as session:
            statement = select(Order).where(Order.order_id == order_id)
            order = session.exec(statement).first()
            if order:
                order.status = status
                if status == "processing":
                    from datetime import datetime
                    order.processed_at = datetime.now()
                elif status == "completed":
                    from datetime import datetime
                    order.completed_at = datetime.now()
                session.add(order)
                session.commit()
                return True
            return False
    
    @staticmethod
    def update_order_item_status(order_id: str, medicine_name: str, status: str):
        """更新訂單項目狀態"""
        with get_session() as session:
            # 獲取訂單
            order_statement = select(Order).where(Order.order_id == order_id)
            order = session.exec(order_statement).first()
            if not order:
                return False
            
            # 獲取藥物
            medicine_statement = select(Medicine).where(Medicine.name == medicine_name)
            medicine = session.exec(medicine_statement).first()
            if not medicine:
                return False
            
            # 更新訂單項目
            item_statement = select(OrderItem).where(
                OrderItem.order_id == order.id,
                OrderItem.medicine_id == medicine.id
            )
            order_item = session.exec(item_statement).first()
            if order_item:
                order_item.status = status
                if status == "picked":
                    from datetime import datetime
                    order_item.picked_at = datetime.now()
                elif status == "verified":
                    from datetime import datetime
                    order_item.verified_at = datetime.now()
                
                session.add(order_item)
                session.commit()
                return True
            return False
    
    @staticmethod
    def log_processing_step(order_id: str, stage: str, status: str, **kwargs):
        """記錄處理步驟"""
        with get_session() as session:
            log = ProcessingLog(
                order_id=order_id,
                stage=stage,
                status=status,
                message=kwargs.get('message'),
                confidence_score=kwargs.get('confidence_score'),
                processing_time=kwargs.get('processing_time')
            )
            session.add(log)
            session.commit()