#!/usr/bin/env python3
"""
Complete Hospital Medicine Management System
Enhanced with basic/detailed medicine info, prescription status management, and ROS2 integration
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.responses import HTMLResponse, JSONResponse
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, ForeignKey, Text
from sqlalchemy.orm import declarative_base, sessionmaker, relationship, Session
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
import uvicorn
import logging
import json
import os
import requests
import yaml
from pathlib import Path

# Database setup
DATABASE_URL = "sqlite:///./complete_hospital_medicine.db"
engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Database Models
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False, index=True)
    position = Column(String(100), nullable=False)  # 位置
    prompt = Column(String(255), nullable=False)    # 提示詞
    confidence = Column(Float, nullable=False)       # 信心值
    amount = Column(Integer, default=0)              # 庫存數量
    
class MedicineDetail(Base):
    __tablename__ = "medicine_detail"
    
    id = Column(Integer, primary_key=True, index=True)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    content = Column(Text, nullable=False)           # 詳細內容

class Prescription(Base):
    __tablename__ = "prescription"
    
    id = Column(Integer, primary_key=True, index=True)
    patient_name = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    status = Column(String(50), default="pending")  # pending, processing, completed, failed
    updated_at = Column(DateTime, default=datetime.utcnow)

class PrescriptionMedicine(Base):
    __tablename__ = "prescription_medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    prescription_id = Column(Integer, ForeignKey("prescription.id"), nullable=False)
    medicine_id = Column(Integer, ForeignKey("medicine.id"), nullable=False)
    amount = Column(Integer, nullable=False)

# Pydantic Models
class MedicineCreate(BaseModel):
    name: str
    position: str
    prompt: str
    confidence: float
    amount: int = 100

class MedicineDetailCreate(BaseModel):
    medicine_id: int
    content: str

class PrescriptionCreate(BaseModel):
    patient_name: str
    medicines: List[Dict[str, Any]]

class StatusUpdate(BaseModel):
    status: str

# FastAPI App
app = FastAPI(title="Complete Hospital Medicine System", version="2.0.0")

# Logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("hospital")

# Database functions
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_database():
    Base.metadata.create_all(bind=engine)
    db = SessionLocal()
    
    # Create sample medicines if none exist
    if db.query(Medicine).count() == 0:
        medicines = [
            Medicine(name="Aspirin", position="A1", prompt="pain_relief_tablet", confidence=0.95, amount=100),
            Medicine(name="Vitamin C", position="A2", prompt="vitamin_supplement", confidence=0.90, amount=50),
            Medicine(name="Ibuprofen", position="B1", prompt="anti_inflammatory", confidence=0.88, amount=75),
            Medicine(name="Paracetamol", position="B2", prompt="fever_reducer", confidence=0.92, amount=120),
            Medicine(name="Calcium", position="C1", prompt="bone_supplement", confidence=0.85, amount=60)
        ]
        
        for med in medicines:
            db.add(med)
        db.commit()
        
        # Add sample details
        details = [
            MedicineDetail(medicine_id=1, content="阿斯匹靈 - 非類固醇消炎藥，用於緩解疼痛、發燒和炎症。常用劑量為每次300-600mg，每日最多4次。副作用可能包括胃腸不適。"),
            MedicineDetail(medicine_id=2, content="維他命C - 水溶性維生素，增強免疫系統，促進膠原蛋白合成。建議每日攝取量為75-90mg，過量可能導致腹瀉。"),
            MedicineDetail(medicine_id=3, content="布洛芬 - 非類固醇消炎藥，具有止痛、退燒、消炎作用。成人每次200-400mg，每日最多1200mg。避免空腹服用。"),
            MedicineDetail(medicine_id=4, content="撲熱息痛 - 解熱鎮痛藥，用於緩解輕至中度疼痛和發燒。成人每次500-1000mg，每日最多4000mg。相對安全，但過量可能損害肝臟。"),
            MedicineDetail(medicine_id=5, content="鈣片 - 礦物質補充劑，維持骨骼健康。建議每日攝取量1000-1200mg，最好與維生素D一起服用以增加吸收。")
        ]
        
        for detail in details:
            db.add(detail)
        db.commit()
    
    db.close()

# Global variables for order management
current_processing_order = None
order_queue = []

# API Routes

@app.get("/")
async def root():
    return {"message": "Complete Hospital Medicine System", "status": "running", "version": "2.0.0"}

@app.get("/api/system/status")
async def get_system_status():
    return {
        "status": "running", 
        "ros_mode": "integrated",
        "database": "connected",
        "current_order": current_processing_order,
        "queue_length": len(order_queue)
    }

# Medicine Management APIs

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    """Get basic medicine information"""
    medicines = db.query(Medicine).all()
    return [
        {
            "id": m.id,
            "name": m.name,
            "position": m.position,
            "prompt": m.prompt,
            "confidence": m.confidence,
            "amount": m.amount
        } for m in medicines
    ]

@app.get("/api/medicine/detailed/{medicine_id}")
async def get_medicine_detail(medicine_id: int, db: Session = Depends(get_db)):
    """Get detailed medicine information"""
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).first()
    
    return {
        "id": medicine.id,
        "name": medicine.name,
        "position": medicine.position,
        "prompt": medicine.prompt,
        "confidence": medicine.confidence,
        "amount": medicine.amount,
        "content": detail.content if detail else "No detailed information available"
    }

@app.get("/api/medicine/detailed")
async def get_all_detailed_medicines(db: Session = Depends(get_db)):
    """Get all medicines with detailed information"""
    medicines = db.query(Medicine).all()
    result = []
    
    for medicine in medicines:
        detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine.id).first()
        result.append({
            "id": medicine.id,
            "name": medicine.name,
            "position": medicine.position,
            "prompt": medicine.prompt,
            "confidence": medicine.confidence,
            "amount": medicine.amount,
            "content": detail.content if detail else "No detailed information available"
        })
    
    return result

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    """Create new medicine"""
    db_medicine = Medicine(**medicine.dict())
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    return {"id": db_medicine.id, "name": db_medicine.name, "message": "Medicine created"}

@app.post("/api/medicine/{medicine_id}/detail")
async def create_medicine_detail(medicine_id: int, detail: MedicineDetailCreate, db: Session = Depends(get_db)):
    """Create or update medicine detail"""
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    existing_detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).first()
    if existing_detail:
        existing_detail.content = detail.content
    else:
        db_detail = MedicineDetail(medicine_id=medicine_id, content=detail.content)
        db.add(db_detail)
    
    db.commit()
    return {"message": "Medicine detail updated"}

# Prescription Management APIs

@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    """Get all prescriptions with detailed information"""
    prescriptions = db.query(Prescription).order_by(Prescription.created_at.desc()).all()
    result = []
    
    for p in prescriptions:
        prescription_medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == p.id
        ).all()
        
        medicines = []
        for pm in prescription_medicines:
            medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if medicine:
                medicines.append({
                    "id": medicine.id,
                    "name": medicine.name,
                    "amount": pm.amount,
                    "position": medicine.position,
                    "prompt": medicine.prompt
                })
        
        result.append({
            "id": p.id,
            "patient_name": p.patient_name,
            "created_at": p.created_at.isoformat() if p.created_at else None,
            "updated_at": p.updated_at.isoformat() if p.updated_at else None,
            "status": p.status,
            "medicines": medicines
        })
    
    return result

@app.post("/api/prescription/")
async def create_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    """Create new prescription"""
    global order_queue
    
    db_prescription = Prescription(patient_name=prescription.patient_name)
    db.add(db_prescription)
    db.commit()
    db.refresh(db_prescription)
    
    # Add medicines to prescription
    for med_data in prescription.medicines:
        pm = PrescriptionMedicine(
            prescription_id=db_prescription.id,
            medicine_id=med_data["medicine_id"],
            amount=med_data["amount"]
        )
        db.add(pm)
        
        # Deduct stock
        medicine = db.query(Medicine).filter(Medicine.id == med_data["medicine_id"]).first()
        if medicine:
            medicine.amount = max(0, medicine.amount - med_data["amount"])
    
    db.commit()
    
    # Add to order queue
    order_queue.append(db_prescription.id)
    logger.info(f"New prescription {db_prescription.id} added to queue. Queue length: {len(order_queue)}")
    
    return {"id": db_prescription.id, "message": "Prescription created and added to queue"}

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_update: StatusUpdate, db: Session = Depends(get_db)):
    """Update prescription status"""
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    
    prescription.status = status_update.status
    prescription.updated_at = datetime.utcnow()
    db.commit()
    
    logger.info(f"Prescription {prescription_id} status updated to: {status_update.status}")
    return {"message": "Status updated", "status": status_update.status}

# ROS2 Integration APIs

@app.get("/api/ros2/order/next")
async def get_next_order_for_ros2(db: Session = Depends(get_db)):
    """ROS2 endpoint to get next order for processing"""
    global current_processing_order, order_queue
    
    # If already processing an order, return 204
    if current_processing_order:
        return JSONResponse(status_code=204, content={"message": "Order currently being processed"})
    
    # Get oldest pending prescription
    if not order_queue:
        # Refresh queue with pending prescriptions
        pending_prescriptions = db.query(Prescription).filter(
            Prescription.status == "pending"
        ).order_by(Prescription.created_at.asc()).all()
        
        order_queue = [p.id for p in pending_prescriptions]
    
    if not order_queue:
        return JSONResponse(status_code=204, content={"message": "No orders to process"})
    
    # Get next order
    prescription_id = order_queue.pop(0)
    current_processing_order = prescription_id
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        current_processing_order = None
        return JSONResponse(status_code=404, content={"message": "Prescription not found"})
    
    # Update status to processing
    prescription.status = "processing"
    prescription.updated_at = datetime.utcnow()
    db.commit()
    
    # Get prescription medicines
    prescription_medicines = db.query(PrescriptionMedicine).filter(
        PrescriptionMedicine.prescription_id == prescription_id
    ).all()
    
    medicines = []
    for pm in prescription_medicines:
        medicine = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
        if medicine:
            medicines.append({
                "name": medicine.name,
                "amount": pm.amount,
                "position": medicine.position,
                "prompt": medicine.prompt,
                "confidence": medicine.confidence
            })
    
    order = {
        "order_id": f"{prescription.id:06d}",
        "prescription_id": prescription.id,
        "patient_name": prescription.patient_name,
        "medicine": medicines
    }
    
    logger.info(f"Sending order {prescription.id} to ROS2 for processing")
    
    return {
        "order": order,
        "yaml": yaml.safe_dump(order, allow_unicode=True, default_flow_style=False)
    }

@app.post("/api/ros2/order/complete")
async def complete_order_from_ros2(payload: Dict[str, Any], db: Session = Depends(get_db)):
    """ROS2 endpoint to report order completion"""
    global current_processing_order
    
    order_id = payload.get('order_id')
    status = payload.get('status', 'success')
    details = payload.get('details', '')
    
    if not order_id:
        raise HTTPException(status_code=400, detail="order_id is required")
    
    try:
        prescription_id = int(order_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid order_id format")
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    
    # Update prescription status
    if status == 'success':
        prescription.status = 'completed'
    else:
        prescription.status = 'failed'
    
    prescription.updated_at = datetime.utcnow()
    db.commit()
    
    # Clear current processing order
    if current_processing_order == prescription_id:
        current_processing_order = None
    
    logger.info(f"Order {prescription_id} completed with status: {status}")
    
    return {
        "message": "Order completed successfully",
        "prescription_id": prescription_id,
        "status": prescription.status,
        "next_available": len(order_queue) > 0 or db.query(Prescription).filter(Prescription.status == "pending").count() > 0
    }

@app.post("/api/ros2/order/progress")
async def update_order_progress(payload: Dict[str, Any]):
    """ROS2 endpoint to report order progress"""
    order_id = payload.get('order_id')
    stage = payload.get('stage', '')
    message = payload.get('message', '')
    
    logger.info(f"Order {order_id} progress - Stage: {stage}, Message: {message}")
    
    return {"status": "received", "order_id": order_id}

# Medicine Information ROS2 APIs

@app.get("/api/ros2/medicine/basic/{medicine_name}")
async def get_medicine_basic_info_ros2(medicine_name: str, db: Session = Depends(get_db)):
    """ROS2 endpoint to get basic medicine information by name"""
    medicine = db.query(Medicine).filter(Medicine.name.ilike(f"%{medicine_name}%")).first()
    
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    return {
        "name": medicine.name,
        "position": medicine.position,
        "prompt": medicine.prompt,
        "confidence": medicine.confidence,
        "amount": medicine.amount,
        "yaml": yaml.safe_dump({
            "name": medicine.name,
            "position": medicine.position,
            "prompt": medicine.prompt,
            "confidence": medicine.confidence,
            "amount": medicine.amount
        }, allow_unicode=True, default_flow_style=False)
    }

@app.get("/api/ros2/medicine/detailed/{medicine_name}")
async def get_medicine_detailed_info_ros2(medicine_name: str, db: Session = Depends(get_db)):
    """ROS2 endpoint to get detailed medicine information by name"""
    medicine = db.query(Medicine).filter(Medicine.name.ilike(f"%{medicine_name}%")).first()
    
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine.id).first()
    
    result = {
        "name": medicine.name,
        "position": medicine.position,
        "prompt": medicine.prompt,
        "confidence": medicine.confidence,
        "amount": medicine.amount,
        "content": detail.content if detail else "No detailed information available"
    }
    
    return {
        **result,
        "yaml": yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)
    }

# Web Interface
@app.get("/medicine.html", response_class=HTMLResponse)
async def medicine_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>藥物管理 - 醫院管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Microsoft JhengHei', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
        }
        
        .sidebar {
            position: fixed;
            left: 0;
            top: 0;
            width: 280px;
            height: 100vh;
            background: rgba(44, 62, 80, 0.95);
            backdrop-filter: blur(10px);
            z-index: 1000;
            padding: 20px;
            box-shadow: 4px 0 20px rgba(0, 0, 0, 0.3);
        }
        
        .sidebar .logo {
            text-align: center;
            margin-bottom: 40px;
            padding-bottom: 20px;
            border-bottom: 2px solid rgba(255, 255, 255, 0.1);
        }
        
        .sidebar .logo h2 {
            color: #ecf0f1;
            font-size: 1.5em;
            font-weight: 700;
            margin-bottom: 5px;
        }
        
        .sidebar .logo p {
            color: #bdc3c7;
            font-size: 0.9em;
        }
        
        .nav-button {
            display: block;
            width: 100%;
            padding: 15px 20px;
            margin: 10px 0;
            background: linear-gradient(45deg, #3498db, #2980b9);
            color: white;
            text-decoration: none;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            text-align: left;
            display: flex;
            align-items: center;
            gap: 12px;
        }
        
        .nav-button:hover {
            background: linear-gradient(45deg, #2980b9, #1f4e79);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(52, 152, 219, 0.4);
        }
        
        .nav-button.active {
            background: linear-gradient(45deg, #e74c3c, #c0392b);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(231, 76, 60, 0.4);
        }
        
        .nav-button .icon {
            font-size: 18px;
            width: 24px;
            text-align: center;
        }
        
        .main-content {
            margin-left: 280px;
            padding: 30px;
            min-height: 100vh;
        }
        
        .page-header {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }
        
        .page-header h1 {
            color: #2c3e50;
            font-size: 2.8em;
            margin-bottom: 10px;
            font-weight: 700;
        }
        
        .content-card {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
        }
        
        .form-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 25px;
            margin-bottom: 30px;
        }
        
        .form-group {
            margin-bottom: 25px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #2c3e50;
            font-size: 15px;
        }
        
        .form-group input, .form-group textarea {
            width: 100%;
            padding: 15px 20px;
            border: 2px solid #e9ecef;
            border-radius: 12px;
            font-size: 15px;
            transition: all 0.3s ease;
            background: #f8f9fa;
            font-family: inherit;
        }
        
        .form-group input:focus, .form-group textarea:focus {
            outline: none;
            border-color: #667eea;
            background: white;
            box-shadow: 0 0 0 4px rgba(102, 126, 234, 0.1);
        }
        
        .btn {
            padding: 15px 30px;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 10px;
            text-decoration: none;
            margin: 8px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(102, 126, 234, 0.4);
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(40, 167, 69, 0.4);
        }
        
        .data-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 25px;
            background: white;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
        }
        
        .data-table th {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            padding: 20px;
            text-align: left;
            font-weight: 600;
            font-size: 14px;
        }
        
        .data-table td {
            padding: 20px;
            border-bottom: 1px solid #e9ecef;
            color: #495057;
            font-size: 14px;
        }
        
        .data-table tr:hover {
            background: #f8f9fa;
            transform: scale(1.01);
            transition: all 0.2s ease;
        }
        
        .status-badge {
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            display: inline-block;
        }
        
        .status-high { background: #d4edda; color: #155724; }
        .status-medium { background: #fff3cd; color: #856404; }
        .status-low { background: #f8d7da; color: #721c24; }
        
        .alert {
            padding: 20px 25px;
            border-radius: 12px;
            margin: 20px 0;
            font-weight: 500;
            border-left: 4px solid;
        }
        
        .alert-success {
            background: #d4edda;
            color: #155724;
            border-color: #28a745;
        }
        
        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border-color: #dc3545;
        }
        
        .loading {
            display: inline-block;
            width: 25px;
            height: 25px;
            border: 3px solid #f3f3f3;
            border-top: 3px solid #667eea;
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        
        .text-center { text-align: center; }
        .hidden { display: none !important; }
    </style>
</head>
<body>
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button active">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <div class="main-content">
        <div class="page-header">
            <h1>藥物管理系統</h1>
            <p>管理基本和詳細藥物資訊</p>
        </div>

        <div class="content-card">
            <h2>新增藥物基本資訊</h2>
            <div class="form-grid">
                <div class="form-group">
                    <label for="medicineName">藥物名稱 *</label>
                    <input type="text" id="medicineName" placeholder="請輸入藥物名稱">
                </div>
                <div class="form-group">
                    <label for="medicinePosition">位置 *</label>
                    <input type="text" id="medicinePosition" placeholder="例: A1">
                </div>
                <div class="form-group">
                    <label for="medicinePrompt">提示詞 *</label>
                    <input type="text" id="medicinePrompt" placeholder="例: pain_relief_tablet">
                </div>
                <div class="form-group">
                    <label for="medicineConfidence">信心值 *</label>
                    <input type="number" id="medicineConfidence" step="0.01" min="0" max="1" value="0.95">
                </div>
                <div class="form-group">
                    <label for="medicineAmount">庫存數量</label>
                    <input type="number" id="medicineAmount" value="100" min="0">
                </div>
            </div>
            <button class="btn btn-primary" onclick="addMedicine()">
                <span>➕</span> 新增藥物
            </button>
        </div>

        <div class="content-card">
            <h2>新增詳細資訊</h2>
            <div class="form-group">
                <label for="detailMedicineSelect">選擇藥物</label>
                <select id="detailMedicineSelect" style="width: 100%; padding: 15px 20px; border: 2px solid #e9ecef; border-radius: 12px; font-size: 15px;">
                    <option value="">請選擇藥物</option>
                </select>
            </div>
            <div class="form-group">
                <label for="medicineContent">詳細內容 *</label>
                <textarea id="medicineContent" rows="4" placeholder="請輸入詳細藥物資訊"></textarea>
            </div>
            <button class="btn btn-success" onclick="addMedicineDetail()">
                <span>📝</span> 新增詳細資訊
            </button>
        </div>

        <div class="content-card">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 20px;">
                <h2>藥物列表</h2>
                <button class="btn btn-success" onclick="loadMedicines()">
                    <span>🔄</span> 重新載入
                </button>
            </div>
            
            <table class="data-table">
                <thead>
                    <tr>
                        <th>ID</th>
                        <th>藥物名稱</th>
                        <th>位置</th>
                        <th>提示詞</th>
                        <th>信心值</th>
                        <th>庫存數量</th>
                        <th>詳細資訊</th>
                    </tr>
                </thead>
                <tbody id="medicineList">
                    <tr>
                        <td colspan="7" class="text-center">
                            <div class="loading"></div> 載入中...
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        let medicines = [];
        
        function showAlert(message, type = 'success') {
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.classList.contains('permanent')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const container = document.querySelector('.content-card');
            if (container) {
                container.insertBefore(alertDiv, container.firstChild);
                setTimeout(() => alertDiv.remove(), 5000);
            }
        }

        async function addMedicine() {
            const name = document.getElementById('medicineName').value.trim();
            const position = document.getElementById('medicinePosition').value.trim();
            const prompt = document.getElementById('medicinePrompt').value.trim();
            const confidence = parseFloat(document.getElementById('medicineConfidence').value);
            const amount = parseInt(document.getElementById('medicineAmount').value) || 0;

            if (!name || !position || !prompt || isNaN(confidence)) {
                showAlert('請填寫所有必要欄位', 'error');
                return;
            }

            if (confidence < 0 || confidence > 1) {
                showAlert('信心值必須在 0 到 1 之間', 'error');
                return;
            }

            const data = {
                name: name,
                position: position,
                prompt: prompt,
                confidence: confidence,
                amount: amount
            };

            try {
                const response = await fetch('/api/medicine/', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    showAlert('藥物新增成功: ' + result.name, 'success');
                    clearMedicineForm();
                    loadMedicines();
                    loadMedicineSelect();
                } else {
                    throw new Error('新增失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        async function addMedicineDetail() {
            const medicineId = document.getElementById('detailMedicineSelect').value;
            const content = document.getElementById('medicineContent').value.trim();

            if (!medicineId || !content) {
                showAlert('請選擇藥物並填寫詳細內容', 'error');
                return;
            }

            try {
                const response = await fetch(`/api/medicine/${medicineId}/detail`, {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({medicine_id: parseInt(medicineId), content: content})
                });
                
                if (response.ok) {
                    showAlert('詳細資訊新增成功', 'success');
                    document.getElementById('medicineContent').value = '';
                    loadMedicines();
                } else {
                    throw new Error('新增失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        async function loadMedicines() {
            try {
                const response = await fetch('/api/medicine/detailed');
                medicines = await response.json();
                displayMedicines();
            } catch (error) {
                showAlert('載入藥物資料失敗: ' + error.message, 'error');
            }
        }

        async function loadMedicineSelect() {
            try {
                const response = await fetch('/api/medicine/basic');
                const basicMedicines = await response.json();
                const select = document.getElementById('detailMedicineSelect');
                
                select.innerHTML = '<option value="">請選擇藥物</option>';
                basicMedicines.forEach(med => {
                    const option = document.createElement('option');
                    option.value = med.id;
                    option.textContent = med.name;
                    select.appendChild(option);
                });
            } catch (error) {
                console.error('載入藥物選項失敗:', error);
            }
        }

        function displayMedicines() {
            const tbody = document.getElementById('medicineList');
            if (medicines.length === 0) {
                tbody.innerHTML = '<tr><td colspan="7" class="text-center">尚無藥物資料</td></tr>';
                return;
            }

            tbody.innerHTML = medicines.map(med => `
                <tr>
                    <td>${med.id}</td>
                    <td><strong>${med.name}</strong></td>
                    <td>${med.position}</td>
                    <td>${med.prompt}</td>
                    <td>${med.confidence.toFixed(2)}</td>
                    <td><span class="status-badge ${med.amount > 50 ? 'status-high' : med.amount > 10 ? 'status-medium' : 'status-low'}">${med.amount}</span></td>
                    <td>${med.content ? '✅ 已設定' : '❌ 未設定'}</td>
                </tr>
            `).join('');
        }

        function clearMedicineForm() {
            document.getElementById('medicineName').value = '';
            document.getElementById('medicinePosition').value = '';
            document.getElementById('medicinePrompt').value = '';
            document.getElementById('medicineConfidence').value = '0.95';
            document.getElementById('medicineAmount').value = '100';
        }

        document.addEventListener('DOMContentLoaded', function() {
            loadMedicines();
            loadMedicineSelect();
        });
    </script>
</body>
</html>
    """

@app.get("/doctor.html", response_class=HTMLResponse)
async def doctor_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>醫生工作台 - 醫院管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Microsoft JhengHei', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
        }
        
        .sidebar {
            position: fixed;
            left: 0;
            top: 0;
            width: 280px;
            height: 100vh;
            background: rgba(44, 62, 80, 0.95);
            backdrop-filter: blur(10px);
            z-index: 1000;
            padding: 20px;
            box-shadow: 4px 0 20px rgba(0, 0, 0, 0.3);
        }
        
        .sidebar .logo {
            text-align: center;
            margin-bottom: 40px;
            padding-bottom: 20px;
            border-bottom: 2px solid rgba(255, 255, 255, 0.1);
        }
        
        .sidebar .logo h2 {
            color: #ecf0f1;
            font-size: 1.5em;
            font-weight: 700;
            margin-bottom: 5px;
        }
        
        .sidebar .logo p {
            color: #bdc3c7;
            font-size: 0.9em;
        }
        
        .nav-button {
            display: block;
            width: 100%;
            padding: 15px 20px;
            margin: 10px 0;
            background: linear-gradient(45deg, #3498db, #2980b9);
            color: white;
            text-decoration: none;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            text-align: left;
            display: flex;
            align-items: center;
            gap: 12px;
        }
        
        .nav-button:hover {
            background: linear-gradient(45deg, #2980b9, #1f4e79);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(52, 152, 219, 0.4);
        }
        
        .nav-button.active {
            background: linear-gradient(45deg, #e74c3c, #c0392b);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(231, 76, 60, 0.4);
        }
        
        .nav-button .icon {
            font-size: 18px;
            width: 24px;
            text-align: center;
        }
        
        .main-content {
            margin-left: 280px;
            padding: 30px;
            min-height: 100vh;
        }
        
        .page-header {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }
        
        .page-header h1 {
            color: #2c3e50;
            font-size: 2.8em;
            margin-bottom: 10px;
            font-weight: 700;
        }
        
        .content-card {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
        }
        
        .form-group {
            margin-bottom: 25px;
        }
        
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #2c3e50;
            font-size: 15px;
        }
        
        .form-group input {
            width: 100%;
            padding: 15px 20px;
            border: 2px solid #e9ecef;
            border-radius: 12px;
            font-size: 15px;
            transition: all 0.3s ease;
            background: #f8f9fa;
            font-family: inherit;
        }
        
        .form-group input:focus {
            outline: none;
            border-color: #667eea;
            background: white;
            box-shadow: 0 0 0 4px rgba(102, 126, 234, 0.1);
        }
        
        .btn {
            padding: 15px 30px;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 10px;
            text-decoration: none;
            margin: 8px;
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(102, 126, 234, 0.4);
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(40, 167, 69, 0.4);
        }
        
        .medicine-card {
            background: white;
            border-radius: 12px;
            padding: 25px;
            margin: 15px 0;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
            border: 2px solid #e9ecef;
            transition: all 0.3s ease;
            cursor: pointer;
        }
        
        .medicine-card:hover {
            transform: translateY(-5px);
            box-shadow: 0 12px 35px rgba(0, 0, 0, 0.15);
            border-color: #667eea;
        }
        
        .medicine-card.selected {
            border-color: #667eea;
            background: linear-gradient(135deg, rgba(102, 126, 234, 0.1), rgba(118, 75, 162, 0.1));
        }
        
        .alert {
            padding: 20px 25px;
            border-radius: 12px;
            margin: 20px 0;
            font-weight: 500;
            border-left: 4px solid;
        }
        
        .alert-success {
            background: #d4edda;
            color: #155724;
            border-color: #28a745;
        }
        
        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border-color: #dc3545;
        }
        
        .loading {
            display: inline-block;
            width: 25px;
            height: 25px;
            border: 3px solid #f3f3f3;
            border-top: 3px solid #667eea;
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        
        .text-center { text-align: center; }
        .hidden { display: none !important; }
    </style>
</head>
<body>
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button active">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <div class="main-content">
        <div class="page-header">
            <h1>醫生工作台</h1>
            <p>透過拉取現有藥物配合數量建立處方籤</p>
        </div>

        <div class="content-card">
            <h2>開立新處方籤</h2>
            <div class="form-group">
                <label for="patientName">病患姓名 *</label>
                <input type="text" id="patientName" placeholder="請輸入病患姓名">
            </div>

            <div class="form-group">
                <label>藥物選擇</label>
                <button class="btn btn-success mb-3" onclick="loadAvailableMedicines()">
                    <span>🔄</span> 載入可用藥物
                </button>
                <div id="medicineSelection" style="max-height: 400px; overflow-y: auto; border: 1px solid #e9ecef; border-radius: 12px; padding: 20px; background: #f8f9fa;">
                    <p class="text-center">請點擊上方按鈕載入藥物資料</p>
                </div>
            </div>

            <div id="selectedMedicinesDiv" class="hidden" style="background: #e8f5e8; border-radius: 12px; padding: 20px; margin: 20px 0;">
                <h3>已選擇的藥物</h3>
                <div id="selectedMedicinesList"></div>
            </div>

            <button class="btn btn-primary" onclick="createPrescription()">
                <span>📝</span> 開立處方籤
            </button>
        </div>
    </div>

    <script>
        let availableMedicines = [];
        let selectedMedicines = [];

        function showAlert(message, type = 'success') {
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.classList.contains('permanent')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const container = document.querySelector('.content-card');
            if (container) {
                container.insertBefore(alertDiv, container.firstChild);
                setTimeout(() => alertDiv.remove(), 5000);
            }
        }

        async function loadAvailableMedicines() {
            const container = document.getElementById('medicineSelection');
            container.innerHTML = '<p class="text-center"><span class="loading"></span> 載入中...</p>';
            
            try {
                const response = await fetch('/api/medicine/basic');
                availableMedicines = await response.json();
                displayMedicineSelection();
            } catch (error) {
                container.innerHTML = '<div class="alert alert-error">載入失敗: ' + error.message + '</div>';
            }
        }

        function displayMedicineSelection() {
            const container = document.getElementById('medicineSelection');
            
            if (availableMedicines.length === 0) {
                container.innerHTML = '<p class="text-center">尚無可用藥物</p>';
                return;
            }

            container.innerHTML = availableMedicines.map(med => `
                <div class="medicine-card" onclick="toggleMedicine(${med.id})" id="med-card-${med.id}">
                    <div style="display: flex; justify-content: space-between; align-items: center;">
                        <div>
                            <h4>${med.name}</h4>
                            <p><strong>位置:</strong> ${med.position} | <strong>庫存:</strong> ${med.amount}</p>
                            <p><strong>提示詞:</strong> ${med.prompt}</p>
                            <p><strong>信心值:</strong> ${med.confidence.toFixed(2)}</p>
                        </div>
                        <div>
                            <input type="number" id="amount-${med.id}" min="1" max="${med.amount}" value="1" 
                                   style="width: 80px; padding: 8px; border: 2px solid #e9ecef; border-radius: 8px;" onclick="event.stopPropagation();">
                            <span style="font-size: 12px; color: #666; display: block; text-align: center; margin-top: 5px;">數量</span>
                        </div>
                    </div>
                </div>
            `).join('');
        }

        function toggleMedicine(medId) {
            const card = document.getElementById(`med-card-${medId}`);
            const medicine = availableMedicines.find(m => m.id === medId);
            const amount = parseInt(document.getElementById(`amount-${medId}`).value) || 1;
            
            const existingIndex = selectedMedicines.findIndex(m => m.medicine_id === medId);
            
            if (existingIndex >= 0) {
                selectedMedicines.splice(existingIndex, 1);
                card.classList.remove('selected');
            } else {
                selectedMedicines.push({
                    medicine_id: medId,
                    name: medicine.name,
                    amount: amount,
                    max_amount: medicine.amount
                });
                card.classList.add('selected');
            }
            
            updateSelectedMedicinesDisplay();
        }

        function updateSelectedMedicinesDisplay() {
            const container = document.getElementById('selectedMedicinesDiv');
            const list = document.getElementById('selectedMedicinesList');
            
            if (selectedMedicines.length === 0) {
                container.classList.add('hidden');
                return;
            }
            
            container.classList.remove('hidden');
            list.innerHTML = selectedMedicines.map((med, index) => `
                <div style="display: flex; justify-content: space-between; align-items: center; padding: 15px; border: 1px solid #ddd; border-radius: 12px; margin: 10px 0; background: white;">
                    <span><strong>${med.name}</strong> - ${med.amount} 顆</span>
                    <button class="btn" style="background: #dc3545; color: white; padding: 8px 16px;" onclick="removeSelectedMedicine(${index})">移除</button>
                </div>
            `).join('');
        }

        function removeSelectedMedicine(index) {
            const med = selectedMedicines[index];
            selectedMedicines.splice(index, 1);
            
            const card = document.getElementById(`med-card-${med.medicine_id}`);
            if (card) {
                card.classList.remove('selected');
            }
            
            updateSelectedMedicinesDisplay();
        }

        async function createPrescription() {
            const patientName = document.getElementById('patientName').value.trim();
            
            if (!patientName) {
                showAlert('請輸入病患姓名', 'error');
                return;
            }
            
            if (selectedMedicines.length === 0) {
                showAlert('請選擇至少一種藥物', 'error');
                return;
            }
            
            selectedMedicines.forEach(med => {
                const input = document.getElementById(`amount-${med.medicine_id}`);
                if (input) {
                    med.amount = parseInt(input.value) || 1;
                }
            });
            
            const data = {
                patient_name: patientName,
                medicines: selectedMedicines.map(med => ({
                    medicine_id: med.medicine_id,
                    amount: med.amount
                }))
            };
            
            try {
                const response = await fetch('/api/prescription/', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
                
                if (response.ok) {
                    const result = await response.json();
                    showAlert(`處方籤開立成功！處方籤編號: ${result.id}，已加入處理佇列`, 'success');
                    clearPrescriptionForm();
                } else {
                    throw new Error('處方籤開立失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        function clearPrescriptionForm() {
            document.getElementById('patientName').value = '';
            selectedMedicines = [];
            document.querySelectorAll('.medicine-card.selected').forEach(card => {
                card.classList.remove('selected');
            });
            updateSelectedMedicinesDisplay();
        }

        document.addEventListener('DOMContentLoaded', function() {
            loadAvailableMedicines();
        });
    </script>
</body>
</html>
    """

@app.get("/prescription.html", response_class=HTMLResponse)
async def prescription_page():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>處方籤管理 - 醫院管理系統</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Microsoft JhengHei', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            background-attachment: fixed;
            color: #2c3e50;
            line-height: 1.6;
            min-height: 100vh;
        }
        
        .sidebar {
            position: fixed;
            left: 0;
            top: 0;
            width: 280px;
            height: 100vh;
            background: rgba(44, 62, 80, 0.95);
            backdrop-filter: blur(10px);
            z-index: 1000;
            padding: 20px;
            box-shadow: 4px 0 20px rgba(0, 0, 0, 0.3);
        }
        
        .sidebar .logo {
            text-align: center;
            margin-bottom: 40px;
            padding-bottom: 20px;
            border-bottom: 2px solid rgba(255, 255, 255, 0.1);
        }
        
        .sidebar .logo h2 {
            color: #ecf0f1;
            font-size: 1.5em;
            font-weight: 700;
            margin-bottom: 5px;
        }
        
        .sidebar .logo p {
            color: #bdc3c7;
            font-size: 0.9em;
        }
        
        .nav-button {
            display: block;
            width: 100%;
            padding: 15px 20px;
            margin: 10px 0;
            background: linear-gradient(45deg, #3498db, #2980b9);
            color: white;
            text-decoration: none;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            text-align: left;
            display: flex;
            align-items: center;
            gap: 12px;
        }
        
        .nav-button:hover {
            background: linear-gradient(45deg, #2980b9, #1f4e79);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(52, 152, 219, 0.4);
        }
        
        .nav-button.active {
            background: linear-gradient(45deg, #e74c3c, #c0392b);
            transform: translateX(8px);
            box-shadow: 0 8px 25px rgba(231, 76, 60, 0.4);
        }
        
        .nav-button .icon {
            font-size: 18px;
            width: 24px;
            text-align: center;
        }
        
        .main-content {
            margin-left: 280px;
            padding: 30px;
            min-height: 100vh;
        }
        
        .page-header {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
            text-align: center;
        }
        
        .page-header h1 {
            color: #2c3e50;
            font-size: 2.8em;
            margin-bottom: 10px;
            font-weight: 700;
        }
        
        .content-card {
            background: white;
            padding: 40px;
            border-radius: 16px;
            box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
            margin-bottom: 30px;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 25px;
            margin-bottom: 30px;
        }
        
        .stat-card {
            background: white;
            padding: 30px;
            border-radius: 12px;
            text-align: center;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
            transition: transform 0.3s ease;
        }
        
        .stat-card:hover {
            transform: translateY(-5px);
        }
        
        .stat-number {
            font-size: 2.5em;
            font-weight: bold;
            color: #667eea;
            margin-bottom: 10px;
        }
        
        .stat-label {
            color: #666;
            font-size: 1em;
            text-transform: uppercase;
            letter-spacing: 1px;
            font-weight: 600;
        }
        
        .btn {
            padding: 15px 30px;
            border: none;
            border-radius: 12px;
            cursor: pointer;
            font-size: 15px;
            font-weight: 600;
            transition: all 0.3s ease;
            display: inline-flex;
            align-items: center;
            gap: 10px;
            text-decoration: none;
            margin: 8px;
        }
        
        .btn-success {
            background: linear-gradient(135deg, #28a745, #20c997);
            color: white;
        }
        
        .btn-success:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(40, 167, 69, 0.4);
        }
        
        .btn-warning {
            background: linear-gradient(135deg, #ffc107, #ff8f00);
            color: white;
        }
        
        .btn-warning:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(255, 193, 7, 0.4);
        }
        
        .btn-primary {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
        }
        
        .btn-primary:hover {
            transform: translateY(-3px);
            box-shadow: 0 12px 35px rgba(102, 126, 234, 0.4);
        }
        
        .data-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 25px;
            background: white;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 4px 20px rgba(0, 0, 0, 0.1);
        }
        
        .data-table th {
            background: linear-gradient(135deg, #667eea, #764ba2);
            color: white;
            padding: 20px;
            text-align: left;
            font-weight: 600;
            font-size: 14px;
        }
        
        .data-table td {
            padding: 20px;
            border-bottom: 1px solid #e9ecef;
            color: #495057;
            font-size: 14px;
            vertical-align: top;
        }
        
        .data-table tr:hover {
            background: #f8f9fa;
            transform: scale(1.01);
            transition: all 0.2s ease;
        }
        
        .status-badge {
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
            text-transform: uppercase;
            display: inline-block;
        }
        
        .status-pending {
            background: #fff3cd;
            color: #856404;
        }
        
        .status-processing {
            background: #d1ecf1;
            color: #0c5460;
        }
        
        .status-completed {
            background: #d4edda;
            color: #155724;
        }
        
        .status-failed {
            background: #f8d7da;
            color: #721c24;
        }
        
        .medicine-item {
            display: inline-block;
            background: #f8f9fa;
            padding: 6px 12px;
            margin: 2px;
            border-radius: 6px;
            border: 1px solid #e9ecef;
            font-size: 12px;
        }
        
        .alert {
            padding: 20px 25px;
            border-radius: 12px;
            margin: 20px 0;
            font-weight: 500;
            border-left: 4px solid;
        }
        
        .alert-success {
            background: #d4edda;
            color: #155724;
            border-color: #28a745;
        }
        
        .alert-error {
            background: #f8d7da;
            color: #721c24;
            border-color: #dc3545;
        }
        
        .loading {
            display: inline-block;
            width: 25px;
            height: 25px;
            border: 3px solid #f3f3f3;
            border-top: 3px solid #667eea;
            border-radius: 50%;
            animation: spin 1s linear infinite;
        }
        
        @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
        }
        
        .text-center { text-align: center; }
    </style>
</head>
<body>
    <div class="sidebar">
        <div class="logo">
            <h2>醫院管理系統</h2>
            <p>Hospital Management System</p>
        </div>
        
        <a href="/medicine.html" class="nav-button">
            <span class="icon">💊</span>
            <span>藥物管理</span>
        </a>
        
        <a href="/doctor.html" class="nav-button">
            <span class="icon">👨‍⚕️</span>
            <span>醫生工作台</span>
        </a>
        
        <a href="/prescription.html" class="nav-button active">
            <span class="icon">📋</span>
            <span>處方籤管理</span>
        </a>
    </div>

    <div class="main-content">
        <div class="page-header">
            <h1>處方籤管理系統</h1>
            <p>監控和管理處方籤狀態，支援ROS2自動處理</p>
        </div>

        <div class="stats-grid">
            <div class="stat-card">
                <div class="stat-number" id="pendingCount">-</div>
                <div class="stat-label">待處理</div>
            </div>
            <div class="stat-card">
                <div class="stat-number" id="processingCount">-</div>
                <div class="stat-label">處理中</div>
            </div>
            <div class="stat-card">
                <div class="stat-number" id="completedCount">-</div>
                <div class="stat-label">已完成</div>
            </div>
            <div class="stat-card">
                <div class="stat-number" id="totalCount">-</div>
                <div class="stat-label">總計</div>
            </div>
        </div>

        <div class="content-card">
            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 25px;">
                <h2>處方籤列表</h2>
                <div>
                    <button class="btn btn-success" onclick="loadPrescriptions()">
                        <span>🔄</span> 重新載入
                    </button>
                    <button class="btn btn-warning" onclick="toggleAutoRefresh()">
                        <span id="autoRefreshIcon">⏸️</span> <span id="autoRefreshText">停止自動更新</span>
                    </button>
                </div>
            </div>

            <table class="data-table">
                <thead>
                    <tr>
                        <th>處方籤編號</th>
                        <th>病患姓名</th>
                        <th>建立時間</th>
                        <th>更新時間</th>
                        <th>狀態</th>
                        <th>藥物清單</th>
                        <th>操作</th>
                    </tr>
                </thead>
                <tbody id="prescriptionList">
                    <tr>
                        <td colspan="7" class="text-center">
                            <div class="loading"></div> 載入中...
                        </td>
                    </tr>
                </tbody>
            </table>
        </div>
    </div>

    <script>
        let autoRefreshTimer = null;
        let isAutoRefreshing = true;

        function showAlert(message, type = 'success') {
            document.querySelectorAll('.alert').forEach(alert => {
                if (!alert.classList.contains('permanent')) {
                    alert.remove();
                }
            });

            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
            alertDiv.innerHTML = message;
            
            const container = document.querySelector('.content-card');
            if (container) {
                container.insertBefore(alertDiv, container.firstChild);
                setTimeout(() => alertDiv.remove(), 5000);
            }
        }

        async function loadPrescriptions() {
            const tbody = document.getElementById('prescriptionList');
            
            try {
                const response = await fetch('/api/prescription/');
                const prescriptions = await response.json();
                
                updateStats(prescriptions);
                
                if (prescriptions.length === 0) {
                    tbody.innerHTML = '<tr><td colspan="7" class="text-center">尚無處方籤資料</td></tr>';
                    return;
                }

                tbody.innerHTML = prescriptions.map(p => {
                    const createdDate = new Date(p.created_at);
                    const updatedDate = new Date(p.updated_at);
                    const medicineList = p.medicines.map(m => 
                        `<span class="medicine-item">${m.name} (${m.amount})</span>`
                    ).join(' ');
                    
                    let actions = '';
                    if (p.status === 'pending') {
                        actions = `<button class="btn btn-primary" onclick="updateStatus(${p.id}, 'processing')" style="padding: 8px 16px; font-size: 12px;">開始處理</button>`;
                    } else if (p.status === 'processing') {
                        actions = `<button class="btn btn-success" onclick="updateStatus(${p.id}, 'completed')" style="padding: 8px 16px; font-size: 12px;">標記完成</button>`;
                    }
                    
                    return `
                        <tr>
                            <td><strong>#${p.id.toString().padStart(6, '0')}</strong></td>
                            <td>${p.patient_name}</td>
                            <td>${createdDate.toLocaleString('zh-TW')}</td>
                            <td>${updatedDate.toLocaleString('zh-TW')}</td>
                            <td><span class="status-badge status-${p.status}">${getStatusText(p.status)}</span></td>
                            <td>${medicineList}</td>
                            <td>${actions}</td>
                        </tr>
                    `;
                }).join('');
                
            } catch (error) {
                tbody.innerHTML = `<tr><td colspan="7" class="text-center"><div class="alert alert-error">載入失敗: ${error.message}</div></td></tr>`;
            }
        }

        function updateStats(prescriptions) {
            const stats = {
                pending: 0,
                processing: 0,
                completed: 0,
                failed: 0,
                total: prescriptions.length
            };
            
            prescriptions.forEach(p => {
                if (stats.hasOwnProperty(p.status)) {
                    stats[p.status]++;
                }
            });
            
            document.getElementById('pendingCount').textContent = stats.pending;
            document.getElementById('processingCount').textContent = stats.processing;
            document.getElementById('completedCount').textContent = stats.completed;
            document.getElementById('totalCount').textContent = stats.total;
        }

        function getStatusText(status) {
            const statusMap = {
                'pending': '待處理',
                'processing': '處理中',
                'completed': '已完成',
                'failed': '失敗'
            };
            return statusMap[status] || status;
        }

        async function updateStatus(id, status) {
            try {
                const response = await fetch(`/api/prescription/${id}/status`, {
                    method: 'PUT',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({status: status})
                });
                
                if (response.ok) {
                    showAlert(`處方籤 #${id.toString().padStart(6, '0')} 狀態已更新為: ${getStatusText(status)}`, 'success');
                    loadPrescriptions();
                } else {
                    throw new Error('狀態更新失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        function toggleAutoRefresh() {
            if (isAutoRefreshing) {
                clearInterval(autoRefreshTimer);
                document.getElementById('autoRefreshIcon').textContent = '▶️';
                document.getElementById('autoRefreshText').textContent = '開始自動更新';
                isAutoRefreshing = false;
            } else {
                startAutoRefresh();
                document.getElementById('autoRefreshIcon').textContent = '⏸️';
                document.getElementById('autoRefreshText').textContent = '停止自動更新';
                isAutoRefreshing = true;
            }
        }

        function startAutoRefresh() {
            autoRefreshTimer = setInterval(loadPrescriptions, 3000);
        }

        document.addEventListener('DOMContentLoaded', function() {
            loadPrescriptions();
            startAutoRefresh();
        });
    </script>
</body>
</html>
    """

if __name__ == "__main__":
    print("Starting Complete Hospital Medicine Management System...")
    init_database()
    
    print("\n🏥 Complete Hospital System Ready!")
    print("=" * 50)
    print("📱 Web Interfaces:")
    print("  - Medicine Management: http://localhost:8001/medicine.html")
    print("  - Doctor Interface: http://localhost:8001/doctor.html") 
    print("  - Prescription Management: http://localhost:8001/prescription.html")
    print("\n🤖 ROS2 Integration Endpoints:")
    print("  - Get Next Order: GET /api/ros2/order/next")
    print("  - Complete Order: POST /api/ros2/order/complete")
    print("  - Progress Update: POST /api/ros2/order/progress")
    print("  - Medicine Basic Info: GET /api/ros2/medicine/basic/{name}")
    print("  - Medicine Detail Info: GET /api/ros2/medicine/detailed/{name}")
    print("\n📋 System Features:")
    print("  ✅ Basic Medicine Info (位置, 名稱, 提示詞, 信心值)")
    print("  ✅ Detailed Medicine Info (詳細內容)")
    print("  ✅ Prescription Status Management")
    print("  ✅ ROS2 Order Queue (一次處理一個)")
    print("  ✅ Automatic Status Updates")
    print("  ✅ Complete API Integration")
    
    uvicorn.run(app, host="0.0.0.0", port=8001)