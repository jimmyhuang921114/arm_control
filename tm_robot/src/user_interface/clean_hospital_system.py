#!/usr/bin/env python3
"""
Clean Hospital Medicine Management System
- Modified position format (1-2, 2-1)
- Doctor uses dropdown to select medicines
- Basic and detailed medicine info stored separately
- Both basic and detailed info required for submission
"""

from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime, ForeignKey, Text, Boolean
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
DATABASE_URL = "sqlite:///./clean_hospital_medicine.db"
engine = create_engine(DATABASE_URL, echo=False)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# Database Models
class Medicine(Base):
    __tablename__ = "medicine"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(255), nullable=False, index=True)
    position = Column(String(100), nullable=False)  # 位置 (格式: 1-2, 2-1)
    prompt = Column(String(255), nullable=False)    # 提示詞
    confidence = Column(Float, nullable=False)       # 信心值
    amount = Column(Integer, default=0)              # 庫存數量

    area = Column(Float, default=0, nullable = False )  
    category  = Column(String(100), default="", nullable=False)  # 文字型別         
    
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
    picked_up = Column(Boolean, default=False, nullable=False)

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
    content: str = ""
    area: float = 0.0
    category: str =""

class MedicineDetailCreate(BaseModel):
    medicine_id: int
    content: str

class PrescriptionCreate(BaseModel):
    patient_name: str
    medicines: List[Dict[str, Any]]

class StatusUpdate(BaseModel):
    status: str

# FastAPI App
app = FastAPI(title="Clean Hospital Medicine System", version="3.0.0")

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
    """Initialize clean database without test data"""
    Base.metadata.create_all(bind=engine)
    logger.info("Clean database initialized without test data")

# Global variables for order management
current_processing_order = None
order_queue = []

# API Routes

@app.get("/")
async def root():
    return {"message": "Clean Hospital Medicine System", "status": "running", "version": "3.0.0"}

@app.get("/api/system/status")
async def get_system_status():
    return {
        "status": "running", 
        "ros_mode": "integrated",
        "database": "connected",
        "current_order": current_processing_order,
        "queue_length": len(order_queue),
        "version": "3.0.0"
    }

# Medicine Management APIs

@app.get("/api/medicine/basic")
async def get_basic_medicines(db: Session = Depends(get_db)):
    """Get basic medicine information only"""
    medicines = db.query(Medicine).all()
    return [
        {
            "id": m.id,
            "name": m.name,
            "position": m.position,
            "prompt": m.prompt,
            "confidence": m.confidence,
            "amount": m.amount,
            "area": m.area,
            "category": m.category,
        } for m in medicines
    ]

@app.get("/api/medicine/detailed")
async def get_detailed_medicines(db: Session = Depends(get_db)):
    """Get detailed medicine information only"""
    medicines = db.query(Medicine).all()
    result = []
    
    for medicine in medicines:
        detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine.id).first()
        result.append({
            "id": medicine.id,
            "name": medicine.name,
            "content": detail.content if detail else ""
        })
    
    return result

@app.get("/api/medicine/combined")
async def get_combined_medicines(db: Session = Depends(get_db)):
    """Get combined basic and detailed medicine information"""
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
            "content": detail.content if detail else "",
            "area": medicine.area,
            "category": medicine.category,

        })
    
    return result

@app.get("/api/medicine/detailed/{medicine_id}")
async def get_medicine_detail(medicine_id: int, db: Session = Depends(get_db)):
    """Get detailed medicine information by ID"""
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
        "content": detail.content if detail else "",
        "area": medicine.area,
        "category": medicine.category,

    }

@app.post("/api/medicine/")
async def create_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    """Create new medicine with basic info and optional detailed info"""
    # Validate position format (should be like 1-2, 2-1, etc.)
    import re
    if not re.match(r'^\d+-\d+$', medicine.position):
        raise HTTPException(status_code=400, detail="位置格式錯誤，應為 1-2 或 2-1 的格式")
    
    # Create medicine basic info
    db_medicine = Medicine(
        name=medicine.name,
        position=medicine.position,
        prompt=medicine.prompt,
        confidence=medicine.confidence,
        amount=medicine.amount,
        area = medicine.area,
        category=medicine.category
    )
    db.add(db_medicine)
    db.commit()
    db.refresh(db_medicine)
    
    # Create medicine detailed info if content is provided
    if medicine.content.strip():
        db_detail = MedicineDetail(
            medicine_id=db_medicine.id,
            content=medicine.content
        )
        db.add(db_detail)
        db.commit()
        message = "藥物基本和詳細資訊已成功建立"
    else:
        message = "藥物基本資訊已成功建立"
    
    return {
        "id": db_medicine.id, 
        "name": db_medicine.name, 
        "message": message
    }

@app.put("/api/medicine/{medicine_id}")
async def update_medicine(medicine_id: int, medicine: MedicineCreate, db: Session = Depends(get_db)):
    """Update medicine with basic info and optional detailed info"""
    # Find existing medicine
    db_medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not db_medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    # Validate position format
    import re
    if not re.match(r'^\d+-\d+$', medicine.position):
        raise HTTPException(status_code=400, detail="位置格式錯誤，應為 1-2 或 2-1 的格式")
    
    # Update basic info
    db_medicine.name = medicine.name
    db_medicine.position = medicine.position
    db_medicine.prompt = medicine.prompt
    db_medicine.confidence = medicine.confidence
    db_medicine.amount = medicine.amount
    db_medicine.area = medicine.area
    db_medicine.category = medicine.category
    
    # Update detailed info
    detail = db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).first()
    if medicine.content.strip():
        if detail:
            detail.content = medicine.content
        else:
            db_detail = MedicineDetail(medicine_id=medicine_id, content=medicine.content)
            db.add(db_detail)
    elif detail:
        # If content is empty but detail exists, remove it
        db.delete(detail)
    
    db.commit()
    
    return {"message": "藥物資訊已更新"}

@app.delete("/api/medicine/{medicine_id}")
async def delete_medicine(medicine_id: int, db: Session = Depends(get_db)):
    """Delete medicine and its detailed info"""
    # Find medicine
    medicine = db.query(Medicine).filter(Medicine.id == medicine_id).first()
    if not medicine:
        raise HTTPException(status_code=404, detail="Medicine not found")
    
    # Delete detailed info first
    db.query(MedicineDetail).filter(MedicineDetail.medicine_id == medicine_id).delete()
    
    # Delete medicine
    db.delete(medicine)
    db.commit()
    
    return {"message": f"藥物 {medicine.name} 已刪除"}

# Prescription Management APIs

@app.get("/api/prescription/")
async def get_prescriptions(db: Session = Depends(get_db)):
    prescriptions = db.query(Prescription).order_by(Prescription.created_at.desc()).all()
    result = []
    for p in prescriptions:
        prescription_medicines = db.query(PrescriptionMedicine).filter(
            PrescriptionMedicine.prescription_id == p.id
        ).all()
        meds = []
        for pm in prescription_medicines:
            m = db.query(Medicine).filter(Medicine.id == pm.medicine_id).first()
            if m:
                meds.append({
                    "id": m.id, "name": m.name, "amount": pm.amount,
                    "position": m.position, "prompt": m.prompt
                })
        result.append({
            "id": p.id,
            "patient_name": p.patient_name,
            "created_at": p.created_at.isoformat() if p.created_at else None,
            "updated_at": p.updated_at.isoformat() if p.updated_at else None,
            "status": p.status,
            "picked_up": p.picked_up,  # ★ 帶出
            "medicines": meds
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

@app.post("/api/prescription/{prescription_id}/pickup")
async def mark_prescription_picked_up(prescription_id: int, db: Session = Depends(get_db)):
    p = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not p:
        raise HTTPException(status_code=404, detail="Prescription not found")
    # 只有完成才可領藥；未完成就拒絕
    if p.status != "completed":
        raise HTTPException(status_code=400, detail="Prescription not completed yet")
    if p.picked_up:
        return {"message": "Already picked up", "id": p.id, "picked_up": True}
    p.picked_up = True
    p.updated_at = datetime.utcnow()
    db.commit()
    logger.info(f"Prescription {p.id} marked as picked up")
    return {"message": "Picked up", "id": p.id, "picked_up": True}

@app.put("/api/prescription/{prescription_id}/status")
async def update_prescription_status(prescription_id: int, status_update: StatusUpdate, db: Session = Depends(get_db)):
    """Update prescription status"""
    global current_processing_order
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        raise HTTPException(status_code=404, detail="Prescription not found")
    
    prescription.status = status_update.status
    prescription.updated_at = datetime.utcnow()
    db.commit()

    if(prescription_id == current_processing_order and status_update.status in ("completed", "failed")):
        current_processing_order = None
    
    logger.info(f"Prescription {prescription_id} status updated to: {status_update.status}")
    return {"message": "Status updated", "status": status_update.status}



@app.get("/api/ros2/order/next")
async def get_next_order_for_ros2(db: Session = Depends(get_db)):
    """ROS2 endpoint to get next order for processing"""
    global current_processing_order, order_queue
    
    if current_processing_order:
        print("order in process")
        return Response(status_code=204)
    
    if not order_queue:
        pending_prescriptions = db.query(Prescription).filter(
            Prescription.status == "pending"
        ).order_by(Prescription.created_at.asc()).all()
        order_queue = [p.id for p in pending_prescriptions]
    
    if not order_queue:
        print("no order in queue")
        return Response(status_code=204)
    
    prescription_id = order_queue.pop(0)
    current_processing_order = prescription_id
    
    prescription = db.query(Prescription).filter(Prescription.id == prescription_id).first()
    if not prescription:
        current_processing_order = None
        return JSONResponse(status_code=404, content={"message": "Prescription not found"})
    
    prescription.status = "processing"
    prescription.updated_at = datetime.utcnow()
    db.commit()
    
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
                "confidence": medicine.confidence,
                "area": medicine.area,
                "category": medicine.category or ""
            })

    def _pos_key(pos: str):
        try:
            r, c = pos.split('-', 1)
            return (int(r.strip()), int(c.strip()))
        except Exception:
            # 位置壞掉時丟到最後
            return (10**9, 10**9)

    medicines.sort(key=lambda m: _pos_key(m.get("position", "")))

    # --- 打包訂單（用排序後的 medicines）---
    order = {
        "order_id": f"{prescription.id:06d}",
        "prescription_id": prescription.id,
        "patient_name": prescription.patient_name,
        "medicine": medicines
    }

    logger.info(f"Sending order {prescription.id} to ROS2 for processing")
    
    return {
        "order": order,
        "yaml": yaml.safe_dump(order, allow_unicode=True, default_flow_style=False),
        "prescription_id": prescription.id
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
    
    if status == 'success':
        prescription.status = 'completed'
    else:
        prescription.status = 'failed'
    
    prescription.updated_at = datetime.utcnow()
    db.commit()
    
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
    
    result = {
        "name": medicine.name,
        "position": medicine.position,
        "prompt": medicine.prompt,
        "confidence": medicine.confidence,
        "amount": medicine.amount,
        "area": medicine.area,
        "category": medicine.category or ""
    }
    
    return {
        **result,
        "yaml": yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)
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
        "content": detail.content if detail else ""
    }
    
    return {
        **result,
        "yaml": yaml.safe_dump(result, allow_unicode=True, default_flow_style=False)
    }
# —— 放在 route 區塊（例如 get_prescriptions 之後）——

def _status_text(s: str) -> str:
    mapping = {"pending":"待處理","processing":"處理中","completed":"已完成","failed":"失敗"}
    return mapping.get(s, s)

@app.get("/api/patient/{patient_name}/prescriptions")
async def get_patient_prescriptions_simple(patient_name: str, db: Session = Depends(get_db)):
    """
    病患端用：只回傳每一單的狀態（新→舊）
    """
    rows = db.query(Prescription)\
        .filter(Prescription.patient_name.ilike(f"%{patient_name}%"))\
        .order_by(Prescription.created_at.desc())\
        .all()
    return [{
        "id": p.id,
        "patient_name": p.patient_name,
        "status": _status_text(p.status),
        "created_at": p.created_at.isoformat() if p.created_at else None,
        "updated_at": p.updated_at.isoformat() if p.updated_at else None,
    } for p in rows]



# Web Interface Routes
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
        
        .required {
            color: #e74c3c;
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
        
        .position-help {
            font-size: 12px;
            color: #666;
            margin-top: 5px;
        }
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
        <!-- 改這一行：把 active 拿掉 -->
        <a href="/patient.html" class="nav-button">
        <span class="icon">🧑‍🦽</span><span>病患查詢</span>
        </a>

    </div>

    <div class="main-content">
        <div class="page-header">
            <h1>藥物管理系統</h1>
            <p>基本和詳細藥物資訊同時管理</p>
        </div>

                 <div class="content-card">
            <h2>新增藥物</h2>
            <div class="form-grid">
                <div class="form-group">
                    <label for="medicineName">藥物名稱 <span class="required">*</span></label>
                    <input type="text" id="medicineName" placeholder="請輸入藥物名稱">
                </div>
                <div class="form-group">
                    <label for="medicinePosition">位置 <span class="required">*</span></label>
                    <input type="text" id="medicinePosition" placeholder="例: 1-2, 2-1">
                    <div class="position-help">格式：行-列 (例如 1-2 表示第1行第2列)</div>
                </div>
                <div class="form-group">
                    <label for="medicinePrompt">提示詞 <span class="required">*</span></label>
                    <input type="text" id="medicinePrompt" placeholder="例: pain_relief_tablet">
                </div>
                <div class="form-group">
                    <label for="medicineConfidence">信心值 <span class="required">*</span></label>
                    <input type="number" id="medicineConfidence" step="0.01" min="0" max="1" value="0.95">
                </div>
                <div class="form-group">
                    <label for="medicineAmount">庫存數量</label>
                    <input type="number" id="medicineAmount" value="100" min="0">
                </div>
                <div class="form-group">
                    <label for="medicineArea">面積</label>
                    <input type="number" id="medicineArea" step="0.01" min="0">
                </div>
                <div class="form-group">
                    <label for="medicineCategory">種類</label>
                    <input type="text" id="medicineCategory" placeholder="例如：瓶裝、藥板、藥膏...">
                </div>

            </div>
            
                         <div class="form-group">
                 <label for="medicineContent">詳細內容 (可選)</label>
                 <textarea id="medicineContent" rows="4" placeholder="請輸入詳細藥物資訊（可選）"></textarea>
             </div>
            
                         <button class="btn btn-primary" onclick="addMedicine()">
                 <span>➕</span> 新增藥物
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
                        <th>面積</th>
                        <th>種類</th>
                        <th>操作</th>
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
            const content = document.getElementById('medicineContent').value.trim();
            let area = parseFloat(document.getElementById('medicineArea').value); 
            const category  = document.getElementById('medicineCategory').value.trim();


                         // Validate required fields
             if (!name || !position || !prompt || isNaN(confidence)) {
                 showAlert('請填寫所有必要欄位', 'error');
                 return;
             }

            // Validate position format
                         const positionRegex = /^\\d+-\\d+$/;
            if (!positionRegex.test(position)) {
                showAlert('位置格式錯誤，應為 1-2 或 2-1 的格式', 'error');
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
                amount: amount,
                content: content,
                area: area,
                category: category
            };

            try {
                const response = await fetch('/api/medicine/', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
                
                                 if (response.ok) {
                     const result = await response.json();
                     showAlert('藥物新增成功: ' + result.name + ' - ' + result.message, 'success');
                     clearMedicineForm();
                     loadMedicines();
                 } else {
                    const error = await response.json();
                    throw new Error(error.detail || '新增失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        async function loadMedicines() {
            try {
                const response = await fetch('/api/medicine/combined');
                medicines = await response.json();
                displayMedicines();
            } catch (error) {
                showAlert('載入藥物資料失敗: ' + error.message, 'error');
            }
        }

        function displayMedicines() {
            const tbody = document.getElementById('medicineList');
            if (medicines.length === 0) {
                tbody.innerHTML = '<tr><td colspan="8" class="text-center">尚無藥物資料</td></tr>';
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
                <td>${med.content ? '已設定' : '未設定'}</td>
                <td>${(med.area ?? 0).toFixed(2)}</td>
                <td>${med.category ? med.category : '-'}</td>
                <td>
                <button class="btn btn-primary" onclick="openEditMedicine(${med.id})">編輯</button>
                <button class="btn btn-success" style="background:linear-gradient(135deg,#dc3545,#c82333)" onclick="deleteMedicine(${med.id}, '${med.name.replace(/'/g, "\\'")}')">刪除</button>
                </td>
            </tr>
            `).join('');

            }


        function clearMedicineForm() {
            document.getElementById('medicineName').value = '';
            document.getElementById('medicinePosition').value = '';
            document.getElementById('medicinePrompt').value = '';
            document.getElementById('medicineConfidence').value = '0.95';
            document.getElementById('medicineAmount').value = '100';
            document.getElementById('medicineContent').value = '';
            document.getElementById('medicineArea').value = '';
            document.getElementById('medicineCategory').value = '';
        }
        let editingId = null;

        function openEditMedicine(id) {
        editingId = id;
        fetch(`/api/medicine/detailed/${id}`)
            .then(r => { if (!r.ok) throw new Error('載入詳細資料失敗'); return r.json(); })
            .then(data => {
            document.getElementById('editName').value = data.name || '';
            document.getElementById('editPosition').value = data.position || '';
            document.getElementById('editPrompt').value = data.prompt || '';
            document.getElementById('editConfidence').value = (data.confidence ?? 0.95);
            document.getElementById('editAmount').value = (data.amount ?? 0);
            document.getElementById('editContent').value = data.content || '';
            document.getElementById('editModal').classList.remove('hidden');
            document.getElementById('editArea').value     = (data.area ?? 0);
            document.getElementById('editCategory').value = data.category || '';
            })
            .catch(err => showAlert('錯誤：' + err.message, 'error'));
        }

        function closeEditModal() {
        document.getElementById('editModal').classList.add('hidden');
        editingId = null;
        }

        async function submitEdit() {
        if (!editingId) return;

        const name = document.getElementById('editName').value.trim();
        const position = document.getElementById('editPosition').value.trim();
        const prompt = document.getElementById('editPrompt').value.trim();
        const confidence = parseFloat(document.getElementById('editConfidence').value);
        const amount = parseInt(document.getElementById('editAmount').value) || 0;
        const content = document.getElementById('editContent').value.trim();
        let area   = parseFloat(document.getElementById('editArea').value);
        const category  = document.getElementById('editCategory').value.trim();

        if (!name || !position || !prompt || isNaN(confidence)) {
            showAlert('請填寫所有必要欄位', 'error'); return;
        }
        if (!/^\d+-\d+$/.test(position)) {
            showAlert('位置格式錯誤，應為 1-2 或 2-1 的格式', 'error'); return;
        }
        if (confidence < 0 || confidence > 1) {
            showAlert('信心值必須在 0~1 之間', 'error'); return;
        }

        const payload = { name, position, prompt, confidence, amount, content, area, category };    

        try {
            const resp = await fetch(`/api/medicine/${editingId}`, {
            method: 'PUT',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(payload)
            });
            if (!resp.ok) {
            const e = await resp.json().catch(()=>({detail:'更新失敗'}));
            throw new Error(e.detail || '更新失敗');
            }
            showAlert('藥物資訊已更新', 'success');
            closeEditModal();
            loadMedicines();
        } catch (err) {
            showAlert('錯誤：' + err.message, 'error');
        }
        }

        async function deleteMedicine(id, name) {
        if (!confirm(`確定刪除「${name}」？`)) return;
        try {
            const resp = await fetch(`/api/medicine/${id}`, { method: 'DELETE' });
            if (!resp.ok) {
            const e = await resp.json().catch(()=>({detail:'刪除失敗'}));
            throw new Error(e.detail || '刪除失敗');
            }
            showAlert(`已刪除：${name}`, 'success');
            loadMedicines();
        } catch (err) {
            showAlert('錯誤：' + err.message, 'error');
        }
        }

        document.addEventListener('DOMContentLoaded', function() {
            loadMedicines();
        });
    </script>
       <!-- ⬇⬇⬇ 把 Modal 貼在這裡（</body> 之前） ⬇⬇⬇ -->
    <!-- 編輯藥物 Modal -->
    <div id="editModal" class="hidden" style="
      position:fixed; inset:0; display:flex; align-items:center; justify-content:center;
      background:rgba(0,0,0,.4); z-index:2000;">
      <div style="background:#fff; width:min(700px,95vw); border-radius:12px; padding:24px; box-shadow:0 10px 40px rgba(0,0,0,.2);">
        <h3 style="margin-bottom:16px;">編輯藥物</h3>
        <div class="form-grid">
          <div class="form-group">
            <label>藥名 <span class="required">*</span></label>
            <input type="text" id="editName">
          </div>
          <div class="form-group">
            <label>位置 <span class="required">*</span></label>
            <input type="text" id="editPosition" placeholder="例: 1-2">
            <div class="position-help">格式：行-列（例 1-2）</div>
          </div>
          <div class="form-group">
            <label>提示詞 <span class="required">*</span></label>
            <input type="text" id="editPrompt" placeholder="例: white_bottle">
          </div>
          <div class="form-group">
            <label>信心值 <span class="required">*</span></label>
            <input type="number" id="editConfidence" step="0.01" min="0" max="1">
          </div>
          <div class="form-group">
            <label>庫存</label>
            <input type="number" id="editAmount" min="0">
          </div>
        <div class="form-group">
          <label>面積</label>
          <input type="number" id="editArea" step="0.01" min="0">
        </div>
        <div class="form-group">
          <label>種類</label>
          <input type="text" id="editCategory">
        </div>
        <div class="form-group" style="grid-column:1/-1">
            <label>詳細內容（可選）</label>
            <textarea id="editContent" rows="4"></textarea>
        </div>
        </div>
        <div style="display:flex; gap:10px; justify-content:flex-end; margin-top:8px;">
          <button class="btn" onclick="closeEditModal()">取消</button>
          <button class="btn btn-primary" onclick="submitEdit()">儲存</button>
        </div>
      </div>
    </div>
    <!-- ⬆⬆⬆ 貼在這裡 ⬆⬆⬆ -->
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
        
        .form-group input, .form-group select {
            width: 100%;
            padding: 15px 20px;
            border: 2px solid #e9ecef;
            border-radius: 12px;
            font-size: 15px;
            transition: all 0.3s ease;
            background: #f8f9fa;
            font-family: inherit;
        }
        
        .form-group input:focus, .form-group select:focus {
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
        
        .btn-danger {
            background: linear-gradient(135deg, #dc3545, #c82333);
            color: white;
        }
        
        .medicine-selection {
            background: #f8f9fa;
            border: 2px solid #e9ecef;
            border-radius: 12px;
            padding: 20px;
            margin: 20px 0;
        }
        
        .medicine-item {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 15px;
            margin: 10px 0;
            background: white;
            border-radius: 8px;
            border: 1px solid #dee2e6;
        }
        
        .medicine-info {
            flex: 1;
        }
        
        .medicine-controls {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .quantity-input {
            width: 80px;
            padding: 8px;
            border: 1px solid #dee2e6;
            border-radius: 6px;
            text-align: center;
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
        <!-- 改這一行：把 active 拿掉 -->
        <a href="/patient.html" class="nav-button">
        <span class="icon">🧑‍🦽</span><span>病患查詢</span>
        </a>

    </div>

    <div class="main-content">
        <div class="page-header">
            <h1>醫生工作台</h1>
            <p>使用下拉表格選擇藥物開立處方籤</p>
        </div>

        <div class="content-card">
            <h2>開立新處方籤</h2>
            
            <div class="form-group">
                <label for="patientName">病患姓名 *</label>
                <input type="text" id="patientName" placeholder="請輸入病患姓名">
            </div>

            <div class="form-group">
                <label for="medicineSelect">選擇藥物</label>
                <select id="medicineSelect">
                    <option value="">請選擇藥物</option>
                </select>
            </div>
            
            <div class="form-group">
                <label for="medicineQuantity">數量</label>
                <input type="number" id="medicineQuantity" min="1" value="1" placeholder="請輸入數量">
            </div>
            
            <button class="btn btn-success" onclick="addMedicineToList()">
                <span>➕</span> 加入藥物
            </button>

            <div id="selectedMedicines" class="medicine-selection hidden">
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

        async function loadMedicineOptions() {
            try {
                const response = await fetch('/api/medicine/basic');
                availableMedicines = await response.json();
                
                const select = document.getElementById('medicineSelect');
                select.innerHTML = '<option value="">請選擇藥物</option>';
                
                availableMedicines.forEach(med => {
                    const option = document.createElement('option');
                    option.value = med.id;
                    option.textContent = `${med.name} (位置: ${med.position}, 庫存: ${med.amount})`;
                    select.appendChild(option);
                });
                
            } catch (error) {
                showAlert('載入藥物選項失敗: ' + error.message, 'error');
            }
        }

        function addMedicineToList() {
            const medicineId = document.getElementById('medicineSelect').value;
            const quantity = parseInt(document.getElementById('medicineQuantity').value);
            
            if (!medicineId) {
                showAlert('請選擇藥物', 'error');
                return;
            }
            
            if (!quantity || quantity < 1) {
                showAlert('請輸入有效數量', 'error');
                return;
            }
            
            const medicine = availableMedicines.find(m => m.id == medicineId);
            if (!medicine) {
                showAlert('找不到選擇的藥物', 'error');
                return;
            }
            
            if (quantity > medicine.amount) {
                showAlert(`庫存不足，最多只能選擇 ${medicine.amount} 個`, 'error');
                return;
            }
            
            // Check if medicine already selected
            const existingIndex = selectedMedicines.findIndex(m => m.medicine_id == medicineId);
            if (existingIndex >= 0) {
                selectedMedicines[existingIndex].amount = quantity;
            } else {
                selectedMedicines.push({
                    medicine_id: parseInt(medicineId),
                    name: medicine.name,
                    position: medicine.position,
                    amount: quantity,
                    max_amount: medicine.amount
                });
            }
            
            updateSelectedMedicinesDisplay();
            
            // Reset form
            document.getElementById('medicineSelect').value = '';
            document.getElementById('medicineQuantity').value = '1';
        }

        function updateSelectedMedicinesDisplay() {
            const container = document.getElementById('selectedMedicines');
            const list = document.getElementById('selectedMedicinesList');
            
            if (selectedMedicines.length === 0) {
                container.classList.add('hidden');
                return;
            }
            
            container.classList.remove('hidden');
            
            list.innerHTML = selectedMedicines.map((med, index) => `
                <div class="medicine-item">
                    <div class="medicine-info">
                        <strong>${med.name}</strong><br>
                        <small>位置: ${med.position} | 數量: ${med.amount}</small>
                    </div>
                    <div class="medicine-controls">
                        <input type="number" class="quantity-input" value="${med.amount}" 
                               min="1" max="${med.max_amount}" 
                               onchange="updateMedicineQuantity(${index}, this.value)">
                        <button class="btn btn-danger" onclick="removeMedicine(${index})" style="padding: 8px 12px; margin: 0;">移除</button>
                    </div>
                </div>
            `).join('');
        }

        function updateMedicineQuantity(index, newQuantity) {
            const quantity = parseInt(newQuantity);
            if (quantity > 0 && quantity <= selectedMedicines[index].max_amount) {
                selectedMedicines[index].amount = quantity;
            } else {
                showAlert('數量無效', 'error');
                updateSelectedMedicinesDisplay();
            }
        }

        function removeMedicine(index) {
            selectedMedicines.splice(index, 1);
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
                    loadMedicineOptions(); // Refresh medicine options to update stock
                } else {
                    throw new Error('處方籤開立失敗');
                }
            } catch (error) {
                showAlert('錯誤: ' + error.message, 'error');
            }
        }

        function clearPrescriptionForm() {
            document.getElementById('patientName').value = '';
            document.getElementById('medicineSelect').value = '';
            document.getElementById('medicineQuantity').value = '1';
            selectedMedicines = [];
            updateSelectedMedicinesDisplay();
        }

        document.addEventListener('DOMContentLoaded', function() {
            loadMedicineOptions();
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
      left: 0; top: 0;
      width: 280px; height: 100vh;
      background: rgba(44, 62, 80, 0.95);
      backdrop-filter: blur(10px);
      z-index: 1000; padding: 20px;
      box-shadow: 4px 0 20px rgba(0,0,0,.3);
    }
    .sidebar .logo { text-align: center; margin-bottom: 40px; padding-bottom: 20px; border-bottom: 2px solid rgba(255,255,255,.1); }
    .sidebar .logo h2 { color:#ecf0f1; font-size:1.5em; font-weight:700; margin-bottom:5px; }
    .sidebar .logo p { color:#bdc3c7; font-size:.9em; }

    .nav-button {
      display:block; width:100%;
      padding:15px 20px; margin:10px 0;
      background:linear-gradient(45deg,#3498db,#2980b9);
      color:#fff; text-decoration:none; border:none; border-radius:12px;
      cursor:pointer; font-size:15px; font-weight:600; transition:.3s;
      text-align:left; display:flex; align-items:center; gap:12px;
    }
    .nav-button:hover { background:linear-gradient(45deg,#2980b9,#1f4e79); transform:translateX(8px); box-shadow:0 8px 25px rgba(52,152,219,.4); }
    .nav-button.active { background:linear-gradient(45deg,#e74c3c,#c0392b); transform:translateX(8px); box-shadow:0 8px 25px rgba(231,76,60,.4); }
    .nav-button .icon { font-size:18px; width:24px; text-align:center; }

    .main-content { margin-left:280px; padding:30px; min-height:100vh; }
    .page-header {
      background:#fff; padding:40px; border-radius:16px;
      box-shadow:0 8px 32px rgba(0,0,0,.1); margin-bottom:30px; text-align:center;
    }
    .page-header h1 { color:#2c3e50; font-size:2.8em; margin-bottom:10px; font-weight:700; }

    .content-card {
      background:#fff; padding:40px; border-radius:16px;
      box-shadow:0 8px 32px rgba(0,0,0,.1); margin-bottom:30px;
    }

    .stats-grid {
      display:grid; grid-template-columns:repeat(auto-fit,minmax(200px,1fr));
      gap:25px; margin-bottom:30px;
    }
    .stat-card { background:#fff; padding:30px; border-radius:12px; text-align:center; box-shadow:0 4px 20px rgba(0,0,0,.1); transition:.3s; }
    .stat-card:hover { transform:translateY(-5px); }
    .stat-number { font-size:2.5em; font-weight:bold; color:#667eea; margin-bottom:10px; }
    .stat-label { color:#666; font-size:1em; letter-spacing:1px; font-weight:600; text-transform:uppercase; }

    .btn {
      padding:15px 30px; border:none; border-radius:12px; cursor:pointer;
      font-size:15px; font-weight:600; transition:.3s; display:inline-flex; align-items:center; gap:10px; text-decoration:none; margin:8px;
    }
    .btn-success { background:linear-gradient(135deg,#28a745,#20c997); color:#fff; }
    .btn-success:hover { transform:translateY(-3px); box-shadow:0 12px 35px rgba(40,167,69,.4); }
    .btn-warning { background:linear-gradient(135deg,#ffc107,#ff8f00); color:#fff; }
    .btn-warning:hover { transform:translateY(-3px); box-shadow:0 12px 35px rgba(255,193,7,.4); }
    .btn-primary { background:linear-gradient(135deg,#667eea,#764ba2); color:#fff; }
    .btn-primary:hover { transform:translateY(-3px); box-shadow:0 12px 35px rgba(102,126,234,.4); }

    .data-table {
      width:100%; border-collapse:collapse; margin-top:25px;
      background:#fff; border-radius:12px; overflow:hidden; box-shadow:0 4px 20px rgba(0,0,0,.1);
    }
    .data-table th {
      background:linear-gradient(135deg,#667eea,#764ba2); color:#fff;
      padding:20px; text-align:left; font-weight:600; font-size:14px;
    }
    .data-table td { padding:20px; border-bottom:1px solid #e9ecef; color:#495057; font-size:14px; vertical-align:top; }
    .data-table tr:hover { background:#f8f9fa; transform:scale(1.01); transition:.2s; }

    .status-badge { padding:8px 16px; border-radius:20px; font-size:12px; font-weight:600; display:inline-block; text-transform:uppercase; }
    .status-pending { background:#fff3cd; color:#856404; }
    .status-processing { background:#d1ecf1; color:#0c5460; }
    .status-completed { background:#d4edda; color:#155724; }
    .status-failed { background:#f8d7da; color:#721c24; }

    .medicine-item {
      display:inline-block; background:#f8f9fa; padding:6px 12px; margin:2px;
      border-radius:6px; border:1px solid #e9ecef; font-size:12px;
    }

    .alert { padding:20px 25px; border-radius:12px; margin:20px 0; font-weight:500; border-left:4px solid; }
    .alert-success { background:#d4edda; color:#155724; border-color:#28a745; }
    .alert-error { background:#f8d7da; color:#721c24; border-color:#dc3545; }

    .loading { display:inline-block; width:25px; height:25px; border:3px solid #f3f3f3; border-top:3px solid #667eea; border-radius:50%; animation:spin 1s linear infinite; }
    @keyframes spin { 0%{transform:rotate(0)} 100%{transform:rotate(360deg)} }

    /* 已完成待領取—編號籤（有間隔橫向排列） */
    .chip-list { display:flex; flex-wrap:wrap; gap:12px; }
    .chip {
      background:#d4edda; color:#155724; border:1px solid #c3e6cb;
      padding:10px 14px; border-radius:999px; font-weight:700;
      display:inline-flex; align-items:center; gap:10px;
    }
    .chip button {
      border:none; border-radius:999px; padding:6px 10px; cursor:pointer;
      background:#28a745; color:#fff; font-weight:700;
    }
    .chip button:hover { filter:brightness(.95); }

    .text-center { text-align:center; }
    .muted { color:#6b7280; }
  </style>
</head>
<body>
  <div class="sidebar">
    <div class="logo">
      <h2>醫院管理系統</h2>
      <p>Hospital Management System</p>
    </div>

    <a href="/medicine.html" class="nav-button">
      <span class="icon">💊</span><span>藥物管理</span>
    </a>

    <a href="/doctor.html" class="nav-button">
      <span class="icon">👨‍⚕️</span><span>醫生工作台</span>
    </a>

    <a href="/prescription.html" class="nav-button active">
      <span class="icon">📋</span><span>處方籤管理</span>
    </a>

    <a href="/patient.html" class="nav-button">
      <span class="icon">🧑‍🦽</span><span>病患查詢</span>
    </a>
  </div>

  <div class="main-content">
    <div class="page-header">
      <h1>處方籤管理系統</h1>
      <p>監控和管理處方籤狀態</p>
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

    <!-- 已完成待領取（編號籤） -->
    <div class="content-card">
      <h2>已完成待領取</h2>
      <div id="pickupList" class="chip-list">
        <span class="muted">載入中…</span>
      </div>
    </div>

    <div class="content-card">
      <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:25px;">
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
      document.querySelectorAll('.alert').forEach(a => { if (!a.classList.contains('permanent')) a.remove(); });
      const el = document.createElement('div');
      el.className = `alert alert-${type === 'error' ? 'error' : 'success'}`;
      el.innerHTML = message;
      const container = document.querySelector('.content-card');
      if (container) {
        container.insertBefore(el, container.firstChild);
        setTimeout(() => el.remove(), 5000);
      }
    }

    function getStatusText(status) {
      const map = { pending:'待處理', processing:'處理中', completed:'已完成', failed:'失敗' };
      return map[status] || status;
    }

    function updateStats(list) {
      const stats = { pending:0, processing:0, completed:0, failed:0, total:list.length };
      list.forEach(p => { if (stats.hasOwnProperty(p.status)) stats[p.status]++; });
      document.getElementById('pendingCount').textContent = stats.pending;
      document.getElementById('processingCount').textContent = stats.processing;
      document.getElementById('completedCount').textContent = stats.completed;
      document.getElementById('totalCount').textContent = stats.total;
    }

    async function loadPrescriptions() {
      const tbody = document.getElementById('prescriptionList');
      const pickupBox = document.getElementById('pickupList');

      try {
        const res = await fetch('/api/prescription/');
        const prescriptions = await res.json();

        // 上方：已完成但未領取 → 顯示編號籤（有間隔）
        const waitingPick = prescriptions.filter(p => p.status === 'completed' && !p.picked_up);
        if (waitingPick.length === 0) {
          pickupBox.innerHTML = '<span class="muted">目前沒有待領取的處方籤</span>';
        } else {
          pickupBox.innerHTML = waitingPick.map(p => {
            const id = String(p.id).padStart(6,'0');
            return `<span class="chip">#${id}<button onclick="markPicked(${p.id})">已領藥</button></span>`;
          }).join('');
        }

        // 下方：列表
        updateStats(prescriptions);

        if (prescriptions.length === 0) {
          tbody.innerHTML = '<tr><td colspan="7" class="text-center">尚無處方籤資料</td></tr>';
          return;
        }

        tbody.innerHTML = prescriptions.map(p => {
          const createdDate = p.created_at ? new Date(p.created_at) : null;
          const updatedDate = p.updated_at ? new Date(p.updated_at) : null;
          const medicineList = (p.medicines || []).map(m =>
            `<span class="medicine-item">${m.name} (${m.amount}) [${m.position}]</span>`
          ).join(' ');

          let actions = '';
          if (p.status === 'pending') {
            actions = `<button class="btn btn-primary" onclick="updateStatus(${p.id}, 'processing')" style="padding:8px 16px; font-size:12px;">開始處理</button>`;
          } else if (p.status === 'processing') {
            actions = `<button class="btn btn-warning" onclick="updateStatus(${p.id}, 'completed')" style="padding:8px 16px; font-size:12px;">標記完成</button>`;
          } else if (p.status === 'completed' && !p.picked_up) {
            actions = `<button class="btn btn-success" onclick="markPicked(${p.id})" style="padding:8px 16px; font-size:12px;">已領藥</button>`;
          } else if (p.status === 'completed' && p.picked_up) {
            actions = `<span class="muted">已領取</span>`;
          }

          return `
            <tr>
              <td><strong>#${p.id.toString().padStart(6, '0')}</strong></td>
              <td>${p.patient_name}</td>
              <td>${createdDate ? createdDate.toLocaleString('zh-TW') : '-'}</td>
              <td>${updatedDate ? updatedDate.toLocaleString('zh-TW') : '-'}</td>
              <td><span class="status-badge status-${p.status}">${getStatusText(p.status)}</span></td>
              <td>${medicineList}</td>
              <td>${actions}</td>
            </tr>
          `;
        }).join('');

      } catch (error) {
        tbody.innerHTML = `<tr><td colspan="7" class="text-center"><div class="alert alert-error">載入失敗: ${error.message}</div></td></tr>`;
        document.getElementById('pickupList').innerHTML = `<span class="muted">載入失敗</span>`;
      }
    }

    async function updateStatus(id, status) {
      try {
        const r = await fetch(`/api/prescription/${id}/status`, {
          method: 'PUT',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ status })
        });
        if (!r.ok) throw new Error('狀態更新失敗');
        showAlert(`處方籤 #${String(id).padStart(6,'0')} 狀態已更新為：${getStatusText(status)}`, 'success');
        loadPrescriptions();
      } catch (e) {
        showAlert('錯誤：' + e.message, 'error');
      }
    }

    async function markPicked(id) {
      try {
        const r = await fetch(`/api/prescription/${id}/pickup`, { method:'POST' });
        if (!r.ok) {
          const e = await r.json().catch(() => ({ detail:'已領藥失敗' }));
          throw new Error(e.detail || '已領藥失敗');
        }
        showAlert(`處方籤 #${String(id).padStart(6,'0')} 已標記為「已領藥」`, 'success');
        loadPrescriptions();
      } catch (err) {
        showAlert('錯誤：' + err.message, 'error');
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

    document.addEventListener('DOMContentLoaded', function () {
      loadPrescriptions();
      startAutoRefresh();
    });
  </script>
</body>
</html>
"""



@app.get("/patient.html", response_class=HTMLResponse)
async def patient_page():
    return """<!DOCTYPE html>
<html lang="zh-TW">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>病患查詢 - 醫院管理系統</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family:'Microsoft JhengHei', Arial, sans-serif;
    background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
    background-attachment: fixed;
    color:#2c3e50; line-height:1.6; min-height:100vh;
    padding: 28px;
  }

  .page-header {
    background:#fff; padding:28px; border-radius:16px;
    box-shadow:0 8px 32px rgba(0,0,0,.08); margin-bottom:22px; text-align:center;
  }
  .page-header h1 { font-size:2.2rem; margin-bottom:6px; font-weight:800; }

  .two-col { display:grid; grid-template-columns: 1fr 1fr; gap:22px; }

  .panel {
    background:#fff; border-radius:16px; padding:20px;
    box-shadow:0 8px 24px rgba(0,0,0,.08); min-height: 60vh; display:flex; flex-direction:column;
  }
  .panel h2 {
    font-size:24px; margin:0 0 12px 0; font-weight:900; letter-spacing:.5px;
  }

  .section-head {
    background: linear-gradient(90deg,#6c63ff,#845ec2);
    color:#fff; border-radius:10px; padding:12px 16px; font-weight:800; margin-bottom:12px;
  }

  /* chip 列表（水平排列、自動換行、留白） */
  .chip-list { display:flex; flex-wrap:wrap; gap:12px; align-content:flex-start; }
  .chip {
    display:inline-flex; align-items:center; gap:10px;
    padding:10px 14px; border-radius:999px; font-weight:800;
    background:#eef2ff; color:#312e81; border:1px solid #c7d2fe;
    box-shadow:0 2px 8px rgba(0,0,0,.06);
  }
  .chip.processing { background:#e0f2fe; color:#0c4a6e; border-color:#bae6fd; }  /* 待處理/處理中 */
  .chip.completed  { background:#d1fae5; color:#065f46; border-color:#a7f3d0; }  /* 已完成(待領取) */

  .muted { color:#6b7280; }
  .loading {
    display:inline-block; width:18px; height:18px; border:3px solid #e5e7eb;
    border-top:3px solid #6c63ff; border-radius:50%; animation:spin 1s linear infinite;
    vertical-align:middle; margin-right:8px;
  }
  @keyframes spin { 0%{transform:rotate(0)} 100%{transform:rotate(360deg)} }
  /* 整體字級加大一階 */
    body { font-size: 30px; }

    /* 標題更大更醒目 */
    .page-header h1 { font-size: 2.8rem; }
    .page-header p { font-size: 1.1rem; }

    /* 兩欄容器：留更大間距 */
    .two-col { gap: 28px; }

    /* 面板：加高、內距加大、字體加大 */
    .panel {
    min-height: 70vh;      /* 原 60vh → 70vh */
    padding: 28px;         /* 原 20px → 28px */
    font-size: 1.05rem;
    }

    /* 區塊標頭更大 */
    .section-head {
    padding: 16px 20px;    /* 原 12/16 → 16/20 */
    font-size: 1.25rem;
    letter-spacing: .5px;
    }

    /* tag 列：間距放大 */
    .chip-list { gap: 16px; }

    /* 編號籤：尺寸、字重、圓角、陰影都放大 */
    .chip {
    padding: 12px 18px;    /* 原 10/14 → 12/18 */
    font-size: 1.1rem;     /* 原本約 16px → 17~18px */
    border-radius: 999px;
    font-weight: 900;
    box-shadow: 0 3px 10px rgba(0,0,0,.08);
    }

    /* 狀態配色維持，但讓對比更清楚 */
    .chip.processing {
    background:#dbeafe; color:#0c4a6e; border-color:#93c5fd;
    }
    .chip.completed {
    background:#bbf7d0; color:#065f46; border-color:#86efac;
    }

    /* 載入動畫也放大一點 */
    .loading { width: 22px; height: 22px; border-width: 3px; }

    /* 手機版：仍能塞下大籤，字再微縮避免換行太醜 */
    @media (max-width: 980px) {
    body { font-size: 17px; }
    .panel { min-height: auto; }
    .chip { font-size: 1.05rem; padding: 10px 16px; }
    }


  @media (max-width: 980px) { .two-col { grid-template-columns: 1fr; } }
</style>
</head>
<body>
  <div class="page-header">
    <h1>藥單處理狀況</h1>
    <p class="muted">左側顯示「待處理 + 處理中」，右側顯示「已完成（尚未領取）」</p>
  </div>

  <div class="two-col">
    <!-- 左：處理中（待處理 + 處理中） -->
    <section class="panel">
      <div class="section-head">處理中</div>
      <div id="listProcessing" class="chip-list">
        <span class="muted"><span class="loading"></span>載入中…</span>
      </div>
      <p class="muted" style="margin-top:10px;">包含「待處理」與「處理中」的訂單。</p>
    </section>

    <!-- 右：已完成（但尚未領取） -->
    <section class="panel">
      <div class="section-head">已完成</div>
      <div id="listCompleted" class="chip-list">
        <span class="muted"><span class="loading"></span>載入中…</span>
      </div>
    </section>
  </div>

<script>
let timer=null;

/** 從後端拉所有處方（含 picked_up） */
async function fetchAll(){
  const r = await fetch('/api/prescription/', {cache:'no-store', headers:{'cache-control':'no-cache','pragma':'no-cache'}});
  if(!r.ok) throw new Error('資料讀取失敗');
  return await r.json();
}

/** 產生 chip（#000123） */
function chip(id, cls){
  return `<span class="chip ${cls}">#${String(id).padStart(6,'0')}</span>`;
}

function render(processing, completed){
  const boxP = document.getElementById('listProcessing');
  const boxC = document.getElementById('listCompleted');

  if(processing.length === 0){
    boxP.innerHTML = '<span class="muted">目前沒有處理中的訂單</span>';
  }else{
    // 依訂單編號由小到大排列
    processing.sort((a,b)=>a.id-b.id);
    boxP.innerHTML = processing.map(p => chip(p.id, 'processing')).join('');
  }

  if(completed.length === 0){
    boxC.innerHTML = '<span class="muted">目前沒有已完成（待領取）的訂單</span>';
  }else{
    completed.sort((a,b)=>a.id-b.id);
    boxC.innerHTML = completed.map(p => chip(p.id, 'completed')).join('');
  }
}

/** 主流程：拉資料→分組→渲染 */
async function load(){
  try{
    const data = await fetchAll();
    const processing = [];  // 待處理 + 處理中
    const completed  = [];  // 已完成且未領取

    for(const p of data){
      if((p.status === 'pending') || (p.status === 'processing')){
        processing.push({id:p.id});
      }else if(p.status === 'completed' && !p.picked_up){
        completed.push({id:p.id});
      }
      // 其他狀態（failed 或 completed+picked_up）此頁隱藏
    }
    render(processing, completed);
  }catch(e){
    document.getElementById('listProcessing').innerHTML =
      `<span class="muted">載入失敗：${e.message}</span>`;
    document.getElementById('listCompleted').innerHTML =
      `<span class="muted">載入失敗：${e.message}</span>`;
  }
}

function start(){ stop(); timer=setInterval(load, 3000); }
function stop(){ if(timer){ clearInterval(timer); timer=null; } }

document.addEventListener('visibilitychange', ()=>{ if(document.visibilityState==='visible'){ load(); } });
document.addEventListener('DOMContentLoaded', ()=>{ load(); start(); });
</script>
</body>
</html>
"""



if __name__ == "__main__":
    print("Starting Clean Hospital Medicine Management System...")
    init_database()
    
    print("\n Clean Hospital System Ready!")
    print("=" * 50)
    print(" Web Interfaces:")
    print("  - Medicine Management: http://localhost:8001/medicine.html")
    print("  - Doctor Interface: http://localhost:8001/doctor.html") 
    print("  - Prescription Management: http://localhost:8001/prescription.html")
    print("  - Prescription Management: http://localhost:8001/patient.html")
    print("  - Fast API Doc: http://localhost:8001/docs ")
    print("\n ROS2 Integration Endpoints:")
    print("  - Get Next Order: GET /api/ros2/order/next")
    print("  - Complete Order: POST /api/ros2/order/complete")
    print("  - Progress Update: POST /api/ros2/order/progress")
    print("  - Medicine Basic Info: GET /api/ros2/medicine/basic/{name}")
    print("  - Medicine Detail Info: GET /api/ros2/medicine/detailed/{name}")
    print("\n Key Features:")
    print("  Position Format: 1-2, 2-1 (row-column)")
    print("  Doctor Dropdown Selection")
    print("  Basic + Detailed Info Required")
    print("  Separate Storage for Basic/Detailed")
    print("  Clean Database (No Test Data)")
    print("  Complete ROS2 Integration")
    
    uvicorn.run(app, host="0.0.0.0", port=8001) 