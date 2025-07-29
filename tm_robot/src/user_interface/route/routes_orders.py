from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
from database.pharmacy_db import PharmacyService
import requests
import json

router = APIRouter()

class CreateOrderRequest(BaseModel):
    patient_id: str
    medicines: List[dict]  # [{"medicine_name": "paracetamol", "quantity": 2}]
    doctor_name: Optional[str] = None
    department: Optional[str] = None
    notes: Optional[str] = None

class OrderResponse(BaseModel):
    order_id: str
    patient_id: str
    patient_name: str
    status: str  # pending 或 completed
    medicines: List[dict]
    created_at: str

@router.post("/orders", response_model=dict)
async def create_order(request: CreateOrderRequest):
    """創建新訂單"""
    try:
        order_id = PharmacyService.create_order(
            patient_id=request.patient_id,
            medicines=request.medicines,
            doctor_name=request.doctor_name,
            department=request.department,
            notes=request.notes
        )
        
        # 發送訂單到機器人系統
        await send_order_to_robot(order_id)
        
        return {
            "success": True,
            "order_id": order_id,
            "message": "訂單創建成功，已發送至機器人系統"
        }
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

@router.get("/orders/{order_id}", response_model=OrderResponse)
async def get_order(order_id: str):
    """獲取訂單詳情"""
    order_data = PharmacyService.get_order_with_items_and_configs(order_id)
    if not order_data:
        raise HTTPException(status_code=404, detail="訂單不存在")
    
    return OrderResponse(**order_data)

@router.get("/orders", response_model=List[OrderResponse])
async def get_orders(status: Optional[str] = None):
    """獲取訂單列表"""
    if status == "pending":
        orders = PharmacyService.get_pending_orders()
        return [
            OrderResponse(**PharmacyService.get_order_with_items_and_configs(order.order_id))
            for order in orders
        ]
    else:
        # 獲取所有訂單的邏輯可以後續擴展
        orders = PharmacyService.get_pending_orders()
        return [
            OrderResponse(**PharmacyService.get_order_with_items_and_configs(order.order_id))
            for order in orders
        ]

@router.get("/patients/search")
async def search_patients(keyword: str):
    """搜尋病人"""
    patients = PharmacyService.search_patients(keyword)
    return [
        {
            "patient_id": p.patient_id,
            "name": p.name,
            "age": p.age,
            "medical_record": p.medical_record
        }
        for p in patients
    ]

@router.get("/medicines")
async def get_medicines():
    """獲取所有藥物"""
    medicines = PharmacyService.get_all_medicines()
    return [
        {
            "medicine_code": m.medicine_code,
            "name": m.name,
            "description": m.description,
            "unit": m.unit
        }
        for m in medicines
    ]

@router.put("/orders/{order_id}/complete")
async def complete_order(order_id: str):
    """標記訂單為完成"""
    success = PharmacyService.update_order_status(order_id, "completed")
    if success:
        return {"success": True, "message": "訂單已完成"}
    else:
        raise HTTPException(status_code=404, detail="訂單不存在")

@router.get("/orders/robot/pending")
async def get_pending_orders_for_robot():
    """供機器人系統獲取待處理訂單"""
    orders = PharmacyService.get_pending_orders()
    robot_orders = []
    
    for order in orders:
        order_data = PharmacyService.get_order_with_items_and_configs(order.order_id)
        
        # 轉換為機器人系統需要的格式
        robot_order = {
            "order_id": order_data["order_id"],
            "patient_id": order_data["patient_id"],
            "medicines": []
        }
        
        for medicine in order_data["medicines"]:
            robot_medicine = {
                "name": medicine["name"],
                "quantity": medicine["quantity"],
                "json_config": medicine["json_config"]  # 這裡包含了預先儲存的JSON配置
            }
            robot_order["medicines"].append(robot_medicine)
        
        robot_orders.append(robot_order)
    
    return robot_orders

async def send_order_to_robot(order_id: str):
    """發送訂單到機器人系統"""
    try:
        # 獲取完整訂單信息
        order_data = PharmacyService.get_order_with_items_and_configs(order_id)
        if not order_data:
            return False
        
        # 轉換為機器人系統格式
        robot_order = {
            "order_id": order_data["order_id"],
            "patient_id": order_data["patient_id"],
            "medicines": []
        }
        
        for medicine in order_data["medicines"]:
            robot_medicine = {
                "name": medicine["name"],
                "quantity": medicine["quantity"],
                "json_config": medicine["json_config"]  # 從資料庫提取的JSON配置
            }
            robot_order["medicines"].append(robot_medicine)
        
        # 這裡實際上會透過ROS發送訂單，暫時用HTTP模擬
        print(f"發送訂單到機器人系統: {json.dumps(robot_order, ensure_ascii=False, indent=2)}")
        
        return True
        
    except Exception as e:
        print(f"發送訂單到機器人系統失敗: {e}")
        return False