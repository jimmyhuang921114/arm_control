from sqlmodel import SQLModel
from typing import Optional
from datetime import datetime


# 用於接收前端新增藥物資料（POST body）
class MedicineCreate(SQLModel):
    name: str
    amount: int
    usage_days: int
    position: str


# 用於查詢/顯示藥物資料（GET 回傳）
class MedicineRead(SQLModel):
    id: int
    name: str
    amount: int
    usage_days: int
    position: str
    create_time: datetime


# 可選：若你要支援藥物更新（PATCH / PUT）
class MedicineUpdate(SQLModel):
    name: Optional[str] = None
    amount: Optional[int] = None
    usage_days: Optional[int] = None
    position: Optional[str] = None
