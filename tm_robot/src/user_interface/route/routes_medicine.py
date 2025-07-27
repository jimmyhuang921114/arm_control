from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from database.database import get_db  
from model import medicine_models
from services import crud_medicine
from schemas.medicine_schema import MedicineCreate  # 如果有 schema 分層

router = APIRouter()

# add medicine
@router.post("/medicine/")
def add_medicine(medicine: MedicineCreate, db: Session = Depends(get_db)):
    return crud_medicine.create_medicine(db, medicine)

# get the medicine from medicine database
@router.get("/medicine/")
def list_all(db: Session = Depends(get_db)):
    return crud_medicine.get_all_medicines(db)

# get the medicine by name for medicine database
@router.get("/medicine/{name}")
def get_by_name(name: str, db: Session = Depends(get_db)):
    return crud_medicine.get_medicine_by_name(db, name)

# delete the medicine by name
@router.delete("/medicine/{name}")
def delete_by_id(name: str, db: Session = Depends(get_db)):
    success = crud_medicine.delete_medicine_by_name(db, name)
    return {"success": success}
