from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from hospital_datebase.database import get_db  
from hospital_datebase.med_sys import medicine_models, crud_medicine

router = APIRouter()

@router.post("/medicine/")
def add_medicine(medicine: medicine_models.Medicine, db: Session = Depends(get_db)):
    return crud_medicine.create_medicine(db, medicine)

@router.get("/medicine/")
def list_all(db: Session = Depends(get_db)):
    return crud_medicine.get_all_medicines(db)

@router.get("/medicine/{name}")
def get_by_name(name: str, db: Session = Depends(get_db)):
    return crud_medicine.get_medicine_by_name(db, name)

@router.delete("/medicine/{id}")
def delete_by_id(id: int, db: Session = Depends(get_db)):
    success = crud_medicine.delete_medicine_by_id(db, id)
    return {"success": success}
