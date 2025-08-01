from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from hospital_datebase.database import get_db
from hospital_datebase.prescription_sys import prescription_models, crud_prescription

router = APIRouter()

# 新增處方
@router.post("/prescription/")
def add_prescription(prescription: prescription_models.Prescription, db: Session = Depends(get_db)):
    return crud_prescription.create_prescription(db, prescription)

# 取得所有處方
@router.get("/prescription/")
def list_all_prescriptions(db: Session = Depends(get_db)):
    return crud_prescription.get_all_prescriptions(db)

# 根據病患姓名查找處方（修正這個 API）
@router.get("/prescription/by_patient_name/{name}")
def get_by_patient_name(name: str, db: Session = Depends(get_db)):
    return crud_prescription.get_prescription_by_patient_name(db, name)

# 根據病患 ID 查找處方（修正這個 API）
@router.get("/prescription/by_patient_id/{patient_id}")
def get_by_patient_id(patient_id: int, db: Session = Depends(get_db)):
    return crud_prescription.get_prescriptions_by_patient_id(db, patient_id)

# 根據 ID 刪除處方
@router.delete("/prescription/{id}")
def delete_by_id(id: int, db: Session = Depends(get_db)):
    success = crud_prescription.delete_prescription_by_id(db, id)
    return {"success": success}
