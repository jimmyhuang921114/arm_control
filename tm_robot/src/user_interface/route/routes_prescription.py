from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session
from database.database import get_db
from model import prescription_models
from services import crud_prescription
from schemas.prescription_schemas import PatientCreate, PrescriptionCreate, PatientRead, PrescriptionRead
from typing import List

router = APIRouter()

# add prescription
@router.post("/prescription/", response_model=PrescriptionRead)
def add_prescription(prescription: PrescriptionCreate, db: Session = Depends(get_db)):
    return crud_prescription.create_prescription(db, prescription)

# get all prescription for database
@router.get("/prescription/", response_model=List[PrescriptionRead])
def list_all_prescriptions(db: Session = Depends(get_db)):
    return crud_prescription.get_all_prescriptions(db)

# get prescription by name from database
@router.get("/prescription/by_patient_name/{name}", response_model=List[PrescriptionRead])
def get_by_patient_name(name: str, db: Session = Depends(get_db)):
    return crud_prescription.get_prescription_by_patient_name(db, name)

# get prescription by identify card from database
@router.get("/prescription/by_patient_id/{patient_id}", response_model=List[PrescriptionRead])
def get_by_patient_id(patient_id: int, db: Session = Depends(get_db)):
    return crud_prescription.get_prescriptions_by_patient_id(db, patient_id)

# delete prescription by identify number from database
@router.delete("/prescription/{id}")
def delete_by_id(id: int, db: Session = Depends(get_db)):
    success = crud_prescription.delete_prescription_by_id(db, id)
    return {"success": success}


# @router.post("/prescription/register", response_model=PatientRead)
# def register_patient_and_prescriptions(data: PatientCreate, db: Session = Depends(get_db)):
#     return crud_prescription.create_patient_with_prescriptions(db, data)


# @router.get("/patients_with_prescriptions", response_model=List[PatientRead])
# def get_patients_with_prescriptions(db: Session = Depends(get_db)):
#     return crud_prescription.get_patients_with_prescriptions(db)