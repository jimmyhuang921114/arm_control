from typing import List #user for type hinting
from fastapi import APIRouter, Depends #for creating API routes and dependency injection
from sqlmodel import Session, select #for database operations and queries
from hospital_datebase.prescription_sys.prescription_models import Prescription # import the Medicine model
from .prescription_models import Prescription # import the Prescription model
from .prescription_db import get_session # import the session dependency

router = APIRouter()

@router.get("/prescriptions/", response_model=List[Prescription])
def get_all_prescriptions(session: Session = Depends(get_session)):
    return session.exec(select(Prescription)).all()

@router.post("/prescriptions/", response_model=Prescription)
def create_prescription(prescription: Prescription, session: Session = Depends(get_session)):
    session.add(prescription)
    session.commit()
    session.refresh(prescription)
    return prescription
