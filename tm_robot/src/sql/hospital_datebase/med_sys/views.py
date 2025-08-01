# hospital_datebase/med_sys/views.py

from fastapi import APIRouter, Depends
from sqlmodel import Session, select
from hospital_datebase.med_sys.medicine_models import Medicine
from .medicine_db import get_session

router = APIRouter()

@router.get("/medicines/", response_model=list[Medicine])
def read_medicines(session: Session = Depends(get_session)):
    return session.exec(select(Medicine)).all()

@router.post("/medicines/", response_model=Medicine)
def create_medicine(medicine: Medicine, session: Session = Depends(get_session)):
    session.add(medicine)
    session.commit()
    session.refresh(medicine)
    return medicine
