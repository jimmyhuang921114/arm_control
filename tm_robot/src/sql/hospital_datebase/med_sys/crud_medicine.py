# hospital_datebase/med_sys/crud_medicine.py

from sqlalchemy.orm import Session
from hospital_datebase.med_sys import medicine_models


def create_medicine(db: Session, medicine: medicine_models.Medicine):
    db.add(medicine)
    db.commit()
    db.refresh(medicine)
    return medicine



def get_medicine_by_name(db: Session, name: str):
    return db.query(medicine_models.Medicine).filter(medicine_models.Medicine.name == name).first()


def get_all_medicines(db: Session):
    return db.query(medicine_models.Medicine).all()


def delete_medicine_by_id(db: Session, medicine_id: int):
    medicine = db.query(medicine_models.Medicine).filter(medicine_models.Medicine.id == medicine_id).first()
    if medicine:
        db.delete(medicine)
        db.commit()
        return True
    return False
