# hospital_datebase/med_sys/crud_medicine.py

from sqlalchemy.orm import Session
from model import medicine_models

#create medicine
def create_medicine(db: Session, medicine: medicine_models.Medicine):
    db.add(medicine)
    db.commit()
    db.refresh(medicine)
    return medicine


#get the medicine by name
def get_medicine_by_name(db: Session, name: str):
    return db.query(medicine_models.Medicine).filter(medicine_models.Medicine.name == name).first()

#get all the medicine in medicine databae
def get_all_medicines(db: Session):
    return db.query(medicine_models.Medicine).all()

#delete the medicine by name
def delete_medicine_by_name(db: Session, name: str):
    medicine = db.query(medicine_models.Medicine).filter(medicine_models.Medicine.id == name).first()
    if medicine:
        db.delete(medicine)
        db.commit()
        return True
    return False
