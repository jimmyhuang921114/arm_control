# hospital_datebase/prescription_sys/crud_prescription.py

from sqlalchemy.orm import Session
from hospital_datebase.prescription_sys import prescription_models

#create prescription
def create_prescription(db: Session, prescription: prescription_models.Prescription):
    db.add(prescription)
    db.commit()
    db.refresh(prescription)
    return prescription



#search prescription by name
def get_prescription_by_name(db: Session, name: str):
    return db.query(prescription_models.Prescription).filter(prescription_models.Prescription.name == name).first()


#search all prescriptions by patient id
def get_prescriptions_by_patient_id(db: Session, patient_id: int):
    return db.query(prescription_models.Prescription).filter(prescription_models.Prescription.patient_id == patient_id).all()

def get_all_prescriptions(db: Session):
    return db.query(prescription_models.Prescription).all()

def delete_prescription(db: Session, prescription: prescription_models.Prescription):
    db.delete(prescription)
    db.commit()
    return prescription


def delete_prescription_by_id(db: Session, prescription_id: int):
    prescription = db.query(prescription_models.Prescription).filter(prescription_models.Prescription.id == prescription_id).first()
    if prescription:
        db.delete(prescription)
        db.commit()
        return True
    return False
