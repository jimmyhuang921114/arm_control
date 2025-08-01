# hospital_datebase/prescription_sys/crud_prescription.py

from sqlalchemy.orm import Session
from model import prescription_models

from datetime import datetime

def create_patient_with_prescriptions(db: Session, data):
    # create patient
    new_patient = prescription_models.Patient(
        name=data.name,
        sex=data.sex,
        age=data.age,
        status_grab=(data.picked_state != "尚未開始"),
        status_picked=(data.collected_state != "尚未開始"),
        create_time=datetime.utcnow()
    )
    db.add(new_patient)
    db.commit()
    db.refresh(new_patient)

    # create many prescriptions
    for p in data.prescriptions:
        new_prescription = prescription_models.Prescription(
            patient_id=new_patient.medicine_id, 
            medicine_name=p.medicine_name,
            medicine_dosage=p.medicine_dosage,
            medicine_frequency=p.medicine_frequency,
            medicine_type=p.medicine_type,
            thing_of_note=p.thing_of_note
        )
        db.add(new_prescription)

    db.commit()
    return {"success": True, "patient_id": new_patient.medicine_id}




#search prescription by name
def get_prescription_by_name(db: Session, name: str):
    return db.query(prescription_models.Prescription).filter(prescription_models.Prescription.name == name).first()


#search all prescriptions by patient id
def get_prescriptions_by_patient_id(db: Session, patient_id: int):
    return db.query(prescription_models.Prescription).filter(prescription_models.Prescription.patient_id == patient_id).all()


#get the all prescription
def get_all_prescriptions(db: Session):
    return db.query(prescription_models.Prescription).all()


#delete prescription by 
# def delete_prescription(db: Session, prescription: prescription_models.Prescription):
#     db.delete(prescription)
#     db.commit()
#     return prescription

#delete prescription by id number
def delete_prescription_by_id(db: Session, prescription_id: int):
    prescription = db.query(prescription_models.Prescription).filter(prescription_models.Prescription.id == prescription_id).first()
    if prescription:
        db.delete(prescription)
        db.commit()
        return True
    return False
