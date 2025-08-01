# prescription_schemas.py
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class PrescriptionCreate(BaseModel):
    medicine_name: str
    medicine_dosage: str
    medicine_frequency: str
    medicine_type: str
    thing_of_note: str

class PrescriptionRead(PrescriptionCreate):
    id: int
    patient_id: int
    create_time: datetime

class PatientCreate(BaseModel):
    name: str
    id: str  # 身分證字號
    sex: str
    age: int
    created_at: str
    picked_state: str
    collected_state: str
    prescriptions: List[PrescriptionCreate]

class PatientRead(BaseModel):
    id: int
    name: str
    sex: str
    age: int
    create_time: datetime
    status_grab: bool
    status_picked: bool
    prescriptions: List[PrescriptionRead] = []
