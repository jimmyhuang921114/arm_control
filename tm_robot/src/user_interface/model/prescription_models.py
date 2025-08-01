from typing import List, Optional
from sqlmodel import Field, SQLModel, Relationship
from datetime import datetime


# Patient model
class Patient(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True) 
    name: str
    sex: str
    age: int
    create_time: datetime = Field(default_factory=datetime.utcnow)
    status_grab: bool = Field(default=False)
    status_picked: bool = Field(default=False)

    prescriptions: List["Prescription"] = Relationship(back_populates="patient")


# Prescription model
class Prescription(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    patient_id: int = Field(foreign_key="patient.id")
    medicine_name: str
    medicine_dosage: str
    medicine_frequency: str
    medicine_type: str
    thing_of_note: str
    create_time: datetime = Field(default_factory=datetime.utcnow)

    patient: Optional[Patient] = Relationship(back_populates="prescriptions")
