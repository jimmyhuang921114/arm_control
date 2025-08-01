#patient prescription models

from typing import List, Optional
from sqlmodel import Field , SQLModel,Relationship,Column
from datetime import datetime


#sturcture of the database model
# one patient can have multiple prescriptions

# Patient model
class Patient(SQLModel, table=True):
    medicine_id: Optional[int] = Field(default=None,primary_key=True) #primary key
    name :str 
    sex : str
    age : int
    create_time : datetime = Field(default_factory=datetime.utcnow) #default value is current time
    status_grab: bool = Field(default=False)
    status_picked: bool = Field(default=False)
    prescriptions: List["Prescription"] = Relationship(back_populates="patient")  # Relationship to Prescription model
    
# Prescription model
class Prescription(SQLModel,table=True):
    id: Optional[int] = Field(default=None, primary_key=True)  # primary key
    patient_id: int = Field(foreign_key="patient.id")  # foreign key, linking to Patient model
    medicine_name : str
    medicine_dosage : str
    medicine_frequency : str
    medicine_type : str 
    thing_of_note : str
    
    create_time: datetime = Field(default_factory=datetime.utcnow)  # default value is current time

    patient: Optional[Patient] = Relationship(back_populates="prescriptions")  # Relationship to Patient model
