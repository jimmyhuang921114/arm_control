from sqlalchemy import create_engine
from models import Base
from sqlmodel import SQLModel


from hospital_datebase.med_sys.medicine_models import Medicine
from hospital_datebase.prescription_sys.prescription_models import Prescription, Patient

engine = create_engine("sqlite:///database.db", echo=True)

def init_db():
    with engine.connect() as conn:
        result = conn.execute("SELECT name FROM sqlite_master WHERE type='table';")
        tables = result.fetchall()
        if not tables:
            SQLModel.metadata.create_all(engine)
            print("資料庫初始化完成")
        else:
            print("資料庫已存在，略過初始化")
