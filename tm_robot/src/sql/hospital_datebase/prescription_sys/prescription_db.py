from sqlmodel import Session, SQLModel, create_engine

# 共用同一個 hospital.db
engine = create_engine("sqlite:///prescription.db", echo=True)
SQLModel.metadata.create_all(engine)

def init_db():
    SQLModel.metadata.create_all(engine)
    
def get_session():
    return Session(engine)