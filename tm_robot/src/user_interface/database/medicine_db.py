from sqlmodel import Session, SQLModel, create_engine

#set database url
DATABASE_URL = "sqlite:///./hospital.db" 

#create database 
engine = create_engine(DATABASE_URL, echo=True)

#medicine and prescription database in same website
def init_db():
    SQLModel.metadata.create_all(engine) 

def get_session():
    return Session(engine)
