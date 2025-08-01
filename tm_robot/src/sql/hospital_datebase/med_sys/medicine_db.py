from sqlmodel import Session, SQLModel, create_engine

DATABASE_URL = "sqlite:///./medicine.db"  # 或你的 MySQL URL

engine = create_engine(DATABASE_URL, echo=True)

def init_db():
    SQLModel.metadata.create_all(engine)

def get_session():
    return Session(engine)
