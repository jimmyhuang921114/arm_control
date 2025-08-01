from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, Session

# set the database url 
DATABASE_URL = "sqlite:///./hospital.db"

#  
engine = create_engine(
    DATABASE_URL, connect_args={"check_same_thread": False} 
)

# create SessionLocal factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 提供 get_db() 給 Depends() 使用
def get_db() -> Session:
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
