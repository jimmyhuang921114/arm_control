import sqlite3
import yaml

DB_PATH = "/workspace/tm_robot/src/tm_robot_main/tm_robot_main/medicine_info.db"

def insert_data(name,amount,locate,prompt,confident):
    conn = sqlite3.connect(DB_PATH)
    curosr = conn.cursor()

