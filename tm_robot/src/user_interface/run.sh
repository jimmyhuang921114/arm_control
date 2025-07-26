#!/bin/bash

echo "建立資料庫..."
python3 init_db.py

echo "啟動 FastAPI..."
echo "開啟 API 文件: http://localhost:8000/docs"

# uvicorn main:app --host 0.0.0.0 --port 8000 --reload
/home/work/.local/bin/uvicorn main:app --reload --host 0.0.0.0 --port 8000
