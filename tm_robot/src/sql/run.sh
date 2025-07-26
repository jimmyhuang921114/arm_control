#!/bin/bash

echo "📦 建立資料庫..."
python3 init_db.py

echo "🚀 啟動 FastAPI..."
echo "🔗 開啟 API 文件: http://localhost:8000/docs"

# 啟動 FastAPI，並將輸出導向背景（如不想要背景可移除最後的 &）
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
