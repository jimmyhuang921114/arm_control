from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from pathlib import Path

# 僅保留 routes
from hospital_datebase.med_sys.routes_medicine import router as medicine_routes
from hospital_datebase.prescription_sys.routes_prescription import router as prescription_routes

# 初始化資料庫
from hospital_datebase.med_sys.medicine_db import init_db as init_medicine_db
from hospital_datebase.prescription_sys.prescription_db import init_db as init_prescription_db

# 註解掉 views，避免重複路由
# from hospital_datebase.med_sys.views import router as medicine_router
# from hospital_datebase.prescription_sys.views import router as prescription_router

# 初始化 FastAPI 應用，使用 lifespan 執行資料庫初始化
@asynccontextmanager
async def lifespan(app: FastAPI):
    init_medicine_db()
    init_prescription_db()
    yield

app = FastAPI(title="Hospital System API", lifespan=lifespan)

# 設定靜態與模板路徑
base_dir = Path(__file__).resolve().parent
template_static_dir = str(base_dir)

# 掛載靜態檔案（JS、CSS）
app.mount("/sql", StaticFiles(directory=template_static_dir), name="sql")

# 掛載模板目錄
templates = Jinja2Templates(directory=template_static_dir)

# 掛載 API 路由（routes 版本）
app.include_router(medicine_routes, prefix="/api", tags=["Medicine API"])
app.include_router(prescription_routes, prefix="/api", tags=["Prescription API"])

# 首頁（渲染 api.html 頁面）
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("api.html", {"request": request})
