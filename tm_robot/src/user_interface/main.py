from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from contextlib import asynccontextmanager
from pathlib import Path

# API 路由匯入
from route.routes_medicine import router as medicine_routes
from route.routes_prescription import router as prescription_routes
from route.routes_orders import router as order_routes

# 初始化資料庫
from database.medicine_db import init_db as init_medicine_db
from database.prescription_db import init_db as init_prescription_db
from database.pharmacy_db import init_pharmacy_db

@asynccontextmanager
async def lifespan(app: FastAPI):
    # 初始化所有資料庫
    init_medicine_db()
    init_prescription_db()
    init_pharmacy_db()
    yield

app = FastAPI(title="Pharmacy System API", lifespan=lifespan)

# 設定資料夾路徑
base_dir = Path(__file__).resolve().parent
static_dir = base_dir / "static"
html_dir = static_dir / "html"
css_dir = static_dir / "css"
js_dir = static_dir / "js"

# 掛載靜態資源
app.mount("/css", StaticFiles(directory=css_dir), name="css")
app.mount("/js", StaticFiles(directory=js_dir), name="js")

# Jinja2 Templates 設定
templates = Jinja2Templates(directory=str(html_dir))

# 掛載 API 路由
app.include_router(medicine_routes, prefix="/api", tags=["Medicine API"])
app.include_router(prescription_routes, prefix="/api", tags=["Prescription API"])
app.include_router(order_routes, prefix="/api", tags=["Order API"])

# 首頁與頁面路由
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    return templates.TemplateResponse("orders.html", {"request": request})

@app.get("/Medicine.html", response_class=HTMLResponse)
async def render_medicine(request: Request):
    return templates.TemplateResponse("Medicine.html", {"request": request})

@app.get("/Prescription.html", response_class=HTMLResponse)
async def render_prescription(request: Request):
    return templates.TemplateResponse("Prescription.html", {"request": request})

@app.get("/doctor.html", response_class=HTMLResponse)
async def render_doctor(request: Request):
    return templates.TemplateResponse("doctor.html", {"request": request})

@app.get("/background.html", response_class=HTMLResponse)
async def render_background(request: Request):
    return templates.TemplateResponse("background.html", {"request": request})

@app.get("/orders.html", response_class=HTMLResponse)
async def render_orders(request: Request):
    return templates.TemplateResponse("orders.html", {"request": request})
