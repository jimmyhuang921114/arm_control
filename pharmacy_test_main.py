from fastapi import FastAPI, Request, HTTPException
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from contextlib import asynccontextmanager
from pathlib import Path
from pydantic import BaseModel
from typing import List, Optional
import sqlite3
import json
import uuid
from datetime import datetime

# 模擬資料庫初始化
def init_test_db():
    """初始化測試資料庫"""
    conn = sqlite3.connect('pharmacy_test.db')
    cursor = conn.cursor()
    
    # 創建表格
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS patients (
            patient_id TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            age INTEGER,
            medical_record TEXT
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS medicines (
            medicine_code TEXT PRIMARY KEY,
            name TEXT NOT NULL,
            description TEXT,
            unit TEXT DEFAULT '顆'
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS medicine_json_configs (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            medicine_code TEXT,
            config_name TEXT,
            json_data TEXT,
            is_default BOOLEAN DEFAULT 1,
            FOREIGN KEY (medicine_code) REFERENCES medicines (medicine_code)
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            order_id TEXT PRIMARY KEY,
            patient_id TEXT,
            status TEXT DEFAULT 'pending',
            doctor_name TEXT,
            department TEXT,
            notes TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (patient_id) REFERENCES patients (patient_id)
        )
    ''')
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS order_items (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            order_id TEXT,
            medicine_code TEXT,
            quantity INTEGER,
            json_config_id INTEGER,
            FOREIGN KEY (order_id) REFERENCES orders (order_id),
            FOREIGN KEY (medicine_code) REFERENCES medicines (medicine_code),
            FOREIGN KEY (json_config_id) REFERENCES medicine_json_configs (id)
        )
    ''')
    
    # 插入測試資料
    test_patients = [
        ('P001', '王小明', 35, '高血壓患者'),
        ('P002', '李小華', 28, '感冒症狀'),
        ('P003', '張大同', 45, '糖尿病患者'),
        ('P004', '陳美麗', 52, '關節炎患者'),
        ('P005', '林志偉', 33, '過敏性鼻炎')
    ]
    
    test_medicines = [
        ('M001', 'Paracetamol', '普拿疼 500mg', '顆'),
        ('M002', 'Aspirin', '阿司匹林 100mg', '顆'),
        ('M003', 'Ibuprofen', '布洛芬 200mg', '顆'),
        ('M004', 'Metformin', '二甲雙胍 500mg', '顆'),
        ('M005', 'Loratadine', '氯雷他定 10mg', '顆')
    ]
    
    test_configs = [
        ('M001', 'default', json.dumps({
            "detection": {"confidence_threshold": 0.4, "size_threshold": 0.05},
            "grasp": {"strategy": "suction", "suction_offset": 0.003},
            "verification": {"llm_prompt": "識別白色圓形普拿疼藥片", "required_confidence": 0.8}
        }), True),
        ('M002', 'default', json.dumps({
            "detection": {"confidence_threshold": 0.35, "size_threshold": 0.04},
            "grasp": {"strategy": "pinch", "approach_angle": 90},
            "verification": {"llm_prompt": "識別白色小圓阿司匹林藥片", "required_confidence": 0.85}
        }), True),
        ('M003', 'default', json.dumps({
            "detection": {"confidence_threshold": 0.4, "size_threshold": 0.06},
            "grasp": {"strategy": "suction", "suction_offset": 0.002},
            "verification": {"llm_prompt": "識別橘色橢圓形布洛芬藥片", "required_confidence": 0.8}
        }), True)
    ]
    
    # 清空並插入資料
    cursor.execute('DELETE FROM order_items')
    cursor.execute('DELETE FROM orders')
    cursor.execute('DELETE FROM medicine_json_configs')
    cursor.execute('DELETE FROM medicines')
    cursor.execute('DELETE FROM patients')
    
    cursor.executemany('INSERT INTO patients VALUES (?, ?, ?, ?)', test_patients)
    cursor.executemany('INSERT INTO medicines VALUES (?, ?, ?, ?)', test_medicines)
    cursor.executemany('INSERT INTO medicine_json_configs (medicine_code, config_name, json_data, is_default) VALUES (?, ?, ?, ?)', test_configs)
    
    conn.commit()
    conn.close()

# 初始化資料庫
init_test_db()

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("🚀 藥局測試系統啟動中...")
    yield
    print("🔚 藥局測試系統關閉")

app = FastAPI(title="藥局測試系統", lifespan=lifespan)

# Pydantic 模型
class CreateOrderRequest(BaseModel):
    patient_id: str
    medicines: List[dict]  # [{"medicine_name": "Paracetamol", "quantity": 2}]
    doctor_name: Optional[str] = None
    department: Optional[str] = None
    notes: Optional[str] = None

# 資料庫輔助函數
def get_db_connection():
    return sqlite3.connect('pharmacy_test.db')

# API 路由
@app.get("/api/patients/search")
async def search_patients(keyword: str):
    """搜尋病人"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        SELECT patient_id, name, age, medical_record 
        FROM patients 
        WHERE name LIKE ? OR patient_id LIKE ? OR medical_record LIKE ?
        LIMIT 10
    ''', (f'%{keyword}%', f'%{keyword}%', f'%{keyword}%'))
    
    results = []
    for row in cursor.fetchall():
        results.append({
            'patient_id': row[0],
            'name': row[1],
            'age': row[2],
            'medical_record': row[3]
        })
    
    conn.close()
    return results

@app.get("/api/medicines")
async def get_medicines():
    """獲取所有藥物"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('SELECT medicine_code, name, description, unit FROM medicines')
    
    results = []
    for row in cursor.fetchall():
        results.append({
            'medicine_code': row[0],
            'name': row[1],
            'description': row[2],
            'unit': row[3]
        })
    
    conn.close()
    return results

@app.post("/api/orders")
async def create_order(request: CreateOrderRequest):
    """創建新訂單"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    try:
        order_id = f"ORD-{uuid.uuid4().hex[:8].upper()}"
        
        # 檢查病人是否存在
        cursor.execute('SELECT patient_id FROM patients WHERE patient_id = ?', (request.patient_id,))
        if not cursor.fetchone():
            raise HTTPException(status_code=404, detail="病人不存在")
        
        # 創建訂單
        cursor.execute('''
            INSERT INTO orders (order_id, patient_id, doctor_name, department, notes)
            VALUES (?, ?, ?, ?, ?)
        ''', (order_id, request.patient_id, request.doctor_name, request.department, request.notes))
        
        # 添加訂單項目
        for medicine in request.medicines:
            medicine_name = medicine.get('medicine_name')
            quantity = medicine.get('quantity', 1)
            
            # 查找藥物
            cursor.execute('SELECT medicine_code FROM medicines WHERE name = ?', (medicine_name,))
            medicine_row = cursor.fetchone()
            if not medicine_row:
                raise HTTPException(status_code=404, detail=f"藥物 {medicine_name} 不存在")
            
            medicine_code = medicine_row[0]
            
            # 查找預設JSON配置
            cursor.execute('''
                SELECT id FROM medicine_json_configs 
                WHERE medicine_code = ? AND is_default = 1
            ''', (medicine_code,))
            config_row = cursor.fetchone()
            config_id = config_row[0] if config_row else None
            
            # 插入訂單項目
            cursor.execute('''
                INSERT INTO order_items (order_id, medicine_code, quantity, json_config_id)
                VALUES (?, ?, ?, ?)
            ''', (order_id, medicine_code, quantity, config_id))
        
        conn.commit()
        
        return {
            "success": True,
            "order_id": order_id,
            "message": "訂單創建成功"
        }
        
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=400, detail=str(e))
    finally:
        conn.close()

@app.get("/api/orders")
async def get_orders(status: Optional[str] = None):
    """獲取訂單列表"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    if status:
        cursor.execute('''
            SELECT o.order_id, o.patient_id, p.name, o.status, o.created_at, o.doctor_name, o.department
            FROM orders o
            JOIN patients p ON o.patient_id = p.patient_id
            WHERE o.status = ?
            ORDER BY o.created_at DESC
        ''', (status,))
    else:
        cursor.execute('''
            SELECT o.order_id, o.patient_id, p.name, o.status, o.created_at, o.doctor_name, o.department
            FROM orders o
            JOIN patients p ON o.patient_id = p.patient_id
            ORDER BY o.created_at DESC
        ''')
    
    orders = []
    for row in cursor.fetchall():
        order_id = row[0]
        
        # 獲取訂單項目
        cursor.execute('''
            SELECT m.name, oi.quantity, mjc.json_data
            FROM order_items oi
            JOIN medicines m ON oi.medicine_code = m.medicine_code
            LEFT JOIN medicine_json_configs mjc ON oi.json_config_id = mjc.id
            WHERE oi.order_id = ?
        ''', (order_id,))
        
        medicines = []
        for item_row in cursor.fetchall():
            medicine_data = {
                'name': item_row[0],
                'quantity': item_row[1],
                'json_config': json.loads(item_row[2]) if item_row[2] else {}
            }
            medicines.append(medicine_data)
        
        orders.append({
            'order_id': order_id,
            'patient_id': row[1],
            'patient_name': row[2],
            'status': row[3],
            'created_at': row[4],
            'doctor_name': row[5],
            'department': row[6],
            'medicines': medicines
        })
    
    conn.close()
    return orders

@app.get("/api/orders/robot/pending")
async def get_pending_orders_for_robot():
    """供機器人系統獲取待處理訂單"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        SELECT o.order_id, o.patient_id
        FROM orders o
        WHERE o.status = 'pending'
        ORDER BY o.created_at ASC
    ''')
    
    robot_orders = []
    for row in cursor.fetchall():
        order_id = row[0]
        
        # 獲取訂單藥物和JSON配置
        cursor.execute('''
            SELECT m.name, oi.quantity, mjc.json_data
            FROM order_items oi
            JOIN medicines m ON oi.medicine_code = m.medicine_code
            LEFT JOIN medicine_json_configs mjc ON oi.json_config_id = mjc.id
            WHERE oi.order_id = ?
        ''', (order_id,))
        
        medicines = []
        for item_row in cursor.fetchall():
            medicine_data = {
                'name': item_row[0],
                'quantity': item_row[1],
                'json_config': json.loads(item_row[2]) if item_row[2] else {}
            }
            medicines.append(medicine_data)
        
        robot_orders.append({
            'order_id': order_id,
            'patient_id': row[1],
            'medicines': medicines
        })
    
    conn.close()
    return robot_orders

@app.put("/api/orders/{order_id}/complete")
async def complete_order(order_id: str):
    """標記訂單為完成"""
    conn = get_db_connection()
    cursor = conn.cursor()
    
    cursor.execute('UPDATE orders SET status = ? WHERE order_id = ?', ('completed', order_id))
    
    if cursor.rowcount == 0:
        conn.close()
        raise HTTPException(status_code=404, detail="訂單不存在")
    
    conn.commit()
    conn.close()
    
    return {"success": True, "message": "訂單已完成"}

# HTML 路由
@app.get("/", response_class=HTMLResponse)
async def read_root():
    return """
<!DOCTYPE html>
<html lang="zh-TW">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>藥局訂單管理系統 - 測試版</title>
    <style>
        body {
            font-family: 'Microsoft YaHei', Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.2);
        }
        .header {
            text-align: center;
            margin-bottom: 40px;
            padding-bottom: 20px;
            border-bottom: 3px solid #667eea;
        }
        .header h1 {
            color: #333;
            margin: 0 0 10px 0;
            font-size: 2.5em;
        }
        .status-badge {
            background: linear-gradient(45deg, #4CAF50, #45a049);
            color: white;
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 14px;
            display: inline-block;
        }
        .section {
            margin-bottom: 40px;
            padding: 25px;
            border: 2px solid #e0e0e0;
            border-radius: 10px;
            background: #fafafa;
        }
        .section h3 {
            margin-top: 0;
            color: #444;
            font-size: 1.3em;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: bold;
            color: #555;
        }
        .form-control {
            width: 100%;
            padding: 12px;
            border: 2px solid #ddd;
            border-radius: 8px;
            box-sizing: border-box;
            font-size: 16px;
            transition: border-color 0.3s;
        }
        .form-control:focus {
            border-color: #667eea;
            outline: none;
            box-shadow: 0 0 10px rgba(102, 126, 234, 0.3);
        }
        .btn {
            padding: 12px 25px;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            font-weight: bold;
            transition: all 0.3s;
            margin: 5px;
        }
        .btn-primary {
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white;
        }
        .btn-primary:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
        }
        .btn-success {
            background: linear-gradient(45deg, #4CAF50, #45a049);
            color: white;
        }
        .btn-success:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(76, 175, 80, 0.4);
        }
        .btn-danger {
            background: linear-gradient(45deg, #f44336, #da190b);
            color: white;
        }
        .btn-info {
            background: linear-gradient(45deg, #17a2b8, #138496);
            color: white;
        }
        .patient-search {
            position: relative;
        }
        .search-results {
            position: absolute;
            top: 100%;
            left: 0;
            right: 0;
            background: white;
            border: 2px solid #667eea;
            border-top: none;
            max-height: 200px;
            overflow-y: auto;
            z-index: 1000;
            border-radius: 0 0 8px 8px;
        }
        .search-result-item {
            padding: 12px;
            cursor: pointer;
            border-bottom: 1px solid #eee;
            transition: background-color 0.3s;
        }
        .search-result-item:hover {
            background-color: #f0f8ff;
        }
        .medicine-item {
            display: flex;
            gap: 15px;
            align-items: center;
            margin-bottom: 15px;
            padding: 15px;
            background-color: white;
            border-radius: 8px;
            border: 1px solid #ddd;
        }
        .medicine-item select, .medicine-item input {
            flex: 1;
        }
        .order-item {
            background: linear-gradient(45deg, #f8f9fa, #e9ecef);
            border: 2px solid #dee2e6;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
            transition: transform 0.3s;
        }
        .order-item:hover {
            transform: translateY(-3px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.1);
        }
        .order-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
        }
        .status {
            padding: 6px 15px;
            border-radius: 25px;
            font-size: 12px;
            font-weight: bold;
            text-transform: uppercase;
        }
        .status.pending {
            background: linear-gradient(45deg, #ffc107, #e0a800);
            color: #212529;
        }
        .status.completed {
            background: linear-gradient(45deg, #28a745, #1e7e34);
            color: white;
        }
        .medicine-tag {
            display: inline-block;
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white;
            padding: 6px 12px;
            border-radius: 15px;
            font-size: 12px;
            margin-right: 8px;
            margin-bottom: 8px;
        }
        .selected-patient {
            padding: 15px;
            background: linear-gradient(45deg, #e3f2fd, #bbdefb);
            border-radius: 8px;
            border: 2px solid #2196f3;
        }
        .loading {
            text-align: center;
            padding: 20px;
            color: #666;
        }
        .alert {
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
        }
        .alert-success {
            background-color: #d4edda;
            border: 1px solid #c3e6cb;
            color: #155724;
        }
        .alert-error {
            background-color: #f8d7da;
            border: 1px solid #f5c6cb;
            color: #721c24;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🏥 藥局訂單管理系統</h1>
            <span class="status-badge">✨ 測試版本 v1.0</span>
        </div>

        <!-- 創建新訂單 -->
        <div class="section">
            <h3>📝 創建新訂單</h3>
            <form id="orderForm">
                <div class="form-group">
                    <label>🔍 病人搜尋</label>
                    <div class="patient-search">
                        <input type="text" id="patientSearch" class="form-control" 
                               placeholder="輸入病歷號或姓名搜尋病人...">
                        <div id="searchResults" class="search-results" style="display: none;"></div>
                    </div>
                </div>

                <div class="form-group">
                    <label>👤 選中病人</label>
                    <div id="selectedPatient" class="selected-patient">
                        未選擇病人
                    </div>
                </div>

                <div class="form-group">
                    <label>👨‍⚕️ 醫師姓名</label>
                    <input type="text" id="doctorName" class="form-control" placeholder="請輸入醫師姓名">
                </div>

                <div class="form-group">
                    <label>🏥 科別</label>
                    <input type="text" id="department" class="form-control" placeholder="請輸入科別">
                </div>

                <div class="form-group">
                    <label>💊 藥物清單</label>
                    <div id="medicineList">
                        <div class="medicine-item">
                            <select class="form-control medicine-select">
                                <option value="">選擇藥物...</option>
                            </select>
                            <input type="number" class="form-control" placeholder="數量" min="1" value="1">
                            <button type="button" onclick="removeMedicine(this)" class="btn btn-danger">❌ 移除</button>
                        </div>
                    </div>
                    <button type="button" onclick="addMedicine()" class="btn btn-success">➕ 新增藥物</button>
                </div>

                <div class="form-group">
                    <label>📝 備註</label>
                    <textarea id="notes" class="form-control" rows="3" placeholder="請輸入備註信息..."></textarea>
                </div>

                <button type="submit" class="btn btn-primary">🚀 創建訂單</button>
            </form>
        </div>

        <!-- 訂單列表 -->
        <div class="section">
            <h3>📋 訂單列表</h3>
            <button onclick="refreshOrders()" class="btn btn-info">🔄 刷新列表</button>
            <div id="orderList" class="loading">
                載入中...
            </div>
        </div>
    </div>

    <script>
        let selectedPatient = null;
        let medicines = [];

        // 初始化
        document.addEventListener('DOMContentLoaded', function() {
            loadMedicines();
            refreshOrders();
        });

        // 載入藥物清單
        async function loadMedicines() {
            try {
                const response = await fetch('/api/medicines');
                medicines = await response.json();
                updateMedicineSelects();
            } catch (error) {
                console.error('載入藥物失敗:', error);
                showAlert('載入藥物清單失敗', 'error');
            }
        }

        // 更新藥物選擇器
        function updateMedicineSelects() {
            const selects = document.querySelectorAll('.medicine-select');
            selects.forEach(select => {
                select.innerHTML = '<option value="">選擇藥物...</option>';
                medicines.forEach(medicine => {
                    const option = document.createElement('option');
                    option.value = medicine.name;
                    option.textContent = `${medicine.name} - ${medicine.description}`;
                    select.appendChild(option);
                });
            });
        }

        // 病人搜尋
        document.getElementById('patientSearch').addEventListener('input', async function(e) {
            const keyword = e.target.value.trim();
            const resultsDiv = document.getElementById('searchResults');
            
            if (keyword.length < 2) {
                resultsDiv.style.display = 'none';
                return;
            }

            try {
                const response = await fetch(`/api/patients/search?keyword=${encodeURIComponent(keyword)}`);
                const patients = await response.json();
                
                resultsDiv.innerHTML = '';
                if (patients.length > 0) {
                    patients.forEach(patient => {
                        const item = document.createElement('div');
                        item.className = 'search-result-item';
                        item.innerHTML = `
                            <strong>${patient.name}</strong> (${patient.patient_id})<br>
                            <small>年齡: ${patient.age} | ${patient.medical_record || '無病歷摘要'}</small>
                        `;
                        item.onclick = () => selectPatient(patient);
                        resultsDiv.appendChild(item);
                    });
                    resultsDiv.style.display = 'block';
                } else {
                    resultsDiv.innerHTML = '<div class="search-result-item">找不到相符的病人</div>';
                    resultsDiv.style.display = 'block';
                }
            } catch (error) {
                console.error('搜尋病人失敗:', error);
                showAlert('搜尋病人失敗', 'error');
            }
        });

        // 選擇病人
        function selectPatient(patient) {
            selectedPatient = patient;
            document.getElementById('selectedPatient').innerHTML = `
                <strong>👤 ${patient.name}</strong> (${patient.patient_id})<br>
                <small>年齡: ${patient.age} | ${patient.medical_record || '無病歷摘要'}</small>
            `;
            document.getElementById('patientSearch').value = '';
            document.getElementById('searchResults').style.display = 'none';
        }

        // 新增藥物
        function addMedicine() {
            const medicineList = document.getElementById('medicineList');
            const newItem = document.createElement('div');
            newItem.className = 'medicine-item';
            newItem.innerHTML = `
                <select class="form-control medicine-select">
                    <option value="">選擇藥物...</option>
                </select>
                <input type="number" class="form-control" placeholder="數量" min="1" value="1">
                <button type="button" onclick="removeMedicine(this)" class="btn btn-danger">❌ 移除</button>
            `;
            medicineList.insertBefore(newItem, medicineList.lastElementChild);
            updateMedicineSelects();
        }

        // 移除藥物
        function removeMedicine(button) {
            const medicineItems = document.querySelectorAll('.medicine-item');
            if (medicineItems.length > 1) {
                button.closest('.medicine-item').remove();
            } else {
                showAlert('至少需要保留一個藥物項目', 'error');
            }
        }

        // 提交訂單
        document.getElementById('orderForm').addEventListener('submit', async function(e) {
            e.preventDefault();

            if (!selectedPatient) {
                showAlert('請選擇病人', 'error');
                return;
            }

            const medicineItems = document.querySelectorAll('.medicine-item');
            const orderMedicines = [];

            for (let item of medicineItems) {
                const select = item.querySelector('.medicine-select');
                const quantity = item.querySelector('input[type="number"]');
                
                if (select.value && quantity.value) {
                    orderMedicines.push({
                        medicine_name: select.value,
                        quantity: parseInt(quantity.value)
                    });
                }
            }

            if (orderMedicines.length === 0) {
                showAlert('請至少選擇一種藥物', 'error');
                return;
            }

            const orderData = {
                patient_id: selectedPatient.patient_id,
                medicines: orderMedicines,
                doctor_name: document.getElementById('doctorName').value,
                department: document.getElementById('department').value,
                notes: document.getElementById('notes').value
            };

            try {
                const response = await fetch('/api/orders', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(orderData)
                });

                const result = await response.json();
                if (result.success) {
                    showAlert(`訂單創建成功！訂單號：${result.order_id}`, 'success');
                    // 重置表單
                    document.getElementById('orderForm').reset();
                    selectedPatient = null;
                    document.getElementById('selectedPatient').innerHTML = '未選擇病人';
                    refreshOrders();
                } else {
                    showAlert('訂單創建失敗：' + result.message, 'error');
                }
            } catch (error) {
                console.error('創建訂單失敗:', error);
                showAlert('創建訂單失敗：網路錯誤', 'error');
            }
        });

        // 刷新訂單列表
        async function refreshOrders() {
            const orderList = document.getElementById('orderList');
            orderList.innerHTML = '<div class="loading">載入中...</div>';

            try {
                const response = await fetch('/api/orders');
                const orders = await response.json();
                
                orderList.innerHTML = '';

                if (orders.length === 0) {
                    orderList.innerHTML = '<p style="text-align: center; color: #666;">暫無訂單</p>';
                    return;
                }

                orders.forEach(order => {
                    const orderDiv = document.createElement('div');
                    orderDiv.className = 'order-item';
                    
                    const medicineList = order.medicines.map(med => 
                        `<span class="medicine-tag">${med.name} x${med.quantity}</span>`
                    ).join('');

                    const statusText = order.status === 'pending' ? '⏳ 處理中' : '✅ 已完成';
                    const statusClass = order.status;

                    orderDiv.innerHTML = `
                        <div class="order-header">
                            <h4>📋 訂單 ${order.order_id}</h4>
                            <span class="status ${statusClass}">${statusText}</span>
                        </div>
                        <p><strong>👤 病人：</strong>${order.patient_name} (${order.patient_id})</p>
                        <p><strong>👨‍⚕️ 醫師：</strong>${order.doctor_name || '未指定'} | <strong>🏥 科別：</strong>${order.department || '未指定'}</p>
                        <div style="margin-top: 10px;">
                            <strong>💊 藥物：</strong><br>
                            ${medicineList}
                        </div>
                        <p style="margin-top: 15px;"><small>📅 創建時間：${new Date(order.created_at).toLocaleString('zh-TW')}</small></p>
                        ${order.status === 'pending' ? `<button onclick="completeOrder('${order.order_id}')" class="btn btn-success">✅ 標記完成</button>` : ''}
                    `;
                    
                    orderList.appendChild(orderDiv);
                });
            } catch (error) {
                console.error('載入訂單失敗:', error);
                orderList.innerHTML = '<p style="color: red; text-align: center;">載入訂單失敗</p>';
            }
        }

        // 完成訂單
        async function completeOrder(orderId) {
            if (!confirm('確定要標記此訂單為完成嗎？')) {
                return;
            }

            try {
                const response = await fetch(`/api/orders/${orderId}/complete`, {
                    method: 'PUT'
                });

                const result = await response.json();
                if (result.success) {
                    showAlert('訂單已標記為完成', 'success');
                    refreshOrders();
                } else {
                    showAlert('操作失敗', 'error');
                }
            } catch (error) {
                console.error('完成訂單失敗:', error);
                showAlert('操作失敗：網路錯誤', 'error');
            }
        }

        // 顯示提示信息
        function showAlert(message, type = 'success') {
            const alertDiv = document.createElement('div');
            alertDiv.className = `alert alert-${type}`;
            alertDiv.textContent = message;
            
            const container = document.querySelector('.container');
            container.insertBefore(alertDiv, container.firstChild);
            
            setTimeout(() => {
                alertDiv.remove();
            }, 5000);
        }

        // 點擊外部關閉搜尋結果
        document.addEventListener('click', function(e) {
            if (!e.target.closest('.patient-search')) {
                document.getElementById('searchResults').style.display = 'none';
            }
        });
    </script>
</body>
</html>
    """

if __name__ == "__main__":
    import uvicorn
    print("🚀 啟動藥局測試系統...")
    print("📱 網址: http://localhost:8000")
    print("🔧 測試功能:")
    print("   • 病人搜尋")
    print("   • 藥物選擇") 
    print("   • 訂單創建")
    print("   • 訂單管理")
    print("   • JSON配置整合")
    uvicorn.run(app, host="0.0.0.0", port=8000)