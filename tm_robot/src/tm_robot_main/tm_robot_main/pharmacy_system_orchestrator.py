#!/usr/bin/env python3
"""
藥局系統總協調器
統一管理從訂單接收、藥物識別、抓取、到分發輸出的完整流程
保密性和安全性優先
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from tm_robot_if.srv import GroundedSAM2Interface, CaptureImage, PoseSrv
import json
import time
import threading
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Any
from enum import Enum
import sqlite3
import hashlib
import yaml
from pathlib import Path
import logging
from dataclasses import dataclass, asdict
from queue import Queue, Empty
import signal


class SystemState(Enum):
    """系統狀態枚舉"""
    IDLE = "idle"
    RECEIVING_ORDER = "receiving_order"
    PROCESSING_VISION = "processing_vision"
    PLANNING_GRASP = "planning_grasp"
    EXECUTING_GRASP = "executing_grasp"
    CONFIRMING_MEDICINE = "confirming_medicine"
    PACKAGING = "packaging"
    DISPENSING = "dispensing"
    COMPLETED = "completed"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


class TaskPriority(Enum):
    """任務優先級"""
    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4
    EMERGENCY = 5


@dataclass
class MedicineTask:
    """藥物任務數據結構"""
    task_id: str
    order_id: str
    medicine_name: str
    quantity: int
    priority: TaskPriority
    patient_id: str
    prescription_id: str
    status: str
    created_time: float
    updated_time: float
    detection_result: Optional[Dict] = None
    grasp_result: Optional[Dict] = None
    confirmation_result: Optional[Dict] = None
    error_message: Optional[str] = None


@dataclass
class SystemMetrics:
    """系統性能指標"""
    total_orders: int = 0
    completed_orders: int = 0
    failed_orders: int = 0
    average_processing_time: float = 0.0
    success_rate: float = 0.0
    current_load: float = 0.0
    uptime: float = 0.0


class PharmacySystemOrchestrator(Node):
    """
    藥局系統總協調器
    負責整個系統的工作流程管理和協調
    """
    
    def __init__(self):
        super().__init__('pharmacy_system_orchestrator')
        
        # 初始化系統狀態
        self.system_state = SystemState.IDLE
        self.startup_time = time.time()
        self.current_task: Optional[MedicineTask] = None
        self.task_queue = Queue()
        self.active_tasks: Dict[str, MedicineTask] = {}
        self.completed_tasks: List[MedicineTask] = []
        
        # 安全和認證
        self.authenticated_users = set()
        self.security_token = self._generate_security_token()
        self.access_log = []
        
        # 系統指標
        self.metrics = SystemMetrics()
        self.performance_log = []
        
        # 載入配置
        self.config = self._load_system_config()
        
        # 初始化資料庫
        self._init_database()
        
        # 設置日誌系統
        self._setup_logging()
        
        # ROS通信設置
        self._setup_ros_communication()
        
        # 啟動工作線程
        self._start_worker_threads()
        
        # 設置信號處理
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        self.get_logger().info("藥局系統總協調器初始化完成")
        self.get_logger().info(f"系統安全令牌: {self.security_token[:8]}...")
    
    def _load_system_config(self) -> Dict:
        """載入系統配置"""
        try:
            config_path = Path(__file__).parent / "system_config.yaml"
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    return yaml.safe_load(f)
            else:
                return self._get_default_config()
        except Exception as e:
            self.get_logger().error(f"載入配置失敗: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict:
        """獲取默認系統配置"""
        return {
            'security': {
                'enable_authentication': True,
                'session_timeout': 3600,
                'max_failed_attempts': 3,
                'audit_logging': True
            },
            'workflow': {
                'max_concurrent_tasks': 5,
                'task_timeout': 300,
                'retry_attempts': 3,
                'auto_recovery': True
            },
            'performance': {
                'target_processing_time': 60.0,
                'max_queue_size': 20,
                'monitoring_interval': 10.0
            },
            'database': {
                'path': '/workspace/pharmacy_system.db',
                'backup_interval': 3600,
                'retention_days': 30
            }
        }
    
    def _init_database(self):
        """初始化系統資料庫"""
        try:
            db_path = self.config['database']['path']
            self.db_conn = sqlite3.connect(db_path, check_same_thread=False)
            self.db_lock = threading.Lock()
            
            # 創建必要的表
            with self.db_lock:
                cursor = self.db_conn.cursor()
                
                # 訂單表
                cursor.execute('''
                    CREATE TABLE IF NOT EXISTS orders (
                        order_id TEXT PRIMARY KEY,
                        patient_id TEXT NOT NULL,
                        prescription_id TEXT,
                        status TEXT NOT NULL,
                        priority INTEGER NOT NULL,
                        created_time REAL NOT NULL,
                        completed_time REAL,
                        total_medicines INTEGER,
                        processed_medicines INTEGER,
                        notes TEXT
                    )
                ''')
                
                # 藥物任務表
                cursor.execute('''
                    CREATE TABLE IF NOT EXISTS medicine_tasks (
                        task_id TEXT PRIMARY KEY,
                        order_id TEXT NOT NULL,
                        medicine_name TEXT NOT NULL,
                        quantity INTEGER NOT NULL,
                        status TEXT NOT NULL,
                        created_time REAL NOT NULL,
                        completed_time REAL,
                        detection_score REAL,
                        grasp_success BOOLEAN,
                        confirmation_result TEXT,
                        error_message TEXT,
                        FOREIGN KEY (order_id) REFERENCES orders (order_id)
                    )
                ''')
                
                # 操作日誌表
                cursor.execute('''
                    CREATE TABLE IF NOT EXISTS operation_logs (
                        log_id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp REAL NOT NULL,
                        user_id TEXT,
                        operation TEXT NOT NULL,
                        details TEXT,
                        success BOOLEAN,
                        error_message TEXT
                    )
                ''')
                
                # 系統指標表
                cursor.execute('''
                    CREATE TABLE IF NOT EXISTS system_metrics (
                        metric_id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp REAL NOT NULL,
                        metric_name TEXT NOT NULL,
                        metric_value REAL NOT NULL,
                        details TEXT
                    )
                ''')
                
                self.db_conn.commit()
                
        except Exception as e:
            self.get_logger().error(f"資料庫初始化失敗: {e}")
    
    def _setup_logging(self):
        """設置系統日誌"""
        self.logger = logging.getLogger('PharmacySystem')
        self.logger.setLevel(logging.INFO)
        
        # 創建文件處理器
        log_path = Path('/tmp/pharmacy_system.log')
        handler = logging.FileHandler(log_path)
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
    
    def _setup_ros_communication(self):
        """設置ROS通信"""
        # 訂閱者
        self.order_sub = self.create_subscription(
            String, '/pharmacy/new_order', self.order_callback, 10)
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        self.vision_result_sub = self.create_subscription(
            String, '/vision/detection_result', self.vision_result_callback, 10)
        
        # 發布者
        self.status_pub = self.create_publisher(
            String, '/pharmacy/system_status', 10)
        self.command_pub = self.create_publisher(
            String, '/pharmacy/command', 10)
        self.metrics_pub = self.create_publisher(
            String, '/pharmacy/metrics', 10)
        
        # 服務客戶端
        self.vision_client = self.create_client(
            GroundedSAM2Interface, 'vision/detect_medicine')
        self.grasp_client = self.create_client(
            CaptureImage, 'enhanced_grab_detect')
        self.robot_control_client = self.create_client(
            PoseSrv, 'thing_pose')
        
        # 服務提供者
        self.system_control_service = self.create_service(
            String, 'pharmacy/system_control', self.system_control_callback)
        
        # 定時器
        self.status_timer = self.create_timer(
            self.config['performance']['monitoring_interval'], 
            self.publish_system_status)
        self.metrics_timer = self.create_timer(60.0, self.update_metrics)
    
    def _start_worker_threads(self):
        """啟動工作線程"""
        # 任務處理線程
        self.task_worker = threading.Thread(
            target=self._task_processing_worker, daemon=True)
        self.task_worker.start()
        
        # 系統監控線程
        self.monitor_worker = threading.Thread(
            target=self._system_monitor_worker, daemon=True)
        self.monitor_worker.start()
        
        # 資料庫備份線程
        self.backup_worker = threading.Thread(
            target=self._database_backup_worker, daemon=True)
        self.backup_worker.start()
    
    def _generate_security_token(self) -> str:
        """生成安全令牌"""
        timestamp = str(time.time())
        random_data = str(uuid.uuid4())
        token_data = f"{timestamp}_{random_data}"
        return hashlib.sha256(token_data.encode()).hexdigest()
    
    def order_callback(self, msg: String):
        """處理新訂單"""
        try:
            order_data = json.loads(msg.data)
            self._log_operation("order_received", order_data)
            
            # 驗證訂單數據
            if not self._validate_order(order_data):
                self.get_logger().error("訂單數據驗證失敗")
                return
            
            # 創建任務
            tasks = self._create_tasks_from_order(order_data)
            
            # 添加到任務隊列
            for task in tasks:
                self.task_queue.put(task)
                self.active_tasks[task.task_id] = task
                self._save_task_to_db(task)
            
            self.get_logger().info(f"已接收訂單 {order_data.get('order_id', 'Unknown')}，創建 {len(tasks)} 個任務")
            
        except Exception as e:
            self.get_logger().error(f"處理訂單失敗: {e}")
            self._log_operation("order_processing_error", {"error": str(e)})
    
    def emergency_stop_callback(self, msg: Bool):
        """緊急停止回調"""
        if msg.data:
            self.get_logger().warn("收到緊急停止信號")
            self.system_state = SystemState.EMERGENCY_STOP
            self._handle_emergency_stop()
    
    def vision_result_callback(self, msg: String):
        """視覺識別結果回調"""
        try:
            result_data = json.loads(msg.data)
            task_id = result_data.get('task_id')
            
            if task_id and task_id in self.active_tasks:
                task = self.active_tasks[task_id]
                task.detection_result = result_data
                task.status = "vision_completed"
                task.updated_time = time.time()
                
                self.get_logger().info(f"任務 {task_id} 視覺識別完成")
                
        except Exception as e:
            self.get_logger().error(f"處理視覺結果失敗: {e}")
    
    def system_control_callback(self, request, response):
        """系統控制服務回調"""
        try:
            command_data = json.loads(request.data)
            command = command_data.get('command')
            
            # 安全檢查
            if not self._verify_access(command_data):
                response.data = json.dumps({"success": False, "error": "權限不足"})
                return response
            
            if command == "pause":
                self._pause_system()
            elif command == "resume":
                self._resume_system()
            elif command == "reset":
                self._reset_system()
            elif command == "shutdown":
                self._shutdown_system()
            else:
                response.data = json.dumps({"success": False, "error": "未知命令"})
                return response
            
            response.data = json.dumps({"success": True})
            
        except Exception as e:
            response.data = json.dumps({"success": False, "error": str(e)})
        
        return response
    
    def _validate_order(self, order_data: Dict) -> bool:
        """驗證訂單數據"""
        required_fields = ['order_id', 'patient_id', 'medicines']
        
        for field in required_fields:
            if field not in order_data:
                self.get_logger().error(f"訂單缺少必要欄位: {field}")
                return False
        
        medicines = order_data['medicines']
        if not isinstance(medicines, list) or len(medicines) == 0:
            self.get_logger().error("訂單中沒有有效的藥物列表")
            return False
        
        return True
    
    def _create_tasks_from_order(self, order_data: Dict) -> List[MedicineTask]:
        """從訂單創建任務"""
        tasks = []
        order_id = order_data['order_id']
        patient_id = order_data['patient_id']
        prescription_id = order_data.get('prescription_id', '')
        priority = TaskPriority(order_data.get('priority', TaskPriority.NORMAL.value))
        
        for medicine in order_data['medicines']:
            task = MedicineTask(
                task_id=str(uuid.uuid4()),
                order_id=order_id,
                medicine_name=medicine['name'],
                quantity=medicine.get('quantity', 1),
                priority=priority,
                patient_id=patient_id,
                prescription_id=prescription_id,
                status="pending",
                created_time=time.time(),
                updated_time=time.time()
            )
            tasks.append(task)
        
        # 保存訂單到資料庫
        self._save_order_to_db(order_data, tasks)
        
        return tasks
    
    def _task_processing_worker(self):
        """任務處理工作線程"""
        while True:
            try:
                if self.system_state == SystemState.EMERGENCY_STOP:
                    time.sleep(1)
                    continue
                
                try:
                    task = self.task_queue.get(timeout=1)
                except Empty:
                    continue
                
                self.current_task = task
                self._process_single_task(task)
                self.current_task = None
                
            except Exception as e:
                self.get_logger().error(f"任務處理線程錯誤: {e}")
                time.sleep(1)
    
    def _process_single_task(self, task: MedicineTask):
        """處理單個任務"""
        try:
            self.get_logger().info(f"開始處理任務: {task.task_id} - {task.medicine_name}")
            
            # 1. 視覺識別階段
            if not self._execute_vision_detection(task):
                self._handle_task_failure(task, "視覺識別失敗")
                return
            
            # 2. 抓取規劃階段
            if not self._execute_grasp_planning(task):
                self._handle_task_failure(task, "抓取規劃失敗")
                return
            
            # 3. 執行抓取階段
            if not self._execute_grasp_action(task):
                self._handle_task_failure(task, "抓取執行失敗")
                return
            
            # 4. 二次確認階段（如果需要）
            if self._needs_confirmation(task):
                if not self._execute_confirmation(task):
                    self._handle_task_failure(task, "二次確認失敗")
                    return
            
            # 5. 包裝和分發階段
            if not self._execute_packaging(task):
                self._handle_task_failure(task, "包裝失敗")
                return
            
            # 任務完成
            self._complete_task(task)
            
        except Exception as e:
            self.get_logger().error(f"處理任務時發生錯誤: {e}")
            self._handle_task_failure(task, str(e))
    
    def _execute_vision_detection(self, task: MedicineTask) -> bool:
        """執行視覺識別"""
        try:
            self.system_state = SystemState.PROCESSING_VISION
            task.status = "vision_processing"
            
            # 調用視覺識別服務
            # 這裡會與整合視覺系統進行通信
            
            # 模擬視覺識別過程
            time.sleep(2)  # 實際實現時會調用真實服務
            
            # 模擬結果
            task.detection_result = {
                "success": True,
                "confidence": 0.85,
                "medicine_detected": task.medicine_name,
                "bbox": [100, 100, 200, 200]
            }
            
            task.status = "vision_completed"
            task.updated_time = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"視覺識別執行失敗: {e}")
            return False
    
    def _execute_grasp_planning(self, task: MedicineTask) -> bool:
        """執行抓取規劃"""
        try:
            self.system_state = SystemState.PLANNING_GRASP
            task.status = "grasp_planning"
            
            # 模擬抓取規劃
            time.sleep(1)
            
            task.grasp_result = {
                "success": True,
                "grasp_pose": [0.1, 0.2, 0.3, 0, 0, 0, 1],
                "confidence": 0.9
            }
            
            task.status = "grasp_planned"
            task.updated_time = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"抓取規劃失敗: {e}")
            return False
    
    def _execute_grasp_action(self, task: MedicineTask) -> bool:
        """執行抓取動作"""
        try:
            self.system_state = SystemState.EXECUTING_GRASP
            task.status = "grasp_executing"
            
            # 模擬抓取執行
            time.sleep(3)
            
            task.status = "grasp_completed"
            task.updated_time = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"抓取執行失敗: {e}")
            return False
    
    def _needs_confirmation(self, task: MedicineTask) -> bool:
        """判斷是否需要二次確認"""
        if not task.detection_result:
            return True
        
        confidence = task.detection_result.get('confidence', 0)
        return confidence < 0.8  # 置信度低於0.8需要確認
    
    def _execute_confirmation(self, task: MedicineTask) -> bool:
        """執行二次確認"""
        try:
            self.system_state = SystemState.CONFIRMING_MEDICINE
            task.status = "confirming"
            
            # 調用LLM確認服務
            time.sleep(2)
            
            task.confirmation_result = {
                "confirmed": True,
                "confidence": 0.9,
                "method": "llm_verification"
            }
            
            task.status = "confirmed"
            task.updated_time = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"二次確認失敗: {e}")
            return False
    
    def _execute_packaging(self, task: MedicineTask) -> bool:
        """執行包裝和分發"""
        try:
            self.system_state = SystemState.PACKAGING
            task.status = "packaging"
            
            # 模擬包裝過程
            time.sleep(2)
            
            task.status = "packaged"
            task.updated_time = time.time()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"包裝失敗: {e}")
            return False
    
    def _complete_task(self, task: MedicineTask):
        """完成任務"""
        task.status = "completed"
        task.updated_time = time.time()
        
        # 從活動任務中移除
        if task.task_id in self.active_tasks:
            del self.active_tasks[task.task_id]
        
        # 添加到完成列表
        self.completed_tasks.append(task)
        
        # 更新資料庫
        self._update_task_in_db(task)
        
        # 更新指標
        self.metrics.completed_orders += 1
        
        self.get_logger().info(f"任務完成: {task.task_id} - {task.medicine_name}")
        
        # 檢查訂單是否完全完成
        self._check_order_completion(task.order_id)
    
    def _handle_task_failure(self, task: MedicineTask, error_message: str):
        """處理任務失敗"""
        task.status = "failed"
        task.error_message = error_message
        task.updated_time = time.time()
        
        self.get_logger().error(f"任務失敗: {task.task_id} - {error_message}")
        
        # 更新資料庫
        self._update_task_in_db(task)
        
        # 更新指標
        self.metrics.failed_orders += 1
        
        # 記錄操作日誌
        self._log_operation("task_failed", {
            "task_id": task.task_id,
            "error": error_message
        })
    
    def _check_order_completion(self, order_id: str):
        """檢查訂單是否完成"""
        order_tasks = [task for task in self.completed_tasks + list(self.active_tasks.values()) 
                      if task.order_id == order_id]
        
        total_tasks = len(order_tasks)
        completed_tasks = len([task for task in order_tasks if task.status == "completed"])
        
        if completed_tasks == total_tasks:
            self.get_logger().info(f"訂單 {order_id} 已完成")
            self._log_operation("order_completed", {"order_id": order_id})
    
    def _system_monitor_worker(self):
        """系統監控工作線程"""
        while True:
            try:
                # 監控系統性能
                self._monitor_system_performance()
                
                # 檢查任務超時
                self._check_task_timeouts()
                
                # 清理過期數據
                self._cleanup_old_data()
                
                time.sleep(self.config['performance']['monitoring_interval'])
                
            except Exception as e:
                self.get_logger().error(f"系統監控錯誤: {e}")
                time.sleep(5)
    
    def _database_backup_worker(self):
        """資料庫備份工作線程"""
        while True:
            try:
                time.sleep(self.config['database']['backup_interval'])
                self._backup_database()
            except Exception as e:
                self.get_logger().error(f"資料庫備份錯誤: {e}")
    
    def _save_task_to_db(self, task: MedicineTask):
        """保存任務到資料庫"""
        try:
            with self.db_lock:
                cursor = self.db_conn.cursor()
                cursor.execute('''
                    INSERT INTO medicine_tasks 
                    (task_id, order_id, medicine_name, quantity, status, created_time)
                    VALUES (?, ?, ?, ?, ?, ?)
                ''', (task.task_id, task.order_id, task.medicine_name, 
                     task.quantity, task.status, task.created_time))
                self.db_conn.commit()
        except Exception as e:
            self.get_logger().error(f"保存任務到資料庫失敗: {e}")
    
    def _save_order_to_db(self, order_data: Dict, tasks: List[MedicineTask]):
        """保存訂單到資料庫"""
        try:
            with self.db_lock:
                cursor = self.db_conn.cursor()
                cursor.execute('''
                    INSERT INTO orders 
                    (order_id, patient_id, prescription_id, status, priority, 
                     created_time, total_medicines, processed_medicines)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ''', (order_data['order_id'], order_data['patient_id'],
                     order_data.get('prescription_id', ''), 'processing',
                     order_data.get('priority', TaskPriority.NORMAL.value),
                     time.time(), len(tasks), 0))
                self.db_conn.commit()
        except Exception as e:
            self.get_logger().error(f"保存訂單到資料庫失敗: {e}")
    
    def _update_task_in_db(self, task: MedicineTask):
        """更新資料庫中的任務"""
        try:
            with self.db_lock:
                cursor = self.db_conn.cursor()
                cursor.execute('''
                    UPDATE medicine_tasks 
                    SET status = ?, completed_time = ?, 
                        detection_score = ?, grasp_success = ?, 
                        confirmation_result = ?, error_message = ?
                    WHERE task_id = ?
                ''', (task.status, 
                     task.updated_time if task.status == "completed" else None,
                     task.detection_result.get('confidence') if task.detection_result else None,
                     task.grasp_result.get('success') if task.grasp_result else None,
                     json.dumps(task.confirmation_result) if task.confirmation_result else None,
                     task.error_message,
                     task.task_id))
                self.db_conn.commit()
        except Exception as e:
            self.get_logger().error(f"更新任務資料庫失敗: {e}")
    
    def _log_operation(self, operation: str, details: Dict):
        """記錄操作日誌"""
        try:
            with self.db_lock:
                cursor = self.db_conn.cursor()
                cursor.execute('''
                    INSERT INTO operation_logs 
                    (timestamp, operation, details, success)
                    VALUES (?, ?, ?, ?)
                ''', (time.time(), operation, json.dumps(details), True))
                self.db_conn.commit()
        except Exception as e:
            self.get_logger().error(f"記錄操作日誌失敗: {e}")
    
    def _verify_access(self, command_data: Dict) -> bool:
        """驗證訪問權限"""
        # 簡化的權限驗證
        return True  # 實際實現時需要完整的認證機制
    
    def publish_system_status(self):
        """發布系統狀態"""
        try:
            status_data = {
                'system_state': self.system_state.value,
                'active_tasks': len(self.active_tasks),
                'queue_size': self.task_queue.qsize(),
                'metrics': asdict(self.metrics),
                'timestamp': time.time(),
                'uptime': time.time() - self.startup_time
            }
            
            msg = String()
            msg.data = json.dumps(status_data)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"發布系統狀態失敗: {e}")
    
    def update_metrics(self):
        """更新系統指標"""
        try:
            current_time = time.time()
            self.metrics.uptime = current_time - self.startup_time
            self.metrics.current_load = len(self.active_tasks) / self.config['workflow']['max_concurrent_tasks']
            
            if self.metrics.total_orders > 0:
                self.metrics.success_rate = self.metrics.completed_orders / self.metrics.total_orders
            
            # 發布指標
            metrics_data = asdict(self.metrics)
            msg = String()
            msg.data = json.dumps(metrics_data)
            self.metrics_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"更新系統指標失敗: {e}")
    
    def _monitor_system_performance(self):
        """監控系統性能"""
        # 實現性能監控邏輯
        pass
    
    def _check_task_timeouts(self):
        """檢查任務超時"""
        current_time = time.time()
        timeout_threshold = self.config['workflow']['task_timeout']
        
        for task in list(self.active_tasks.values()):
            if current_time - task.created_time > timeout_threshold:
                self.get_logger().warn(f"任務超時: {task.task_id}")
                self._handle_task_failure(task, "任務執行超時")
    
    def _cleanup_old_data(self):
        """清理過期數據"""
        # 實現數據清理邏輯
        pass
    
    def _backup_database(self):
        """備份資料庫"""
        try:
            backup_path = f"/tmp/pharmacy_backup_{int(time.time())}.db"
            with self.db_lock:
                # 創建資料庫備份
                backup_conn = sqlite3.connect(backup_path)
                self.db_conn.backup(backup_conn)
                backup_conn.close()
            
            self.get_logger().info(f"資料庫已備份到: {backup_path}")
            
        except Exception as e:
            self.get_logger().error(f"資料庫備份失敗: {e}")
    
    def _handle_emergency_stop(self):
        """處理緊急停止"""
        self.get_logger().warn("執行緊急停止程序")
        
        # 停止所有正在進行的任務
        for task in self.active_tasks.values():
            task.status = "emergency_stopped"
            task.error_message = "緊急停止"
        
        # 清空任務隊列
        while not self.task_queue.empty():
            try:
                self.task_queue.get_nowait()
            except Empty:
                break
    
    def _pause_system(self):
        """暫停系統"""
        self.system_state = SystemState.IDLE
        self.get_logger().info("系統已暫停")
    
    def _resume_system(self):
        """恢復系統"""
        if self.system_state == SystemState.EMERGENCY_STOP:
            self.get_logger().warn("系統處於緊急停止狀態，無法恢復")
            return
        
        self.system_state = SystemState.IDLE
        self.get_logger().info("系統已恢復")
    
    def _reset_system(self):
        """重置系統"""
        self.active_tasks.clear()
        while not self.task_queue.empty():
            try:
                self.task_queue.get_nowait()
            except Empty:
                break
        
        self.system_state = SystemState.IDLE
        self.get_logger().info("系統已重置")
    
    def _shutdown_system(self):
        """關閉系統"""
        self.get_logger().info("開始系統關閉程序")
        
        # 完成當前任務
        if self.current_task:
            self.get_logger().info("等待當前任務完成...")
        
        # 保存狀態
        self._backup_database()
        
        # 關閉資料庫連接
        if hasattr(self, 'db_conn'):
            self.db_conn.close()
        
        self.get_logger().info("系統關閉完成")
    
    def _signal_handler(self, signum, frame):
        """信號處理器"""
        self.get_logger().info(f"收到信號 {signum}，開始關閉系統")
        self._shutdown_system()
        exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = PharmacySystemOrchestrator()
    
    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        orchestrator.get_logger().info("收到中斷信號")
    finally:
        orchestrator._shutdown_system()
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()