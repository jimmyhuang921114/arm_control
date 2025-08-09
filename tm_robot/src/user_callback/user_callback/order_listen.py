#!/usr/bin/env python3
import os
import time
import json
import yaml
import rclpy
import threading
from typing import Dict, Any, List, Tuple
from queue import Queue, Empty
from rclpy.node import Node
from std_msgs.msg import String
from tm_robot_if.srv import MedicineOrder  # Request.command 放 YAML

# 可選：HTTP 回報/拉單
try:
    import requests
except Exception:
    requests = None


class OrderHandlerNode(Node):
    """
    特性：
    - /hospital/medicine_order ：ROS2 Service（push 模式）
    - 每隔 N 秒自動輪詢 ORDER_PULL_URL（pull 模式）
      * 若正在忙或佇列非空，本次輪詢直接跳過（直到下一次再問）
    - /hospital/order_status ：Topic 廣播進度
    - （可選）HTTP 回報：進度與完成
    """

    def __init__(self):
        super().__init__('order_handler')

        # ===== HTTP/輪詢設定 =====
        self.base_url = os.getenv('ORDER_BASE_URL', 'http://127.0.0.1:8000')
        self.pull_url = os.getenv('ORDER_PULL_URL', f"{self.base_url.rstrip('/')}/api/order/next")
        self.pull_interval = float(os.getenv('ORDER_PULL_INTERVAL', '3'))  # 每幾秒輪詢一次
        self.http_timeout = float(os.getenv('ORDER_HTTP_TIMEOUT', '5'))
        self.http_retry = int(os.getenv('ORDER_HTTP_RETRY', '2'))
        self.progress_path = os.getenv('ORDER_PROGRESS_PATH', '/api/order/progress')
        self.complete_path = os.getenv('ORDER_COMPLETE_PATH', '/api/order/complete')
        self.auth_token = os.getenv('ORDER_HTTP_TOKEN', '')

        if requests is None:
            self.get_logger().warn("requests 不可用：將無法 HTTP 拉單/回報（只用 ROS 端功能）。")

        # ===== ROS 介面 =====
        self.order_srv = self.create_service(
            MedicineOrder, '/hospital/medicine_order', self._order_service_cb
        )
        self.status_pub = self.create_publisher(String, '/hospital/order_status', 10)

        # ===== 佇列 + 背景單工 worker =====
        self._queue: "Queue[Tuple[Dict[str, Any], str]]" = Queue()
        self._busy = False
        self._lock = threading.Lock()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # ===== 定時輪詢（pull）=====
        # 注意：我們不會在忙或佇列非空時真的去打 API（函式裡會 return）
        self._pull_timer = self.create_timer(self.pull_interval, self._poll_once)

        self.get_logger().info("✅ OrderHandlerNode ready. Push/Pull 皆可用。")

    # ===================== Service（Push） =====================
    def _order_service_cb(self, request: MedicineOrder.Request, response: MedicineOrder.Response):
        yaml_text = request.command or ''
        try:
            order = yaml.safe_load(yaml_text)
            if not isinstance(order, dict): raise ValueError("YAML 不是 dict")
            if 'order_id' not in order: raise ValueError("缺少 order_id")
            if 'medicine' not in order or not isinstance(order['medicine'], list):
                raise ValueError("缺少 medicine list 或格式錯誤")

            self._queue.put((order, yaml_text))
            qsize = self._queue.qsize()
            msg = f"📥 收到訂單 {order['order_id']}（佇列長度={qsize}）"
            self._pub_status(msg)
            self._report_progress(order['order_id'], "queued", msg)

            response.success = True
            response.message = f"Accepted. Queue size={qsize}"
        except Exception as e:
            err = f"❌ 解析失敗：{e}"
            self.get_logger().error(err)
            response.success = False
            response.message = err
        return response

    # ===================== Timer Poll（Pull） =====================
    def _poll_once(self):
        """每次計時器觸發：若空閒且佇列空，嘗試去拉一筆新訂單；否則直接跳過。"""
        # 忙就跳過
        with self._lock:
            if self._busy:
                return
        # 佇列有東西也跳過（一次一單，避免一次拉多單）
        if not self._queue.empty():
            return
        # 沒有 requests 就不用拉
        if requests is None:
            return

        try:
            r = requests.get(self.pull_url, headers=self._headers(), timeout=self.http_timeout)
            if r.status_code == 204 or not r.text.strip():
                # 沒新單
                return
            r.raise_for_status()

            # 可能回 JSON 或純 YAML
            data = None
            ctype = r.headers.get('content-type', '')
            if 'application/json' in ctype:
                data = r.json()

            if data and isinstance(data, dict) and 'order' in data:
                order = data['order']
                yaml_text = yaml.safe_dump(order, allow_unicode=True)
            elif data and isinstance(data, dict) and 'yaml' in data:
                yaml_text = data['yaml']
                order = yaml.safe_load(yaml_text)
            else:
                yaml_text = r.text
                order = yaml.safe_load(yaml_text)

            if not isinstance(order, dict) or 'order_id' not in order or 'medicine' not in order:
                self.get_logger().warn("拉到的訂單格式不正確，略過")
                return

            self._queue.put((order, yaml_text))
            self._pub_status(f"🛰️ 拉到新訂單：{order['order_id']}（已入佇列）")
            self._report_progress(order['order_id'], "queued", "pulled and queued")
        except Exception as e:
            # 輪詢錯誤不影響主流程，留待下次 timer 再試
            self.get_logger().warn(f"拉單失敗：{e}")

    # ===================== Worker（一次一單） =====================
    def _worker_loop(self):
        while rclpy.ok():
            try:
                order, yaml_text = self._queue.get(timeout=0.25)
            except Empty:
                continue

            with self._lock:
                self._busy = True

            try:
                self._handle_order(order, yaml_text)
            except Exception as e:
                oid = order.get('order_id', '?')
                self.get_logger().exception(f"處理訂單 {oid} 例外")
                self._pub_status(f"❌ 訂單 {oid} 處理失敗：{e}")
                self._report_complete(oid, "failed", str(e))
            finally:
                with self._lock:
                    self._busy = False
                self._queue.task_done()

    # ===================== Main Logic =====================
    def _handle_order(self, order: Dict[str, Any], yaml_text: str):
        oid = order.get('order_id', 'unknown')
        patient = order.get('patient_name', 'unknown')
        meds: List[Dict[str, Any]] = order.get('medicine', [])

        self._pub_status(f"🚀 開始處理訂單 {oid}（病患：{patient}，品項數：{len(meds)}）")
        self._report_progress(oid, "started", "order started")
        self._print_order(order, yaml_text)

        for i, med in enumerate(meds, 1):
            self._process_medicine(oid, med, i, len(meds))

        self._pub_status(f"✅ 訂單 {oid} 完成")
        self._report_complete(oid, "success", "order finished")

    def _print_order(self, order: Dict[str, Any], yaml_text: str):
        self.get_logger().info("=" * 80)
        self.get_logger().info("📋 訂單 YAML：")
        for line in yaml_text.splitlines():
            if line.strip():
                self.get_logger().info("   " + line)
        self.get_logger().info("-" * 80)
        self.get_logger().info(f"🆔 order_id: {order.get('order_id','N/A')}")
        self.get_logger().info(f"👤 patient_name: {order.get('patient_name','N/A')}")
        meds = order.get('medicine', [])
        self.get_logger().info(f"💊 medicine items: {len(meds)}")
        for i, m in enumerate(meds, 1):
            self.get_logger().info(f"   {i}. "
                                   f"name={m.get('name','N/A')} "
                                   f"amount={m.get('amount',0)} "
                                   f"locate={m.get('locate',[0,0])} "
                                   f"prompt={m.get('prompt','N/A')}")
        self.get_logger().info("=" * 80)

    def _process_medicine(self, order_id: str, med: Dict[str, Any], idx: int, total: int):
        name = med.get('name', 'N/A')
        amount = med.get('amount', 0)
        locate = med.get('locate', [0, 0])
        prompt = med.get('prompt', 'unknown')

        stage_msg = f"({idx}/{total}) 處理 {name} | 數量={amount} | 位置={locate} | 類型={prompt}"
        self._pub_status("🔧 " + stage_msg)
        self._report_progress(order_id, "processing", stage_msg, item=name, index=idx, total=total)

        # ===== 在這裡串你的實際機器人流程 =====
        # self.send_tm_flow_mode_and_spin(...)
        # bbox, mask = self.call_groundsam2_and_spin(...)
        # pose = self.estimate_grab_point(...)
        # self.send_curobo_pose_and_spin(pose)
        # self.call_second_camera_and_spin(...)
        # ok = self.call_llm_and_spin(name)
        # 失敗重試/超時保護自行加上

        time.sleep(1)  # demo 延時（可刪）

        done = f"{name} 完成"
        self._pub_status(f"✅ {done}")
        self._report_progress(order_id, "item_done", done, item=name, index=idx, total=total)

    # ===================== HTTP Utils =====================
    def _headers(self) -> Dict[str, str]:
        h = {"Content-Type": "application/json"}
        if self.auth_token:
            h["Authorization"] = f"Bearer {self.auth_token}"
        return h

    def _post_with_retry(self, url: str, payload: Dict[str, Any]) -> bool:
        if requests is None:
            self.get_logger().warn("requests 不可用：略過 HTTP 回報")
            return False
        for attempt in range(1, self.http_retry + 1):
            try:
                r = requests.post(url, headers=self._headers(), data=json.dumps(payload), timeout=self.http_timeout)
                if 200 <= r.status_code < 300:
                    return True
                self.get_logger().warn(f"HTTP {r.status_code}: {r.text}（第{attempt}/{self.http_retry}次）")
            except Exception as e:
                self.get_logger().warn(f"HTTP 例外：{e}（第{attempt}/{self.http_retry}次）")
            time.sleep(min(0.5 * attempt, 3.0))
        return False

    def _report_progress(self, order_id: str, stage: str, message: str,
                         item: str = "", index: int = 0, total: int = 0):
        payload = {
            "order_id": order_id,
            "stage": stage,           # queued / started / processing / item_done
            "message": message,
            "item": item,
            "index": index,
            "total": total,
            "ts": time.time(),
        }
        self.status_pub.publish(String(data=f"[{stage}] {message}"))
        url = self.base_url.rstrip('/') + self.progress_path
        if not self._post_with_retry(url, payload):
            # 不阻斷流程
            pass

    def _report_complete(self, order_id: str, status: str, details: str = ""):
        payload = {
            "order_id": order_id,
            "status": status,         # success / failed
            "details": details,
            "ts": time.time(),
        }
        url = self.base_url.rstrip('/') + self.complete_path
        if not self._post_with_retry(url, payload):
            # 不阻斷流程
            pass

    # ===================== Misc =====================
    def _pub_status(self, text: str):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def is_busy(self) -> bool:
        with self._lock:
            return self._busy


def main(args=None):
    rclpy.init(args=args)
    node = OrderHandlerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
