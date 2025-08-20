#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import socket
import time
import numpy as np
import threading
from tm_robot_if.srv import SecondCamera as SecondCameraSrv

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # ---- concurrency ----
        self.cb_group = ReentrantCallbackGroup()
        self.busy_lock = threading.Lock()
        self.done_event = threading.Event()
        self.session_result = False

        # ---- state ----
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_capture_time = 0.0

        # ---- io ----
        self.capture_dir = os.path.abspath('./tm_robot/src/visuial/sample_picture')
        os.makedirs(self.capture_dir, exist_ok=True)

        # ---- ROS IO ----
        self.create_subscription(
            Image, 'camera/image_raw',
            self.second_camera_img_callback, 10,
            callback_group=self.cb_group
        )
        self.create_service(
            SecondCameraSrv, 'camera2',
            self.capture_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info('CameraPublisher ready. Subscribing /camera/image_raw; Service /camera2')

    def second_camera_img_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_frame = frame
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")

    # === Service: 開 thread 跑拍照流程，等事件完成才回傳 ===
    def capture_callback(self, req, res):
        shots = 1 if getattr(req, 'pic_cnt', 4) == 1 else 4

        if not self.busy_lock.acquire(blocking=False):
            self.get_logger().warn("上一個拍攝流程尚未結束，拒絕新的請求")
            res.success = False
            return res

        self.done_event.clear()
        self.session_result = False

        t = threading.Thread(
            target=self.socket_session_thread,
            args=(shots, 6001, 30.0, 0.5, 30.0),
            daemon=True
        )
        t.start()

        # 這裡等待 thread 完成；因為 executor 是 MultiThreaded，影像訂閱仍會執行
        self.get_logger().info(f"等待拍攝流程完成（shots={shots}）...")
        self.done_event.wait()  # 等 thread 設定完成事件
        res.success = bool(self.session_result)
        self.busy_lock.release()
        return res

    # === 背景 thread：完整 socket + 存圖 + 合成 ===
    def socket_session_thread(self,
                              shots: int,
                              port: int,
                              connect_timeout: float,
                              per_shot_debounce: float,
                              recv_timeout: float):
        ok = False
        try:
            ok = self._capture_session(shots, port, connect_timeout, per_shot_debounce, recv_timeout)
        except Exception as e:
            self.get_logger().error(f"socket_session_thread 例外：{e}")
            ok = False
        finally:
            self.session_result = ok
            self.done_event.set()

    # === 實際拍攝流程（阻塞在 thread 中，不占用 ROS executor 主執行緒）===
    def _capture_session(self,
                         shots: int,
                         port: int,
                         connect_timeout: float,
                         per_shot_debounce: float,
                         recv_timeout: float) -> bool:
        self.get_logger().info(f"SecondCamera called. Waiting for {shots} signal(s) on port {port}...")

        # 清理舊檔（只清 1..shots）
        for i in range(1, shots + 1):
            p = os.path.join(self.capture_dir, f"img{i}.jpg")
            try:
                if os.path.exists(p):
                    os.remove(p)
            except Exception as e:
                self.get_logger().warn(f"刪除舊檔失敗 {p}: {e}")

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        client = None
        try:
            try:
                server.bind(('0.0.0.0', port))
            except OSError as e:
                self.get_logger().error(f"Port {port} 被佔用: {e}")
                return False

            server.listen(1)
            server.settimeout(connect_timeout)
            self.get_logger().info(f"Socket server listening on port {port}")

            try:
                client, addr = server.accept()
            except socket.timeout:
                self.get_logger().error(f"等待連線逾時（{connect_timeout}s）")
                return False
            except Exception as e:
                self.get_logger().error(f"accept() 失敗: {e}")
                return False

            self.get_logger().info(f"Socket client connected from {addr}")
            client.settimeout(recv_timeout)

            count = 0
            while count < shots:
                try:
                    data = client.recv(1024)
                except socket.timeout:
                    self.get_logger().error(f"接收訊號逾時（{recv_timeout}s）")
                    return False
                except Exception as e:
                    self.get_logger().warn(f"Socket recv error: {e}")
                    return False

                if not data:
                    self.get_logger().warn("Socket closed by client")
                    return False

                self.get_logger().info(f"Socket received: {repr(data)}")

                now = time.time()
                if (b'\x01' in data) and (now - self.last_capture_time) > per_shot_debounce:
                    self.last_capture_time = now

                    with self.frame_lock:
                        frame = None if self.latest_frame is None else self.latest_frame.copy()

                    if frame is None:
                        self.get_logger().warn("尚未獲得最新影像（/camera/image_raw 還沒來），忽略此次觸發")
                        continue

                    filename = os.path.join(self.capture_dir, f"img{count + 1}.jpg")
                    if cv2.imwrite(filename, frame):
                        count += 1
                        self.get_logger().info(f"拍攝中: {count}/{shots} -> {filename}")
                    else:
                        self.get_logger().error(f"cv2.imwrite 失敗：{filename}")
                        return False

            # 合成或單張輸出
            paths = [os.path.join(self.capture_dir, f"img{i}.jpg") for i in range(1, shots + 1)]
            if not self.combine_images(paths, tile_size=1024):
                return False

            # 告知 client 完成（best-effort）
            try:
                client.sendall(b'FINISH\n')
            except Exception as e:
                self.get_logger().warn(f"Send FINISH failed: {e}")

            self.get_logger().info("SecondCamera 完成")
            return True

        finally:
            try:
                if client is not None:
                    client.close()
            except Exception:
                pass
            try:
                server.close()
            except Exception:
                pass

    def resize_with_pad(self, img, tile_size=1024):
        h, w = img.shape[:2]
        scale = min(tile_size / max(1, w), tile_size / max(1, h))
        new_w = int(max(1, w * scale))
        new_h = int(max(1, h * scale))
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((tile_size, tile_size, 3), dtype=np.uint8)
        x0 = (tile_size - new_w) // 2
        y0 = (tile_size - new_h) // 2
        canvas[y0:y0+new_h, x0:x0+new_w] = resized
        return canvas

    def combine_images(self, paths, tile_size=1024) -> bool:
        imgs = []
        for p in paths:
            if not os.path.exists(p):
                self.get_logger().warn(f"{p} file not exist")
                return False
            img = cv2.imread(p)
            if img is None:
                self.get_logger().warn(f"can not read image: {p}")
                return False
            imgs.append(self.resize_with_pad(img, tile_size))

        out = os.path.join(self.capture_dir, "combined.jpg")

        if len(imgs) == 1:
            if cv2.imwrite(out, imgs[0]):
                self.get_logger().info(f"image save at : {out} (single)")
                return True
            self.get_logger().error(f"cv2.imwrite fail {out}")
            return False

        if len(imgs) == 4:
            try:
                top = cv2.hconcat([imgs[0], imgs[1]])
                bottom = cv2.hconcat([imgs[2], imgs[3]])
                combined = cv2.vconcat([top, bottom])
            except Exception as e:
                self.get_logger().error(f"圖像合成失敗：{e}")
                return False

            if cv2.imwrite(out, combined):
                self.get_logger().info(f"image save at : {out} (2x2)")
                return True
            self.get_logger().error(f"cv2.imwrite fail {out}")
            return False

        self.get_logger().warn(f"combine_images only supports 1 or 4 images (got {len(imgs)}).")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
