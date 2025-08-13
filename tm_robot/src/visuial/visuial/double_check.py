#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import socket
import threading
import time
from tm_robot_if.srv import SecondCamera
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture("/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0")

        if not self.cap.isOpened():
            self.get_logger().error('無法開啟攝影機')
            return
        else:
            self.get_logger().info('攝影機已開啟')

        # 相機參數設定
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 300.0)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 1)
        self.cap.set(cv2.CAP_PROP_GAIN, 0.1)
        self.cap.set(cv2.CAP_PROP_GAMMA, 0.4)
        self.print_camera_params()

        # 儲存資料夾
        self.capture_dir = os.path.abspath('./src/visuial/sample_picture')
        os.makedirs(self.capture_dir, exist_ok=True)

        # Frame 鎖與快取
        self.latest_frame = None
        self.lock = threading.Lock()
        self.last_capture_time = 0

        # 啟動影像捕捉執行緒
        self.read_thread = threading.Thread(target=self.read_camera_loop, daemon=True)
        self.read_thread.start()

        # 發布定時器
        self.timer = self.create_timer(0.03, self.timer_callback)

        # Service: 拍照控制
        self.double_check_srv = self.create_service(SecondCamera, 'camera2', self.capture_callback)

    def print_camera_params(self):
        param_names = [
            ("FRAME_WIDTH", cv2.CAP_PROP_FRAME_WIDTH),
            ("FRAME_HEIGHT", cv2.CAP_PROP_FRAME_HEIGHT),
            ("BRIGHTNESS", cv2.CAP_PROP_BRIGHTNESS),
            ("CONTRAST", cv2.CAP_PROP_CONTRAST),
            ("SATURATION", cv2.CAP_PROP_SATURATION),
            ("HUE", cv2.CAP_PROP_HUE),
            ("GAIN", cv2.CAP_PROP_GAIN),
            ("EXPOSURE", cv2.CAP_PROP_EXPOSURE),
            ("AUTO_EXPOSURE", cv2.CAP_PROP_AUTO_EXPOSURE),
            ("GAMMA", cv2.CAP_PROP_GAMMA),
            ("AUTO_WB", cv2.CAP_PROP_AUTO_WB),
            ("WHITE_BALANCE_BLUE_U", cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
            ("WHITE_BALANCE_RED_V", cv2.CAP_PROP_WHITE_BALANCE_RED_V),
            ("FPS", cv2.CAP_PROP_FPS),
        ]
        self.get_logger().info("=== 相機目前參數 ===")
        for name, prop in param_names:
            val = self.cap.get(prop)
            self.get_logger().info(f"{name}: {val}")

    def read_camera_loop(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                flipped = cv2.flip(frame, -1)
                with self.lock:
                    self.latest_frame = flipped.copy()
            time.sleep(0.01)

    def timer_callback(self):
        with self.lock:
            if self.latest_frame is not None:
                msg = self.bridge.cv2_to_imgmsg(self.latest_frame, encoding='bgr8')
                self.publisher.publish(msg)

    def capture_callback(self, request, response):
        # 啟動非同步 socket thread，主線程不阻塞
        socket_thread = threading.Thread(target=self.socket_handler, daemon=True)
        socket_thread.start()
        socket_thread.join()
        response.success = True
        return response

    def socket_handler(self):
        self.get_logger().info("SecondCamera service called. Waiting for 4 signals...")

        # 清除舊圖
        for i in range(1, 5):
            path = os.path.join(self.capture_dir, f"img{i}.jpg")
            if os.path.exists(path):
                os.remove(path)

        count = 0
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind(('0.0.0.0', 6001))
        except OSError as e:
            self.get_logger().error(f"Port 6001 已被佔用: {e}")
            return

        server.listen(1)
        self.get_logger().info("Socket server listening on port 6001")
        client, addr = server.accept()
        self.get_logger().info(f"Socket client connected from {addr}")
        client.settimeout(None)

        while count < 4:
            try:
                data = client.recv(1024)
            except Exception as e:
                self.get_logger().warn(f"Socket error: {e}")
                break

            if not data:
                self.get_logger().warn("Socket closed by client")
                break

            self.get_logger().info(f"Socket received: {repr(data)}")

            now = time.time()
            if b'\x01' in data and (now - self.last_capture_time) > 0.5:
                self.last_capture_time = now
                with self.lock:
                    frame = self.latest_frame.copy() if self.latest_frame is not None else None
                if frame is not None:
                    filename = os.path.join(self.capture_dir, f"img{count + 1}.jpg")
                    if cv2.imwrite(filename, frame):
                        self.get_logger().info(f"拍攝中: {count + 1}/4 -> {filename}")
                        count += 1
                    else:
                        self.get_logger().error(f"cv2.imwrite failed: {filename}")
                else:
                    self.get_logger().warn("尚未獲得最新影像")

        if count == 4:
            self.combine_image()
            try:
                client.sendall(b'FINISH\n')
            except Exception as e:
                self.get_logger().warn(f"Send FINISH failed: {e}")

        client.close()
        server.close()

    def resize_with_pad(img, target_w=1024, target_h=1024):
        """等比例縮放，不變形；不足的部分以黑色填充到指定大小。"""
        h, w = img.shape[:2]
        scale = min(target_w / w, target_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # 建立黑底畫布並把縮放後的圖置中
        canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)
        x0 = (target_w - new_w) // 2
        y0 = (target_h - new_h) // 2
        canvas[y0:y0+new_h, x0:x0+new_w] = resized
        return canvas

    def combine_image(self, tile_size=1024, darken_factor=1.0):
        """合併 4 張圖成 2x2；darken_factor < 1 會變暗，例如 0.7"""
        filenames = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg"]
        images = []
        for name in filenames:
            path = os.path.join(self.capture_dir, name)
            if not os.path.exists(path):
                self.get_logger().warn(f"{path} 不存在")
                return
            img = cv2.imread(path)
            if img is None:
                self.get_logger().warn(f"無法讀取圖片: {path}")
                return
            # 等比例縮放 + 補邊到 tile_size
            img_pad = self.resize_with_pad(img, tile_size, tile_size)
            images.append(img_pad)

        if len(images) != 4:
            self.get_logger().warn("圖片數量不足，無法合成")
            return

        top = cv2.hconcat([images[0], images[1]])
        bottom = cv2.hconcat([images[2], images[3]])
        combined = cv2.vconcat([top, bottom])  # 大小 = (2*tile_size, 2*tile_size)
        output_path = os.path.join(self.capture_dir, "combined.jpg")
        if cv2.imwrite(output_path, combined):
            self.get_logger().info(f"合併圖片儲存於: {output_path}")
        else:
            self.get_logger().error(f"cv2.imwrite 失敗：{output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()
