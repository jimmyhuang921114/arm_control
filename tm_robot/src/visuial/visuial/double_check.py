#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import socket
import threading
import time
import numpy as np
from tm_robot_if.srv import SecondCamera as SecondCameraSrv

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.last_capture_time = 0.0
        self.capture_dir = os.path.abspath('./tm_robot/src/visuial/sample_picture')
        os.makedirs(self.capture_dir, exist_ok=True)

        self.create_subscription(Image, 'camera/image_raw', self.second_camera_img_callback, 10)
        self.create_service(SecondCameraSrv, 'camera2', self.capture_callback)
        self.get_logger().info('CameraPublisher ready. Subscribing /camera/image_raw; Service /camera2')

    def second_camera_img_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        with self.lock:
            self.latest_frame = frame

    def capture_callback(self, req, res):
        # 讀 request 的 pic_cnt (1=>等1張；4=>等4張；其它值一律當4)
        try:
            shots = int(req.pic_cnt)
        except Exception:
            shots = 4
        if shots != 1:
            shots = 4

        threading.Thread(target=self.socket_handler, args=(shots,), daemon=True).start()
        res.success = True
        return res

    def socket_handler(self, shots: int):
        self.get_logger().info(f"SecondCamera called. Waiting for {shots} signal(s) on port 6001...")

        # 清除舊圖
        for i in range(1, shots + 1):
            p = os.path.join(self.capture_dir, f"img{i}.jpg")
            if os.path.exists(p):
                try:
                    os.remove(p)
                except Exception as e:
                    self.get_logger().warn(f"刪除舊檔失敗 {p}: {e}")

        count = 0
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            server.bind(('0.0.0.0', 6001))
        except OSError as e:
            self.get_logger().error(f"Port 6001 被佔用: {e}")
            server.close()
            return

        server.listen(1)
        self.get_logger().info("Socket server listening on port 6001")
        try:
            client, addr = server.accept()
        except Exception as e:
            self.get_logger().error(f"accept() 失敗: {e}")
            server.close()
            return

        self.get_logger().info(f"Socket client connected from {addr}")
        client.settimeout(None)

        try:
            while count < shots:
                try:
                    data = client.recv(1024)
                except Exception as e:
                    self.get_logger().warn(f"Socket recv error: {e}")
                    break

                if not data:
                    self.get_logger().warn("Socket closed by client")
                    break

                self.get_logger().info(f"Socket received: {repr(data)}")

                now = time.time()
                # 防彈跳：0.5 秒
                if (b'\x01' in data) and (now - self.last_capture_time) > 0.5:
                    self.last_capture_time = now

                    with self.lock:
                        frame = None if self.latest_frame is None else self.latest_frame.copy()
                    if frame is None:
                        self.get_logger().warn("尚未獲得最新影像（/camera/image_raw 還沒來）")
                        continue

                    filename = os.path.join(self.capture_dir, f"img{count + 1}.jpg")
                    ok = cv2.imwrite(filename, frame)
                    if ok:
                        count += 1
                        self.get_logger().info(f"拍攝中: {count}/{shots} -> {filename}")
                    else:
                        self.get_logger().error(f"cv2.imwrite 失敗：{filename}")

            if count == shots:
                # 合成或直接輸出
                paths = [os.path.join(self.capture_dir, f"img{i}.jpg") for i in range(1, shots + 1)]
                self.combine_images(paths, tile_size=1024)
                try:
                    client.sendall(b'FINISH\n')
                except Exception as e:
                    self.get_logger().warn(f"Send FINISH failed: {e}")
        finally:
            try:
                client.close()
            except:
                pass
            server.close()

    def resize_with_pad(self, img, tile_size=1024):
        h, w = img.shape[:2]
        scale = min(tile_size / w, tile_size / h)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((tile_size, tile_size, 3), dtype=np.uint8)
        x0 = (tile_size - new_w) // 2
        y0 = (tile_size - new_h) // 2
        canvas[y0:y0+new_h, x0:x0+new_w] = resized
        return canvas

    def combine_images(self, paths, tile_size=1024):
        """
        支援：
          - 1 張：直接存成 combined.jpg（置中填滿）
          - 4 張：2x2 佈局
        """
        imgs = []
        for p in paths:
            if not os.path.exists(p):
                self.get_logger().warn(f"{p} file not exist")
                return
            img = cv2.imread(p)
            if img is None:
                self.get_logger().warn(f"can not read image: {p}")
                return
            imgs.append(self.resize_with_pad(img, tile_size))

        out = os.path.join(self.capture_dir, "combined.jpg")
        if len(imgs) == 1:
            if cv2.imwrite(out, imgs[0]):
                self.get_logger().info(f"image save at : {out} (single)")
            else:
                self.get_logger().error(f"cv2.imwrite fail {out}")
            return

        if len(imgs) == 4:
            top = cv2.hconcat([imgs[0], imgs[1]])
            bottom = cv2.hconcat([imgs[2], imgs[3]])
            combined = cv2.vconcat([top, bottom])
            if cv2.imwrite(out, combined):
                self.get_logger().info(f"image save at : {out} (2x2)")
            else:
                self.get_logger().error(f"cv2.imwrite fail {out}")
            return

        self.get_logger().warn(f"combine_images only supports 1 or 4 images (got {len(imgs)}).")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
