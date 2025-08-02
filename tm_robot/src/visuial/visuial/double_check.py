import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import socket
from tm_robot_if.srv import SecondCamera

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)

        # ====== 新增相機參數設定 ======
        if self.cap.isOpened():
            # 關閉自動曝光 (根據相機驅動，1=手動，3=自動)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3.0)
            # 設定曝光值（0~5000），值越小越暗，依你的需求設定
            self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.90)  # 注意：OpenCV 有時是負值，實際看驅動
            # 調整亮度（範圍依攝影機而異）
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.8)
            # 增益（Gain）
            self.cap.set(cv2.CAP_PROP_GAIN, 0.0)
            # Gamma（若支援）
            self.cap.set(cv2.CAP_PROP_GAMMA, 0.4)
        # ==============================

        self.frame_ready = False
        self.latest_frame = None
        self.count = 0
        self.capture_dir = os.path.abspath('./src/second_camera/sample_picture/')
        os.makedirs(self.capture_dir, exist_ok=True)

        self.double_check_srv = self.create_service(SecondCamera, 'camera2', self.capture_callback)
        self.timer = self.create_timer(0.03, self.timer_callback)

        if not self.cap.isOpened():
            self.get_logger().error('無法開啟攝影機')
        else:
            self.get_logger().info('攝影機已開啟')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            flipped = cv2.flip(frame, 0)
            self.latest_frame = flipped.copy()
            self.frame_ready = True
            msg = self.bridge.cv2_to_imgmsg(flipped, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            self.get_logger().warn('讀取影像失敗')

    def capture_callback(self, request, response):
        self.get_logger().info("SecondCamera service called. Waiting for 4 signals...")

        count = 0
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(('0.0.0.0', 6000))
        server.listen(1)
        self.get_logger().info("Socket server listening on port 6000")

        client, addr = server.accept()
        self.get_logger().info(f"Socket client connected from {addr}")

        while count < 4:
            data = client.recv(1024)
            if not data:
                break
            msg = data.strip().lower()
            if data == b'\x01' or msg == b'true':
                self.get_logger().info(f"Received TRUE signal {count+1}/4")
                if self.frame_ready and self.latest_frame is not None:
                    filename = os.path.join(self.capture_dir, f"img{count+1}.jpg")
                    if cv2.imwrite(filename, self.latest_frame):
                        self.get_logger().info(f"Saved image: {filename}")
                    else:
                        self.get_logger().error(f"Failed to save image: {filename}")
                    count += 1
                else:
                    self.get_logger().warn("Frame not ready")
            else:
                self.get_logger().warn(f"Unrecognized message: {data}")

        client.close()
        server.close()

        self.combine_image()

        self.get_logger().info("Done capturing and combining images.")
        response.success = True
        return response

    def combine_image(self):
        filenames = ["img1.jpg", "img2.jpg", "img3.jpg", "img4.jpg"]
        images = []
        for name in filenames:
            path = os.path.join(self.capture_dir, name)
            img = cv2.imread(path)
            if img is None:
                self.get_logger().warn(f"Failed to read image {path}")
                return
            img = cv2.resize(img, (1024, 1024))
            images.append(img)

        if len(images) != 4:
            self.get_logger().warn("Missing images. Cannot combine.")
            return

        top = cv2.hconcat([images[0], images[1]])
        bottom = cv2.hconcat([images[2], images[3]])
        combined = cv2.vconcat([top, bottom])

        output_path = os.path.join(self.capture_dir, "combined.jpg")
        cv2.imwrite(output_path, combined)
        self.get_logger().info(f"Combined image saved to {output_path}")

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
