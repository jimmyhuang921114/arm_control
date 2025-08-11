import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO
import cv2


class YoloPointCloudNode(Node):
    def __init__(self):
        super().__init__('yolo_pcd_node')
        self.bridge = CvBridge()
        # self.yolo = YOLO('yolov8m.pt')  # 可替換為 yolov8m.pt 等
        self.yolo = YOLO('/workspace/src/visuial/resource/best.pt')
        self.color_image = None
        self.pcd_data = None

        self.image_sub = self.create_subscription(
            Image, '/tm_robot/color_image', self.image_callback, 10)

        self.pcd_sub = self.create_subscription(
            PointCloud2, '/tm_robot/pointcloud', self.pcd_callback, 10)

        self.obj_pcd_pub = self.create_publisher(PointCloud2, '/tm_robot/object_pointcloud', 10)

        self.timer = self.create_timer(1.0 / 10.0, self.process_callback)
        self.get_logger().info("YOLO + PointCloud node started")

    def image_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.header = msg.header

    def pcd_callback(self, msg):
        self.pcd_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))

    def process_callback(self):
        if self.color_image is None or self.pcd_data is None:
            return

        results = self.yolo(self.color_image)
        boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)

        height, width, _ = self.color_image.shape
        selected_points = []

        for box in boxes:
            x1, y1, x2, y2 = box
            x1, x2 = np.clip([x1, x2], 0, width - 1)
            y1, y2 = np.clip([y1, y2], 0, height - 1)

            for v in range(y1, y2):
                for u in range(x1, x2):
                    idx = v * width + u
                    if 0 <= idx < len(self.pcd_data):
                        pt = self.pcd_data[idx]
                        if not np.isnan(pt[2]) and pt[2] < 2.0:  # 避免過遠或 NaN
                            selected_points.append(pt)

        if selected_points:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera_link"

            fields = [
                pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
            ]

            cloud_msg = pc2.create_cloud(header, fields, selected_points)
            self.obj_pcd_pub.publish(cloud_msg)
            self.get_logger().info(f"發布分割點雲，共 {len(selected_points)} 點")
        else:
            self.get_logger().info("無有效 YOLO 偵測或點雲")

def main(args=None):
    rclpy.init(args=args)
    node = YoloPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
