import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoViewerNode(Node):
    def __init__(self):
        super().__init__('aruco_viewer_node')

        self.bridge = CvBridge()

        # 訂閱 RealSense color image 該topic根據你實際使用修改
        self.subscription = self.create_subscription(
            Image,
            '/tm_robot/color_image',
            self.image_callback,
            10
        )

        # 載入 aruco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("Aruco Viewer Node initialized")

    def image_callback(self, msg):
        # 將ROS影像轉為OpenCV格式
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # 偵測 ArUco 標記
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            for corner in corners:
                # 計算中心點
                pts = corner[0]
                center_x = int(np.mean(pts[:, 0]))
                center_y = int(np.mean(pts[:, 1]))
                # 在中心畫十字
                cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0),
                               markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # 畫出畫面中心十字
        cv2.drawMarker(frame, (w // 2, h // 2), (0, 0, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # 顯示畫面
        cv2.imshow('Aruco Viewer', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
