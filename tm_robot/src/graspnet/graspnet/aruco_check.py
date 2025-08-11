import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from cv_bridge import CvBridge
from tm_robot_if.srv import PoseSrv
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster

ARUCO_DICT = cv2.aruco.DICT_4X4_50

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.z_normal_list = []

        # 相機參數
        self.Intrinsic = np.array([
            [904.8729050868374, 0.0, 634.3937317400505],
            [0.0, 903.3201754368574, 369.0644726085734],
            [0.0, 0.0, 1.0]
        ])
        self.Distortion = np.array([
            0.14464552813304468,
            0.5821827164855585,
            0.004314151191910511,
           -0.001112447533308546,
           -2.461367058886307
        ])

        # 訂閱與發布
        self.subscription = self.create_subscription(Image, '/tm_robot/color_image', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/tool_pose_ee', 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.pose_client = self.create_client(PoseSrv, 'thing_pose')
        self.br = TransformBroadcaster(self)  # TF 廣播器

        self.marker_length = 0.05
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.get_logger().info("ArucoDetector with TF + Pose initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.Intrinsic, self.Distortion
            )
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
                R_mat, _ = cv2.Rodrigues(rvec)
                t_vec = tvec.reshape((3,))

                # 濾波 z 軸
                Rx_180 = R.from_euler('y', 180, degrees=True)
                R_rotated = R.from_matrix(R_mat @ Rx_180.as_matrix())
                self.z_normal_list.append(R_rotated.as_matrix()[:, 2])
                if len(self.z_normal_list) > 10:
                    self.z_normal_list.pop(0)

                median_z = np.median(np.array(self.z_normal_list), axis=0)
                median_z /= np.linalg.norm(median_z)

                # 使用 median_z 重建旋轉矩陣
                z_axis = median_z
                x_temp = np.array([1.0, 0.0, 0.0])
                # if np.abs(np.dot(z_axis, x_temp)) > 0.95:
                #     x_temp = np.array([0.0, 1.0, 0.0])

                y_axis = np.cross(z_axis, x_temp)
                y_axis /= np.linalg.norm(y_axis)
                x_axis = np.cross(y_axis, z_axis)
                x_axis /= np.linalg.norm(x_axis)

                R_smooth = np.stack([x_axis, y_axis, z_axis], axis=1)
                quat = R.from_matrix(R_smooth).as_quat()

                # 發布 Pose
                pose_cam = PoseStamped()
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.header.frame_id = 'camera_color_optical_frame'
                pose_cam.pose.position = Point(x=t_vec[0], y=t_vec[1], z=t_vec[2])
                pose_cam.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                self.pose_pub.publish(pose_cam)

                # 發布 TF
                aruco_id = int(ids[i][0])
                self.broadcast_tf(pose_cam, f"aruco_{aruco_id}")

                # 顯示資訊
                cv2.drawFrameAxes(frame, self.Intrinsic, self.Distortion, rvec, tvec, self.marker_length * 1.5)
                cv2.putText(frame, f"ID: {aruco_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1])-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 傳送至 service
                if not self.pose_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error("Service 'thing_pose' not available")
                    return

                req = PoseSrv.Request()
                req.pose = pose_cam.pose
                future = self.pose_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)

                if future.result() and future.result().success:
                    self.get_logger().info(f"Pose sent to thing_pose service (ID: {aruco_id})")
                else:
                    self.get_logger().error(f"Failed to send pose for ArUco ID {aruco_id}")

        # 顯示影像
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)
        # cv2.imshow("Aruco Detection", frame)
        # cv2.waitKey(1)

    def broadcast_tf(self, pose: PoseStamped, child_frame_id: str):
        t = TransformStamped()
        t.header.stamp = pose.header.stamp
        t.header.frame_id = pose.header.frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
