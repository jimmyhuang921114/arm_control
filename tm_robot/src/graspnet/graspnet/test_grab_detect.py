#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, PointField
from geometry_msgs.msg import Point, Vector3, PoseStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import open3d as o3d
import cv2
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from tm_robot_if.srv import PoseSrv, CaptureImage

class PlaneFittingNode(Node):
    def __init__(self):
        super().__init__('plane_fitting_node')

        # Parameters
        self.declare_parameter('distance_threshold', 0.0001)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 1000)

        # Subscribers
        self.sub_pc = self.create_subscription(PointCloud2, '/tm_robot/pointcloud', self.pc_callback, 10)
        self.sub_color = self.create_subscription(Image, '/tm_robot/color_image', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/tm_robot/depth_image', self.depth_callback, 10)
        # Publisher / Services
        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/debug_pcd', 10)
        self.pub_marker = self.create_publisher(Marker, 'plane_marker', 10)
        self.pub_center = self.create_publisher(Point, 'plane_center', 10)
        self.pub_normal = self.create_publisher(Vector3, 'plane_normal', 10)
        self.pose_client = self.create_client(PoseSrv, 'thing_pose')
        self.create_service(CaptureImage, 'grab_detect', self.handle_mask_service)

        # Intrinsics
        self.K = np.array([[904.87290509, 0.0, 634.39373174],
                           [0.0, 903.32017544, 369.06447261],
                           [0.0, 0.0, 1.0]])

        self.dist_coeffs = np.array([
            0.05773164354848198,
            0.5821827164855585,
            0.004314151191910511,
            -0.001112447533308546,
            -2.461367058886307
        ])

        self.intrinsic = {
            'fx': 904.8729050868374,
            'fy': 903.3201754368574,
            'cx': 634.3937317400505,
            'cy': 369.0644726085734
        }

        # Latest buffers
        self.latest_depth = None
        self.latest_mask = None
        self.latest_image = None
        self.latest_pc = None
        self.sending = False
        self.pose_sent = False

    def pc_callback(self, msg):
        self.latest_pc = msg

    def color_callback(self, msg):
        self.latest_image = msg

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"深度影像轉換失敗: {e}")
            self.latest_depth = None


    def pubish_pcd(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_depth_optical_frame"
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = pc2.create_cloud(header, fields, points)
        self.pcd_pub.publish(pc_msg)

    def handle_mask_service(self, request, response):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(request.mask, desired_encoding='mono8')
            self.get_logger().info("成功接收到 binary mask，開始估算抓取姿態")
            self.plane_fitting(self.latest_pc)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"mask 轉換失敗: {e}")
            response.success = False
        return response

    def plane_fitting(self, pc_msg):
        if self.latest_depth is None or self.latest_mask is None or pc_msg is None:
            self.get_logger().warn("缺少必要資料，無法執行平面擬合")
            return

        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']

        # 解析 mask 對應點雲
        points = []
        height, width = self.latest_mask.shape
        for v in range(height):
            for u in range(width):
                if self.latest_mask[v, u] == 0:
                    continue
                z = self.latest_depth[v, u] * 0.001
                if z == 0 or np.isnan(z):
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])

        points = np.array(points, dtype=np.float32)
        if len(points) < 50:
            self.get_logger().warn('點數不足，無法進行平面擬合')
            return

        # RANSAC 擬合平面
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        model, inliers = pcd.segment_plane(
            distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
            ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
            num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
        )

        [a, b, c, d] = model
        normal = np.array([a, b, c], dtype=np.float32)
        normal /= np.linalg.norm(normal)
        center = np.mean(np.asarray(pcd.select_by_index(inliers).points), axis=0)

        # 建立姿態
        x_ref = np.array([-1.0, 0.0, 0.0])
        y_axis = np.cross(normal, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        x_axis /= np.linalg.norm(x_axis)
        rot_matrix = np.stack([x_axis, y_axis, normal], axis=1)
        quat = R.from_matrix(rot_matrix).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'camera_depth_optical_frame'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position = Point(x=center[0], y=center[1], z=center[2])
        pose_msg.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.pose_pub.publish(pose_msg)

        tf_msg = TransformStamped()
        tf_msg.header = pose_msg.header
        tf_msg.child_frame_id = 'grasp_pose'
        tf_msg.transform.translation = Vector3(x=center[0], y=center[1], z=center[2])
        tf_msg.transform.rotation = pose_msg.pose.orientation
        self.br.sendTransform(tf_msg)

        # Marker and Vector3
        marker = Marker()
        marker.header = pc_msg.header
        marker.ns = 'fitted_plane'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [
            Point(x=center[0], y=center[1], z=center[2]),
            Point(x=center[0] + normal[0]*0.1, y=center[1] + normal[1]*0.1, z=center[2] + normal[2]*0.1)
        ]
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_marker.publish(marker)

        self.pub_center.publish(Point(x=float(center[0]), y=float(center[1]), z=float(center[2])))
        self.pub_normal.publish(Vector3(x=float(normal[0]), y=float(normal[1]), z=float(normal[2])))

        # Call PoseSrv
        req = PoseSrv.Request()
        req.pose = pose_msg.pose
        self.sending = True
        future = self.pose_client.call_async(req)

        def handle_result(fut):
            self.sending = False
            try:
                result = fut.result()
                if result.success:
                    self.get_logger().info("Pose sent to service successfully.")
                    self.pose_sent = True
                else:
                    self.get_logger().error("PoseSrv returned failure.")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(handle_result)

def main(args=None):
    rclpy.init(args=args)
    node = PlaneFittingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
