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
        self.declare_parameter('distance_threshold', 0.001)
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
        self.K = np.array([[905.5264861373364, 0.0, 633.5127254288533],
                           [0.0, 904.4704173255083, 363.7238756445801],
                           [0.0, 0.0, 1.0]])
        
         
        self.dist_coeffs = np.array([
            0.1457895796758955,
            -0.3161928425604433,
            0.0020954680239515847,
            -0.001678205668629361,
            0.03480936976818285
        ])
        self.intrinsic = {
            'fx': 905.5264861373364,
            'fy': 904.4704173255083,
            'cx': 633.5127254288533,
            'cy': 363.7238756445801
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
            # 假設 depth_msg.encoding == '16UC1' (mm)
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_m = depth_mm.astype(np.float32) * 0.001
            self.latest_depth = depth_m
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
            self.bbox = request.bbox
            self.fit_plane_from_bbox()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"mask 轉換失敗: {e}")
            response.success = False
        return response

    def fit_plane_from_bbox(self):
        if self.latest_depth is None or self.bbox is None:
            self.get_logger().warn("缺少 depth 或 bbox，無法執行平面擬合")
            return

        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']

        xmin, ymin, xmax, ymax = map(int, self.bbox)

        # region_pts = []
        # for v in range(ymin, ymax):
        #     for u in range(xmin, xmax):
        #         if self.latest_mask is not None and self.latest_mask[v, u] != 255:
        #             continue
        #         z = self.latest_depth[v, u]
        #         if z <= 0 or np.isnan(z):
        #             continue
        #         x = (u - cx) * z / fx
        #         y = (v - cy) * z / fy
        #         region_pts.append([x, y, z])
        camera_matrix = self.K.astype(np.float32)
        dist_coeffs = self.dist_coeffs.astype(np.float32)

        region_pts = []
        for v in range(ymin, ymax):
            for u in range(xmin, xmax):
                if self.latest_mask is not None and self.latest_mask[v, u] != 255:
                    continue
                z = self.latest_depth[v, u]
                if z <= 0 or np.isnan(z):
                    continue

                # --- 去畸變 ---
                pts = np.array([[[u, v]]], dtype=np.float32)
                undistorted = cv2.undistortPoints(pts, camera_matrix, dist_coeffs, P=camera_matrix)
                u_nd, v_nd = undistorted[0, 0]

                x = (u_nd - cx) * z / fx
                y = (v_nd - cy) * z / fy
                region_pts.append([x, y, z])


        # ==============================================================
        pts = np.array(region_pts, dtype=np.float32)
        # 取 z 欄的 50 分位做門檻
        z_vals = pts[:, 2]
        z_thresh = np.percentile(z_vals, 50)
        # 過濾出 z <= 門檻（後 50%）
        bottom50_pts = pts[z_vals <= z_thresh]
        if bottom50_pts.shape[0] < 50:
            self.get_logger().warn(f"篩選後點雲數量不足: {bottom50_pts.shape[0]}")
            return
        # 更新 region_pts 為後 50%
        region_pts = bottom50_pts.tolist()
        # ==============================================================


        if len(region_pts) < 50:
            self.get_logger().warn(f"ROI 點雲數量不足: {len(region_pts)}")
            return

        # 建立 Open3D 點雲
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(region_pts)

        # 單一統計濾波
        pcd_filtered, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        normals = []
        centers = []
        last_inliers = []

        z_axis = np.array([0, 0, 1], dtype=np.float32)

        # model, inliers = pcd_filtered.segment_plane(
        #         distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
        #         ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
        #         num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
        #     )
        # [a, b, c, d] = model
        # inlier_cloud = pcd_filtered.select_by_index(inliers)
        # inlier_points = np.asarray(inlier_cloud.points)
        # center = np.median(inlier_points, axis=0)
        # normal = np.array([a, b, c], dtype=np.float32)
        # normal /= np.linalg.norm(normal)

        best_model = None
        best_inliers = []
        max_inlier_count = 0
        best_center = None
        best_normal = None

        for i in range(10):
            model, inliers = pcd_filtered.segment_plane(
                distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
                ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
                num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
            )

            if len(inliers) > max_inlier_count:
                max_inlier_count = len(inliers)
                best_model = model
                best_inliers = inliers

        # 沒有找到平面
        if best_model is None or len(best_inliers) < 50:
            self.get_logger().warn("無法找到足夠 inlier 的平面")
            return

        # 解析最佳平面
        [a, b, c, d] = best_model
        normal = np.array([a, b, c], dtype=np.float32)
        normal /= np.linalg.norm(normal)


        camera_z = np.array([0, 0, 1], dtype=np.float32)
        angle_deg = np.rad2deg(np.arccos(np.clip(np.dot(normal, camera_z), -1.0, 1.0)))
        self.get_logger().info(f"平面法向量與相機 z 軸夾角: {angle_deg:.2f}°")

        MAX_ANGLE_THRESHOLD = 20.0  # 可自行調整允許最大夾角
        if angle_deg > MAX_ANGLE_THRESHOLD:
            self.get_logger().warn(f"法向量夾角超過 {MAX_ANGLE_THRESHOLD}°，忽略此平面")
            # return


        inlier_cloud = pcd_filtered.select_by_index(best_inliers)
        inlier_points = np.asarray(inlier_cloud.points)
        center = np.median(inlier_points, axis=0)


        # 畫出 inlier mask
        mask_img = np.zeros(self.latest_depth.shape, dtype=np.uint8)
        inlier_cloud_final = pcd_filtered.select_by_index(best_inliers)
        inlier_points_final = np.asarray(inlier_cloud_final.points)
        for x, y, z in inlier_points_final:
            if z <= 0 or not np.isfinite(z):
                continue
            u = int((x * fx) / z + cx)
            v = int((y * fy) / z + cy)
            if 0 <= u < mask_img.shape[1] and 0 <= v < mask_img.shape[0]:
                mask_img[v, u] = 255
        cv2.imshow("Inlier Mask", mask_img)
        cv2.waitKey(1)

        # 建立姿態
        x_ref = np.array([-1.0, 0.0, 0.0])
        y_axis = np.cross(normal, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        x_axis /= np.linalg.norm(x_axis)
        rot_matrix = np.stack([x_axis, y_axis, normal], axis=1)
        quat = R.from_matrix(rot_matrix).as_quat()
        center[2] = center[2]
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

        marker = Marker()
        marker.header.frame_id = 'camera_depth_optical_frame'
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

        # 呼叫 PoseSrv
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