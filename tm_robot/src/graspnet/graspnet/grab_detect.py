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
        self.declare_parameter('distance_threshold', 0.005)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 2000)

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
        self.intrinsic = {
            'fx': 904.9848095391405,
            'fy': 903.7175291660693,
            'cx': 633.8065528892273,
            'cy': 363.5218153379028
        }

        self.K = np.array([[self.intrinsic['fx'], 0.0, self.intrinsic['cx']],
                           [0.0, self.intrinsic['fy'], self.intrinsic['cy']],
                           [0.0, 0.0, 1.0]])
        
         
        self.dist_coeffs = np.array([
            0.14787459074090117,
            -0.3418279021458157,
            0.002034687037960633,
            -0.0014113664491230658,
            0.09191879730682097
        ])

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
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_m = depth_mm.astype(np.float32) * 0.001
            self.latest_depth = depth_m
            # depth_normalized = cv2.normalize(depth_mm, None, 0, 255, cv2.NORM_MINMAX)
            # depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            # depth_show = depth_mm * 0.001
            # print(depth_mm.shape)
            # print(depth_mm.dtype)
            img_clipped = np.clip(depth_mm / 2, 0, 255)
            self.depth_img_showable = img_clipped.astype(np.uint8)
            # cv2.imshow("Depth Image", self.depth_img_showable)
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
            self.shelf_level = request.shelf_level
            self.fit_plane_from_bbox()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"mask 轉換失敗: {e}")
            response.success = False
        return response

    def fit_plane_from_bbox(self):
        if self.latest_mask is not None:
            ys, xs = np.where(self.latest_mask > 0)
            if len(xs) > 0 and len(ys) > 0:
                mask_center = (int(np.mean(xs)), int(np.mean(ys)))
                # cv2.circle(combined_mask, mask_center, 5, (0, 255, 0), -1)
                # self.get_logger().info(f"Mask center: {mask_center}")
            else:
                self.get_logger().warn("mask 沒有有效像素，無法計算中心點")


        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']

        xmin, ymin, xmax, ymax = map(int, self.bbox)
        bbox_center_uv = [int((xmin + xmax) / 2), int((ymin + ymax) / 2)]
            

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
                # if self.latest_mask is not None and self.latest_mask[v, u] != 255:
                #     continue
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
        z_thresh = np.percentile(z_vals,60)
        self.get_logger().warn(f"z_thresh: {z_thresh}")
        z_thresh += 0.00
        # 過濾出 z <= 門檻（後 50%）
        bottom50_pts = pts[z_vals <= z_thresh]          
        if bottom50_pts.shape[0] < 50:
            self.get_logger().warn(f"篩選後點雲數量不足: {bottom50_pts.shape[0]}")
            return
        # 更新 region_pts 為後 50%
        region_pts = bottom50_pts.tolist()

        # 取 bbox 內有效點後：
        # pts = np.asarray(region_pts, dtype=np.float32)
        # z = pts[:, 2].astype(np.float32)

        # # 1) 先分群：近 / 遠
        # z1 = z.reshape(-1, 1)
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
        # _compact, labels, centers = cv2.kmeans(z1, 2, None, criteria, 5, cv2.KMEANS_PP_CENTERS)
        # near_label = int(np.argmin(centers.ravel()))
        # mask_near = (labels.ravel() == near_label)

        # # 2) 在近群上取「窄帶」
        # z0 = np.median(z[mask_near])
        # band = 0.02  # 4 mm，可調參
        # mask_band = np.abs(z - z0) <= band
        # mask_use = mask_near & mask_band

        # pts_used = pts[mask_use]
        # if pts_used.shape[0] < 50:
        #     # 退化：近群前 10% 當窄帶
        #     q = np.percentile(z[mask_near], 10.0)
        #     pts_used = pts[(mask_near) & (z <= q)]
        #     if pts_used.shape[0] < 50:
        #         self.get_logger().warn(f"cap 候選太少: {pts_used.shape[0]}")
        #         return

        # region_pts = pts_used.tolist()

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

            MAX_ANGLE_THRESHOLD = 10.0  # 可自行調整允許最大夾角
            if angle_deg < MAX_ANGLE_THRESHOLD:
                self.get_logger().warn(f"法向量夾角超過 {MAX_ANGLE_THRESHOLD}°，忽略此平面")
                break


        inlier_cloud = pcd_filtered.select_by_index(best_inliers)
        inlier_points = np.asarray(inlier_cloud.points)
        center = np.mean(inlier_points, axis=0)

        # 建立姿態
        x_ref = np.array([-1.0, 0.0, 0.0])
        y_axis = np.cross(normal, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        x_axis /= np.linalg.norm(x_axis)
        rot_matrix = np.stack([x_axis, y_axis, normal], axis=1)
        quat = R.from_matrix(rot_matrix).as_quat()

        # Mix XY from bbox and pcd
        # center[2] = center[2]
        # --- 去畸變 ---
        bbox_pts = np.array([[bbox_center_uv]], dtype=np.float32)
        bbox_undistorted = cv2.undistortPoints(bbox_pts, camera_matrix, dist_coeffs, P=camera_matrix)
        bbox_u_nd, bbox_v_nd = bbox_undistorted[0, 0]
        # bbox_x = (bbox_u_nd - cx) * center[2] / fx
        # bbox_y = (bbox_v_nd - cy) * center[2] / fy
        # center[0] = (bbox_x + center[0]) / 2
        # center[1] = (bbox_y + center[1]) / 2

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
        combined_mask = np.stack([mask_img, self.depth_img_showable, self.depth_img_showable], axis=-1)
        # draw pcd center
        pcd_center_uv = [int((center[0] * fx) / center[2] + cx), int((center[1] * fy) / center[2] + cy)]
        cv2.circle(combined_mask, pcd_center_uv, 5, (255, 0, 0), -1)
        # draw bbox center
        cv2.circle(combined_mask, bbox_center_uv, 5, (0, 255, 0), -1)
        cv2.circle(combined_mask, mask_center ,5 , (0,0,255) , -1)
        cv2.imshow("Inlier Mask", combined_mask)
        cv2.waitKey(1)
        
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
        req.shelf_level = self.shelf_level
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
    node.create_rate(100)
    while rclpy.ok():
        rclpy.spin_once(node)
        cv2.waitKey(1)

if __name__ == '__main__':
    main()



