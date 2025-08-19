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

import math


class PlaneFittingNode(Node):
    def __init__(self):
        super().__init__('plane_fitting_node')

        # Parameters
        self.declare_parameter('distance_threshold', 0.001)
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

        self.depth_stack = []  # list of depth image frames
        self.max_depth_frames = 3


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
            self.bbox=request.bbox
            # self.plane_fitting(self.latest_pc)
            self.fit_plane_from_bbox()
            response.success = True
        except Exception as e:
            self.get_logger().error(f"mask 轉換失敗: {e}")
            response.success = False
        return response
    
    
    def fit_plane_from_bbox(self):
        if self.latest_depth is None or self.latest_mask is None or self.bbox is None:
            self.get_logger().warn("缺少 depth 或 mask 或 bbox")
            return

        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']
        xmin, ymin, xmax, ymax = map(int, self.bbox)
        camera_matrix = self.K.astype(np.float32)
        dist_coeffs   = self.dist_coeffs.astype(np.float32)

        # 只取 mask∩bbox 的像素
        submask = self.latest_mask[ymin:ymax, xmin:xmax] > 0
        ys, xs = np.where(submask)
        xs = xs + xmin
        ys = ys + ymin
        if xs.size == 0:
            self.get_logger().warn("mask∩bbox 沒有有效像素")
            return

        # 回投影成 3D
        region_pts = []
        for u, v in zip(xs, ys):
            z = self.latest_depth[v, u]
            if z <= 0 or not np.isfinite(z):
                continue
            pts_uv = np.array([[[u, v]]], dtype=np.float32)
            und = cv2.undistortPoints(pts_uv, camera_matrix, dist_coeffs, P=camera_matrix)
            u_nd, v_nd = und[0, 0]
            x = (u_nd - cx) * z / fx
            y = (v_nd - cy) * z / fy
            region_pts.append([x, y, z])

        if len(region_pts) < 200:
            self.get_logger().warn(f"有效點太少: {len(region_pts)}")
            return

        # 平面擬合（一次）
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(region_pts)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        model, inliers = pcd.segment_plane(
            distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
            ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
            num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
        )
        if not inliers or len(inliers) < 50:
            self.get_logger().warn("無法找到足夠 inlier 的平面")
            return

        a, b, c, d = model
        normal = np.array([a, b, c], dtype=np.float32)
        normal /= np.linalg.norm(normal)

        inlier_points = np.asarray(pcd.select_by_index(inliers).points)
        # 生成 10 個候選中心：對 inliers 做隨機子取樣後取中位數
        rng = np.random.default_rng()
        k   = min(400, len(inlier_points))
        centers = []
        for _ in range(10):
            idx = rng.choice(len(inlier_points), size=k, replace=False)
            centers.append(np.median(inlier_points[idx], axis=0))

        # 姿態：以擬合平面的法向為 z 軸（朝外），構造右手座標
        x_ref = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
        y_axis = np.cross(normal, x_ref); y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal); x_axis /= np.linalg.norm(x_axis)
        rot_matrix = np.stack([x_axis, y_axis, normal], axis=1)
        quat = R.from_matrix(rot_matrix).as_quat()

        # 視覺化（可選）
        mask_img = np.zeros(self.latest_depth.shape, dtype=np.uint8)
        for x, y, z in inlier_points:
            if z <= 0 or not np.isfinite(z):
                continue
            u = int((x * fx) / z + cx)
            v = int((y * fy) / z + cy)
            if 0 <= u < mask_img.shape[1] and 0 <= v < mask_img.shape[0]:
                mask_img[v, u] = 255
        vis = np.stack([mask_img, self.depth_img_showable, self.depth_img_showable], axis=-1)
        for ctr in centers:
            u = int((ctr[0]*fx)/ctr[2] + cx)
            v = int((ctr[1]*fy)/ctr[2] + cy)
            cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
        # 也畫一下 mask 中心
        mcy, mcx = np.mean(ys).astype(int), np.mean(xs).astype(int)
        cv2.circle(vis, (mcx, mcy), 5, (0, 255, 0), -1)
        cv2.imshow("Inliers & 10 centers", vis); cv2.waitKey(1)

        # 將 10 個候選點逐一送到 thing_pose（camera frame）
        for i, center in enumerate(centers, 1):
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'camera_depth_optical_frame'
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = float(center[0])
            pose_msg.pose.position.y = float(center[1])
            pose_msg.pose.position.z = float(center[2])
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])

            req = PoseSrv.Request()
            req.pose = pose_msg.pose
            self.pose_client.call_async(req)

        self.get_logger().info(f"[Batch] 已送出 10 個候選抓取點（同一張 mask）")


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




    def plane_fitting(self, pc_msg):
        if self.latest_depth is None or self.latest_mask is None or pc_msg is None:
            self.get_logger().warn("缺少必要資料，無法執行平面擬合")
            return

        # self.depth_stack.append(self.latest_depth.copy())
        # if len(self.depth_stack) < self.max_depth_frames:
        #     self.get_logger().info(f"等待更多 depth 幀 ({len(self.depth_stack)}/{self.max_depth_frames})")
        #     return



        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']
        # cv2.imshow("mask Result", self.latest_mask)
        # cv2.waitKey(1)
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

        # for depth_img in self.depth_stack:
        #     for v in range(height):
        #         for u in range(width):
        #             if self.latest_mask[v, u] == 0:
        #                 continue
        #             z = depth_img[v, u] * 0.001
        #             if z == 0 or np.isnan(z):
        #                 continue
        #             x = (u - cx) * z / fx
        #             y = (v - cy) * z / fy
        #             points.append([x, y, z])
        # # 平面擬合完成後，清空暫存深度
        # self.depth_stack.clear()





        points = np.array(points, dtype=np.float32)
        print("=========================================================================")
        print(points)
        print("=========================================================================")
        points_sorted=sorted(points, key=lambda x: x[2], reverse=False)
        tmp=points_sorted[0][2]
        # for x,y,z in points:
        #     if z-tmp>0.001:
        z_threshold = 0.015
        filtered_points=[]
        for i in points_sorted:
            if np.abs(i[2]-tmp)<=z_threshold:
                filtered_points.append(i)
            # print(x,y,z)


        if len(points) < 50:
            self.get_logger().warn('點數不足，無法進行平面擬合')
            return

        # RANSAC 擬合平面
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
        pcd_filtered, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
        # model, inliers = pcd_filtered.segment_plane(
        #     distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
        #     ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
        #     num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
        # )


        # [a, b, c, d] = model
        # normal = np.array([a, b, c], dtype=np.float32)
        # normal /= np.linalg.norm(normal)
        # center = np.mean(np.asarray(pcd.select_by_index(inliers).points), axis=0)

        
        best_index = -1
        min_angle_sum = float('inf')
        models = []
        normals = []
        centers = []
        inlier_lists = []

        # 擬合 10 次
        for i in range(50):
            model_i, inliers_i = pcd_filtered.segment_plane(
                distance_threshold=self.get_parameter('distance_threshold').get_parameter_value().double_value,
                ransac_n=self.get_parameter('ransac_n').get_parameter_value().integer_value,
                num_iterations=self.get_parameter('num_iterations').get_parameter_value().integer_value
            )
            [a, b, c, d] = model_i
            normal_i = np.array([a, b, c], dtype=np.float32)
            normal_i /= np.linalg.norm(normal_i)

            points_i = np.asarray(pcd_filtered.select_by_index(inliers_i).points)
            if points_i.shape[0] < 10:
                continue  # 太少 inlier，略過
            center_i = points_i[np.argmin(points_i[:, 2])]  # 改為最近點

            models.append(model_i)
            normals.append(normal_i)
            centers.append(center_i)
            inlier_lists.append(inliers_i)


        N = len(normals)
        angle_matrix = np.zeros((N, N))

        # 計算所有兩兩 normal 角度差
        for i in range(N):
            for j in range(i + 1, N):
                angle = math.acos(np.clip(np.dot(normals[i], normals[j]), -1.0, 1.0))
                angle_matrix[i, j] = angle
                angle_matrix[j, i] = angle

        # 計算每個 normal 與其他 normal 的角度差中位數
        medians = []
        for i in range(N):
            angle_diffs = angle_matrix[i, :]
            med = np.median(angle_diffs)
            medians.append(med)

        # 找中位角度最小者
        best_index = int(np.argmin(medians))

        # 最佳 normal 與其他資訊
        normal = normals[best_index]
        model = models[best_index]
        center = centers[best_index]
        inliers = inlier_lists[best_index]





        # 計算平均 normal
        # avg_normal = np.mean(normals, axis=0)
        # avg_normal /= np.linalg.norm(avg_normal)

        # # 找與平均 normal 角度最小的那次
        # for i, n in enumerate(normals):
        #     angle_diff = math.acos(np.clip(np.dot(n, avg_normal), -1.0, 1.0))
        #     if angle_diff < min_angle_sum:
        #         min_angle_sum = angle_diff
        #         best_index = i


        # # 取出最佳那次
        # model = models[best_index]
        # normal = avg_normal
        # center = centers[best_index]
        # inliers = inlier_lists[best_index]







        mask_img = np.zeros(self.latest_depth.shape, dtype=np.uint8)

        fx, fy = self.intrinsic['fx'], self.intrinsic['fy']
        cx, cy = self.intrinsic['cx'], self.intrinsic['cy']

        # 從過濾後的點雲中選出 inlier 點
        inlier_cloud = pcd_filtered.select_by_index(inliers)
        inlier_points = np.asarray(inlier_cloud.points)

        for point in inlier_points:
            x, y, z = point
            if z <= 0:
                continue
            u = int((x * fx) / z + cx)
            v = int((y * fy) / z + cy)
            if 0 <= u < mask_img.shape[1] and 0 <= v < mask_img.shape[0]:
                mask_img[v, u] = 255

        cv2.imshow("out Result", mask_img)
        cv2.waitKey(1)





        
        # inlier_points = np.asarray(pcd.select_by_index(inliers).points)
        # if len(inlier_points) == 0:
        #     self.get_logger().warn("沒有有效的 inlier 點")
        #     return
        # center = inlier_points[np.argmin(inlier_points[:, 2])]




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
