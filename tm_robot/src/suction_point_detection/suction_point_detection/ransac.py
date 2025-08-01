#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image, PointField
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Point, Vector3, PoseStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import open3d as o3d
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from tm_robot_if.srv import PoseSrv

class PlaneFittingNode(Node):
    def __init__(self):
        super().__init__('plane_fitting_node')
        # 參數宣告
        self.declare_parameter('distance_threshold', 0.01)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 1000)

        # 訂閱 ROI、點雲與彩色影像
        self.sub_roi = self.create_subscription(BoundingBox2D, 'spd/plane_roi', self.roi_callback, 10)
        self.sub_pc  = self.create_subscription(PointCloud2, 'tm_robot/pointcloud', self.pc_callback, 10)
        self.sub_color = self.create_subscription(Image, 'tm_robot/color_image', self.color_callback, 10)

        # 發佈 center, normal, marker, roi image
        self.pub_center    = self.create_publisher(Point, 'spd/plane_center', 10)
        self.pub_normal    = self.create_publisher(Vector3, 'spd/plane_normal', 10)
        self.pub_marker    = self.create_publisher(Marker, 'spd/plane_marker', 10)
        self.pub_roi_image = self.create_publisher(Image,'spd/color_image_with_roi', 10)

        # Bridge 與暫存
        self.bridge = CvBridge()
        self.latest_roi   = None
        self.latest_pc    = None
        self.latest_image = None

        # TF and Pose
        self.br = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/debug_pcd', 10)
        self.pose_client = self.create_client(PoseSrv, 'thing_pose')

    def roi_callback(self, msg: BoundingBox2D):
        self.latest_roi = msg
        self.draw_and_publish_roi()
        self.try_process()

    def pc_callback(self, msg: PointCloud2):
        self.latest_pc = msg
        self.try_process()

    def color_callback(self, msg: Image):
        self.latest_image = msg
        self.draw_and_publish_roi()

    def try_process(self):
        if self.latest_roi is None or self.latest_pc is None:
            return
        self.fit_plane(self.latest_pc, self.latest_roi)
        # 處理完成後清空
        self.latest_roi = None
        self.latest_pc  = None

    def draw_and_publish_roi(self):
        if self.latest_roi is None or self.latest_image is None:
            return
        # 1) 轉 cv image 並畫框
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        h_img, w_img = cv_img.shape[:2]
        cx = int(self.latest_roi.center.position.x)
        cy = int(self.latest_roi.center.position.y)
        sx = int(self.latest_roi.size_x/2)
        sy = int(self.latest_roi.size_y/2)
        xmin = max(0, cx-sx)
        xmax = min(w_img, cx+sx)
        ymin = max(0, cy-sy)
        ymax = min(h_img, cy+sy)
        cv2.rectangle(cv_img, (xmin, ymin), (xmax, ymax), (0,255,0), 5)
        # 2) 發佈
        img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        img_msg.header = self.latest_image.header
        self.pub_roi_image.publish(img_msg)

    def pubish_pcd(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_depth_optical_frame"  # <- use appropriate frame
        cloud_data = []
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pointcloud_msg = pc2.create_cloud(header, fields, points)
        self.pcd_pub.publish(pointcloud_msg)


    def fit_plane(self, pc_msg: PointCloud2, roi_msg: BoundingBox2D):
        # 1) 根據 organized/unorganized 決定影像大小 H, W
        if pc_msg.height > 1:
            H, W = pc_msg.height, pc_msg.width
        else:
            if self.latest_image is None:
                self.get_logger().warn('未取得影像，無法裁切 unorganized 點雲')
                return
            H, W = self.latest_image.height, self.latest_image.width

        # 2) 計算 ROI 座標
        cx = int(roi_msg.center.position.x)
        cy = int(roi_msg.center.position.y)
        sx = int(roi_msg.size_x / 2)
        sy = int(roi_msg.size_y / 2)
        xmin = max(0, cx - sx)
        xmax = min(W, cx + sx)
        ymin = max(0, cy - sy)
        ymax = min(H, cy + sy)
        self.get_logger().info(f'ROI: x[{xmin},{xmax}) y[{ymin},{ymax}), H={H} W={W}')

        # 3) 只讀取 ROI 範圍內的點 (ROI → 再格式轉換)
        region_pts = []
        for idx, p in enumerate(pc2.read_points(pc_msg, field_names=('x','y','z'), skip_nans=False)):
            row = idx // W
            if row < ymin:
                continue
            if row >= ymax:
                break  # 讀完 ROI 後即可跳出
            col = idx % W
            if col < xmin or col >= xmax:
                continue
            # 過濾 NaN / infinite
            if not (np.isfinite(p[0]) and np.isfinite(p[1]) and np.isfinite(p[2])):
                continue
            region_pts.append([p[0], p[1], p[2]])

        points = np.array(region_pts, dtype=np.float32)
        self.get_logger().info(f'裁切後點數: {points.shape[0]}')
        if points.shape[0] < 50:
            self.get_logger().warn('點數不足，無法擬合')
            return

        # 4) 移除離群值
        pcd_all = o3d.geometry.PointCloud()
        pcd_all.points = o3d.utility.Vector3dVector(points)
        pcd_filtered, ind = pcd_all.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        points = np.asarray(pcd_filtered.points, dtype=np.float32)
        self.get_logger().info(f'移除離群值後點數: {points.shape[0]}')
        if points.shape[0] < 50:
            self.get_logger().warn('移除離群值後點數不足，無法擬合')
            return
        
        self.pubish_pcd(points)

        # 5) RANSAC 平面擬合
        dist_th = self.get_parameter('distance_threshold').value
        ransac_n = self.get_parameter('ransac_n').value
        iters    = self.get_parameter('num_iterations').value
        model, inliers = pcd_filtered.segment_plane(
            distance_threshold=dist_th,
            ransac_n=ransac_n,
            num_iterations=iters)
        a, b, c, d = model
        normal = np.array([a, b, c], dtype=np.float32)
        normal /= np.linalg.norm(normal)
        center = np.mean(np.asarray(pcd_filtered.select_by_index(inliers).points), axis=0)

        # 確保法線方向
        # sensor_origin = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        # if np.dot(normal, center - sensor_origin) < 0:
        #     normal = -normal
        #     print(f"inv")
        # else:
        #     print(f"not inv")

        x_ref = np.array([-1.0, 0.0, 0.0])
        # if np.abs(np.dot(normal, x_ref)) > 0.95:
        #     x_temp = np.array([0.0, 1.0, 0.0])
        y_axis = np.cross(normal, x_ref)
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, normal)
        x_axis /= np.linalg.norm(x_axis)

        rot_matrix = np.stack([x_axis, y_axis, normal], axis=1)
        print(f"rot_matrix: \n{rot_matrix}")
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


        # 6) 發佈結果（將 numpy.float 显式转为 Python float）
        self.pub_center.publish(Point(
            x=float(center[0]),
            y=float(center[1]),
            z=float(center[2])
        ))
        self.pub_normal.publish(Vector3(
            x=float(normal[0]),
            y=float(normal[1]),
            z=float(normal[2])
        ))

        marker = Marker()
        marker.header = pc_msg.header
        marker.ns     = 'fitted_plane'
        marker.id     = 0
        marker.type   = Marker.ARROW
        marker.action = Marker.ADD
        # 下面也把 center/normal 转成 Python float
        marker.points = [
            Point(
                x=float(center[0]),
                y=float(center[1]),
                z=float(center[2])
            ),
            Point(
                x=float(center[0] + normal[0]*0.1),
                y=float(center[1] + normal[1]*0.1),
                z=float(center[2] + normal[2]*0.1)
            )
        ]
        marker.scale.x = 0.01
        marker.scale.y = 0.02
        marker.scale.z = 0.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_marker.publish(marker)
        self.get_logger().info(f'法線: {normal}, 中心: {center}')

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
