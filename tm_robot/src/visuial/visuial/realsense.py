import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class RealsenseColorPointCloudNode(Node):
    def __init__(self):
        super().__init__('realsense_color_pcd_node')
        self.bridge = CvBridge()

        self.color_pub = self.create_publisher(Image, '/tm_robot/color_image', 10)
        self.depth_pub = self.create_publisher(Image, '/tm_robot/depth_image', 10)
        self.pcd_pub = self.create_publisher(PointCloud2, '/tm_robot/pointcloud', 10)

        self.get_logger().info("RealSense color + pointcloud node started")

        # 初始化 RealSense 相機
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # 啟動攝影機
        self.pipeline.start(config)

        # 取得 depth sensor 設定選項
        device = self.pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()

        # 設定近距離模式：High Accuracy + min_distance + laser_power
        if depth_sensor.supports(rs.option.visual_preset):
            try:
                preset_range = depth_sensor.get_option_range(rs.option.visual_preset)
                for i in range(int(preset_range.max + 1)):
                    name = depth_sensor.get_option_value_description(rs.option.visual_preset, i)
                    if "High Density" in name:
                        depth_sensor.set_option(rs.option.visual_preset, i)
                        self.get_logger().info(f"Set visual_preset to: {name}")
                        break
            except Exception as e:
                self.get_logger().warn(f"Failed to set visual_preset: {e}")


        if depth_sensor.supports(rs.option.min_distance):
            depth_sensor.set_option(rs.option.min_distance, 0.1)

        if depth_sensor.supports(rs.option.laser_power):
            depth_sensor.set_option(rs.option.laser_power, 360.0)

        # 深度對齊彩色畫面
        self.align = rs.align(rs.stream.color)

        # 30Hz 執行
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("無法取得影像")
            return

        # 發布彩色影像
        color_image = np.asanyarray(color_frame.get_data())
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        color_msg.header.stamp = self.get_clock().now().to_msg()
        color_msg.header.frame_id = "camera_color_optical_frame"
        self.color_pub.publish(color_msg)

        # 發布深度影像
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera_depth_optical_frame"
        self.depth_pub.publish(depth_msg)

        # 建立並發布點雲
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)
        vtx = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_depth_optical_frame"

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_msg = pc2.create_cloud(header, fields, vtx)
        self.pcd_pub.publish(cloud_msg)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseColorPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
