#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import yaml
import os

YAML_PATH = '/workspace/tm_robot/src/tm_robot_main/tm_robot_main/robot_base_point.yaml'

class ShelfPublisher(Node):
    def __init__(self):
        super().__init__('shelf_publisher_node')
        
        self.pose_pub = self.create_publisher(Pose, '/cube_position', 10)
        self.floor_pub = self.create_publisher(Int32, '/open_floor', 10)

        self.target_name = "first_shelf_pull"
        self.timer = self.create_timer(1.0, self.publish_from_yaml)
        self.get_logger().info("ShelfPublisher start")

    def publish_from_yaml(self):
        if not os.path.exists(YAML_PATH):
            self.get_logger().error(f"can't find：{YAML_PATH}")
            return

        with open(YAML_PATH, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if 'arm_base_pose' not in data or self.target_name not in data['arm_base_pose']:
            self.get_logger().error(f"no target pose in yaml: {self.target_name}")
            return

    #     if not self.order_list:
    #         return None
    #     return self.order_list.pop(0)
        pose_data = data['arm_base_pose'][self.target_name]
        if len(pose_data) != 6:
            self.get_logger().error("error data length")
            return

        x, y, z, roll_deg, pitch_deg, yaw_deg = pose_data
        roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        pose_msg = Pose()
        pose_msg.position = Point(x=x, y=y, z=z)
        pose_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f"Pose: {self.target_name}  /cube_position")

        floor_msg = Int32()
        floor_msg.data = 1
        self.floor_pub.publish(floor_msg)
        self.get_logger().info("Pose Int32 to /open_floor")


def main(args=None):
    rclpy.init(args=args)
    node = ShelfPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
