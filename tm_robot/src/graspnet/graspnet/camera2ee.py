import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np

class Camera2EE:
    def __init__(self):
        self.translation = np.array([
            0.029208644571584202,
            0.03115268306366813,
            0.06579499003271454
        ])
        self.quaternion = [
            -0.02720095141396113,
            -0.0375279346237228,
            -0.9988975179546995,
            0.007450569709350762
        ]

        self.T_camera_to_ee = np.eye(4)
        self.T_camera_to_ee[:3, :3] = R.from_quat(self.quaternion).as_matrix()
        self.T_camera_to_ee[:3, 3] = self.translation

    def transform_pose(self, input_pose: Pose) -> Pose:
        pos = input_pose.position
        pos_vec = np.array([pos.x, pos.y, pos.z, 1.0])
        new_pos = self.T_camera_to_ee @ pos_vec

        # 姿態轉換
        R_input = R.from_quat([
            input_pose.orientation.x,
            input_pose.orientation.y,
            input_pose.orientation.z,
            input_pose.orientation.w
        ])
        R_transform = R.from_matrix(self.T_camera_to_ee[:3, :3])
        R_output = R.from_matrix(R_transform.as_matrix() @ R_input.as_matrix())

        quat_out = R_output.as_quat()

        # 組成轉換後 Pose
        transformed_pose = Pose()
        transformed_pose.position = Point(
            x=new_pos[0],
            y=new_pos[1],
            z=new_pos[2]
        )
        transformed_pose.orientation = Quaternion(
            x=quat_out[0],
            y=quat_out[1],
            z=quat_out[2],
            w=quat_out[3]
        )
        return transformed_pose


class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('camera_to_ee_node')
        self.converter = Camera2EE()
        self.sub = self.create_subscription(PoseStamped, '/suction/pose', self.pose_callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/suction/pose_ee', 10)
        self.get_logger().info("Camera to EE Transform Node started.")

    def pose_callback(self, msg: PoseStamped):
        transformed_pose = self.converter.transform_pose(msg.pose)

        new_msg = PoseStamped()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = 'ee_link'
        new_msg.pose = transformed_pose

        self.pub.publish(new_msg)
        self.get_logger().info(
            f"Camera Pose → EE Pose: "
            f"({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) → "
            f"({new_msg.pose.position.x:.3f}, {new_msg.pose.position.y:.3f}, {new_msg.pose.position.z:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
