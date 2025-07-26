import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Pose
from geometry_msgs.msg import Pose as PoseMsg
import numpy as np
from scipy.spatial.transform import Rotation as R

from tm_robot_if.srv import PoseSrv
from tm_msgs.srv import SetPositions

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped



from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class Camera2Base(Node):
    def __init__(self):
        super().__init__("camera2ee")
        client_cb_group = ReentrantCallbackGroup()
        timer_cb_group = client_cb_group
        
        self.thing_pose_srv = self.create_service(PoseSrv, 'thing_pose', self.pose_recevice,callback_group=client_cb_group)
        # self.tm_pose_client = self.create_client(SetPositions, '/set_positions',callback_group=client_cb_group)
        self.base_pose_pub = self.create_publisher(Pose, '/cube_position', 10)

        # self.pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 10)

    
        self.br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.T_camera_to_ee = self.create_hand_eye_transform()

        self.get_logger().info("Camera2Base node initialized")

    def create_hand_eye_transform(self):
        trans = np.array([0.03182497415436079, 0.033778346369806436, 0.06894045731602626])
        quat = [-0.0005731808229620196, 0.005829958484186053, -0.9999819369362807, 0.001344934563953267]
        T = np.eye(4)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = trans
        return T

    def lookup_ee_to_base(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='base',
                source_frame='flange',
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            trans = tf.transform.translation
            rot = tf.transform.rotation
            T = np.eye(4)
            T[:3, :3] = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            T[:3, 3] = [trans.x, trans.y, trans.z]
            return T
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None
    
    def pose_recevice(self, request, response):
        cam_pose = request.pose

        # === camera_pose → 4x4 matrix ===
        T_cam = np.eye(4)
        T_cam[:3, :3] = R.from_quat([
            cam_pose.orientation.x,
            cam_pose.orientation.y,
            cam_pose.orientation.z,
            cam_pose.orientation.w
        ]).as_matrix()
        T_cam[:3, 3] = [cam_pose.position.x, cam_pose.position.y, cam_pose.position.z]

        self.get_logger().info(
            f"[Received Pose from thing_pose]\n"
            f"  Position: x={cam_pose.position.x:.4f}, y={cam_pose.position.y:.4f}, z={cam_pose.position.z:.4f}"
        )

        T_ee_to_base = self.lookup_ee_to_base()
        if T_ee_to_base is None:
            response.success = False
            return response

        # === camera → base 轉換 ===
        T_base = T_ee_to_base @ self.T_camera_to_ee @ T_cam
        pos = T_base[:3, 3]
        quat = R.from_matrix(T_base[:3, :3]).as_quat()

        # === 封裝成 PoseStamped 並發布 ===
        base_pose = Pose()
        base_pose.position.x = pos[0]
        base_pose.position.y = pos[1]-0.02
        base_pose.position.z = pos[2]+0.05
        base_pose.orientation.x = quat[0]
        base_pose.orientation.y = quat[1]
        base_pose.orientation.z = quat[2]
        base_pose.orientation.w = quat[3]
        self.get_logger().info(f"PoseStamped:\n{base_pose}")

        self.base_pose_pub.publish(base_pose)
        self.get_logger().info("Published converted pose to /converted_pose")

        # === TF for visualization (可選) ===
        tf_msg = TransformStamped()
        # tf_msg.header = base_pose.header
        # tf_msg.child_frame_id = 'converted_pose'
        tf_msg.transform.translation.x = pos[0]
        tf_msg.transform.translation.y = pos[1]
        tf_msg.transform.translation.z = pos[2]
        tf_msg.transform.rotation = base_pose.orientation
        self.br.sendTransform(tf_msg)

        response.success = True
        return response

def main(args=None):
    # rclpy.init()
    # node = Camera2Base()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    rclpy.init()
    node = Camera2Base()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
