#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import time
from tm_robot_if.srv import TMFlowMode
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R


class SocketService(Node):
    def __init__(self):
        super().__init__('socket_service')
        self.host = '0.0.0.0'
        self.port = 6000
        # ROS 2 service
        self.tm_flow_mode_srv = self.create_service(TMFlowMode, 'tm_flow_mode', self.tm_flow_mode_callback)
        # 建立 socket server
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen()
        self.get_logger().info(f"Socket server listening on port {self.port}")


    def tm_flow_mode_callback(self, request, response):
        try:
            self.get_logger().info(f"received call: mode = {request.mode}, argument = {request.argument}")
            conn, addr = self.server.accept()
            self.get_logger().info(f"client: {addr}")
            with conn:
                data = request.mode + "," + request.argument
                conn.sendall((data).encode("utf-8"))
                self.get_logger().info(f"send string: {data}")
                time.sleep(0.5)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"error: {e}")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SocketService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("keyboard exit")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import socket, time
# from tm_robot_if.srv import TMFlowMode
# import threading
# from std_msgs.msg import Int32
# from geometry_msgs.msg import Pose,Point,Quaternion

# from scipy.spatial.transform import Rotation as R

# class SocketService(Node):
#     def __init__(self):
#         super().__init__('socket_service')
#         self.host = '0.0.0.0'
#         self.port = 6000
#         self.tm_flow_mode_srv = self.create_service(TMFlowMode, 'tm_flow_mode', self.tm_flow_mode_callback)
#         self.shelf_select_pub = self.create_publisher(Int32, '/open_floor',10)
#         self.base_pose_pub = self.create_publisher(Pose, '/cube_position', 10)
#         self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         self.server.bind((self.host, self.port))
#         self.server.listen()
#         self.get_logger().info(f"Socket server listening on port {self.port}")

#     def tm_flow_mode_callback(self, request, response):
#         self.get_logger().info(f"mode: {request.mode}, argument: {request.argument}")
#         self.server_send(request.mode)
#         time.sleep(1)
#         self.server_send(request.argument)
#         response.success = True
#         return response

#     def server_send(self, value: str):
#         conn, addr = self.server.accept()
#         self.get_logger().info(f"Connected from {addr}")
#         with conn:
#             self.get_logger().info(f"Sending value: {value}")
#             conn.sendall(value.encode("utf-8"))
#             time.sleep(0.2)
#             conn.close()
    

#     # def publish_shelf_number(self, num):
#     #     pose_msg = Pose()
#     #     pose_msg.position = Point(x=-0.234, y=-0.357, z=0.692)

#     #     self.base_pose_pub.publish(pose_msg)
#     #     self.get_logger().info("已發佈 Pose 給 /cube_position")
#     #     roll, pitch, yaw = 144.46, -1.57, -89.18
#     #     quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
#     #     q_msg = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
#     #     print(f"Quaternion: x={q_msg.x}, y={q_msg.y}, z={q_msg.z}, w={q_msg.w}")


#     #     msg = Int32()
#     #     num=1
#     #     msg.data = num
#     #     self.shelf_select_pub.publish(msg)
#     #     self.get_logger().info(f"已發佈層架指令：{num}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = SocketService()
#     node.create_rate(100)
#     while rclpy.ok():
#         rclpy.spin_once(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



