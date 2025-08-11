import struct
import socket
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np

class LandMarkNode(Node):
    def __init__(self):
        super().__init__('rail_landmark')
        self.publisher_position = self.create_publisher(Pose, '/rail_position', 10)
        self.position_msg=Pose()
        self.connect()
    
    def connect(self):
        HOST = '192.168.10.1'
        PORT = 5001
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        s.bind((HOST, PORT))
        s.listen(5)
        print('server start at: %s:%s' % (HOST, PORT))
        print('wait for connection...')
        conn, addr = s.accept()
        print('connecting by ' + str(addr))
        while True:
            indata = conn.recv(1024)
            if len(indata) % 4 == 0:
                ## decode socket
                count = len(indata) // 4
                floats = struct.unpack(f'{count}f', indata)
                print('rec float[]:', floats)
                if floats==():
                    break
                print('======================================')
                base_pos = [floats[0]/1000, floats[1]/1000, floats[2]/1000]
                base_euler = [floats[3], floats[4], floats[5]] 
                self.T_landmark = self.pose_to_matrix(base_pos, base_euler, order='xyz')
                print(f"landmark@base:\n {self.T_landmark}")
                ## landmark@base to rail_origin@base
                self.T_mark_to_rail=np.eye(4)
                self.T_mark_to_rail[:3, :3] = R.from_euler('z', [0], degrees=True).as_matrix()
                self.T_mark_to_rail[:3, 3] = [-0.9265, -0.1145, -1.103]
                self.T_rail = self.T_landmark @ self.T_mark_to_rail
                print(f"rail@base:\n {self.T_rail}")
                print(f"pose: {self.T_rail[:3, 3]}")
                print(f"euler: {R.from_matrix(self.T_rail[:3, :3]).as_euler('xyz', degrees=True)}")
                ## publish
                quat = R.from_matrix(self.T_rail[:3, 3]).as_quat()
                self.position_msg.position.x = self.T_rail[0, 3]
                self.position_msg.position.y = self.T_rail[1, 3]
                self.position_msg.position.z = self.T_rail[2, 3]
                self.position_msg.orientation.x = quat[0]
                self.position_msg.orientation.y = quat[1]
                self.position_msg.orientation.z = quat[2]
                self.position_msg.orientation.w = quat[3]
                self.get_logger().info(f'pub pose: {self.position_msg}')
                self.publisher_position.publish(self.position_msg)
            else:
                # code=indata.decode()
                # print('recv: ' + str(indata.decode()))
                # print(code[0:3])
                print('rec float[] data lenth:', len(indata), 'bytes')
                print('org data:', indata)

def main(args=None):
    rclpy.init(args=args)
    node = LandMarkNode()
    node.destroy_node()
    rclpy.shutdown()

