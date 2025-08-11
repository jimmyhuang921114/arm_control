from re import T
import time
import struct
import socket

from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32,Bool
import numpy as np
import threading


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_position = self.create_publisher(Pose, '/shelf_position', 10)
        # self.publisher_landmark = self.create_publisher(Pose, '/shelf_landmark', 10)
        self.leave_curobo = self.create_publisher(Bool, '/leave_node', 10)
        # self.publisher_shelf_status = self.create_publisher(Bool, '/shelf_status', 10)
        self.position_msg = Pose()
        self.landmark_msg = Pose()

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.floor_value = None
        
        self.camera_pos = [0, 0, 0]
        self.camera_euler = [0, 0, 0]
        self.T_camera = self.pose_to_matrix(self.camera_pos, self.camera_euler)
        self.count=0
        
        # self.photo_pose=[-396.45, 197.82, 768.02, 160.08, 0.84, 179.46]
        self.subscription = self.create_subscription(
            Int32,
            '/open_floor',
            self.floor_callback,
            10
        )
        
    def pose_to_matrix(self,position, euler_deg, order='xyz'):
        rot = R.from_euler(order, euler_deg, degrees=True).as_matrix()
        trans = np.eye(4)
        trans[:3, :3] = rot
        trans[:3, 3] = position
        return trans
    
    
    def euler_to_quaternion(self,rx, ry, rz, order='xyz', degrees=True):
        r = R.from_euler(order, [rx, ry, rz], degrees=degrees)
        q = r.as_quat()  # 回傳順序為 [x, y, z, w]
        return tuple(q)

    def transform_pose(self,position, euler_deg, T_transform, order='xyz'):
        T_pose = self.pose_to_matrix(position, euler_deg, order)
        T_new = T_transform @ T_pose
        new_pos = T_new[:3, 3]
        new_euler = R.from_matrix(T_new[:3, :3]).as_euler(order, degrees=True)
        return new_pos, new_euler
    
    def floor_callback(self,msg):
        print(f"floor set: {msg.data}")
        self.floor_value = msg.data
    
    def connect(self):
        while True:
            HOST = '0.0.0.0'
            PORT = 5000
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
                    count = len(indata) // 4
                    floats = struct.unpack(f'{count}f', indata)
                    print('rec float[]:', floats)
                    if floats==():
                        break
                    print('======================================')
                    base_euler = [floats[3], floats[4], floats[5]] 
                    # landmark_quat = R.from_euler('xyz', base_euler, degrees=True).as_quat()
                    # self.landmark_msg.position.x=floats[0]/1000
                    # self.landmark_msg.position.y=floats[1]/1000
                    # self.landmark_msg.position.z=floats[2]/1000
                    # self.landmark_msg.orientation.x=landmark_quat[0]
                    # self.landmark_msg.orientation.y=landmark_quat[1]
                    # self.landmark_msg.orientation.z=landmark_quat[2]
                    # self.landmark_msg.orientation.w=landmark_quat[3]
                    # self.publisher_landmark.publish(self.landmark_msg)
                    # self.get_logger().info(f'pubed {self.landmark_msg}', once=True)

                    base_pos = [floats[0]/1000, floats[1]/1000, floats[2]/1000]
                    self.T_base = self.pose_to_matrix(base_pos, base_euler, order='xyz')
                    print(self.T_base)
                    
                    R_160x = R.from_euler('x', 160, degrees=True).as_matrix()
                    p=[0.0, 0.0, 0.0]
                    self.T_mark_to_shelf=np.eye(4)
                    self.T_mark_to_shelf[:3, :3] = R_160x
                    self.shelf_pose=self.T_base @ self.T_mark_to_shelf
                    self.shelf_pose[:3, 3] = self.shelf_pose[:3, 3] + [-0.03, -0.048992, -1.159648] 
                    r=R.from_matrix(self.shelf_pose[:3, :3]).as_euler('xyz', degrees=True)
                    print("shelf_rotate=",r)
                    print("shelf_pose=",self.shelf_pose[:3,3])
                    quat=self.euler_to_quaternion(r[0],r[1],r[2])
                    self.position_msg.position.x=self.shelf_pose[0,3]
                    self.position_msg.position.y=self.shelf_pose[1,3]
                    self.position_msg.position.z=self.shelf_pose[2,3]
                    self.position_msg.orientation.x=quat[0]
                    self.position_msg.orientation.y=quat[1]
                    self.position_msg.orientation.z=quat[2]
                    self.position_msg.orientation.w=quat[3]
                    self.get_logger().info(f'pubed {self.position_msg}', once=True)
                    self.get_logger().info(f'euler angle is {r}', once=True)
                    self.publisher_position.publish(self.position_msg)
                    
                else:
                    print('rec float[] data lenth:', len(indata), 'bytes')
                    print('org data:', indata)

        
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.create_rate(100)

    threading.Thread(target=minimal_publisher.connect, daemon=True).start()

    while rclpy.ok():
        rclpy.spin_once(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()