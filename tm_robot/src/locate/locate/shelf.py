from pickle import TRUE
import time,threading
import struct
import socket
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import numpy as np
from std_msgs.msg import Bool,String
from tm_msgs.msg import StaResponse

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_position = self.create_publisher(Pose, '/cube_position', 10)
        self.publisher_shelf_status = self.create_publisher(Bool, '/shelf_status', 10)
        self.publisher_switch_controller = self.create_publisher(Bool, '/tm5s/joint_command', 10)
        self.publisher_leave_listen = self.create_publisher(Bool, '/leave_node', 10)
        self.subscription_data = self.create_subscription(
            String,
            '/tmp',
            self.data_callback,
            10)
        self.position_msg=Pose()
        self.shelf_status_msg=Bool()
        
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.first = True

        self.sub=self.create_subscription(
            StaResponse,
            '/sta_response',
            self.listen_node_callback,
            10)
        
        self.subscription_data = self.create_subscription(
            Bool,
            '/arrive_status',
            self.check_point_callback,
            10)
        
        
        self.HOST = '192.168.10.1'
        self.PORT = 8080
        self.conn = None
        self.addr = None
        self.connect()
        
        
        

        self.camera_pos = [0, 0, 0]
        self.camera_euler = [0, 0, 0]
        self.T_camera = self.pose_to_matrix(self.camera_pos, self.camera_euler)
        self.count=0
        
        self.base_to_mark_pos=[-381.56143, 454.91217, 568.50037]
        self.base_to_mark_euler=[-159.23111, 0.9784483, -0.4009205]
        
        # self.top_photo_pose=[-396.95, 109.53, 773.95, 144.87, 0.80, 179.04]
        self.top_photo_pose=[-116.438, -397.21, 763.834, 144.969, -1.122, -89.497]
        
        self.top_pre_pull_pose=[0.36, -7.306, -54.165, 1.448, -16.31, -91.212]
        self.top_pull_pose=[3.178, 27.499, -13.029, -0.46, -20.181, -92.447]
        self.top_pulling_pose=[8.927, 162.065, -62.532, -0.46, -20.182, -92.447]
        self.top_pulled_pose=[1.186, 113.212, -97.123, 3.869, -17.842, -89.902]
        self.top_pull_pose2=[0.53, 14.18, -11.33, 2.67, -16.86, -92.05]
        self.top_pull_pose3=[1.03 , 33.68, -14.65, 2.13, -18.77, -91.16]
        self.top_point=[self.top_photo_pose, 
                        self.top_pre_pull_pose, 
                        # self.top_pull_pose,
                        self.top_pull_pose3,
                        self.top_pulling_pose,
                        # self.top_pulled_pose
                        ]
        
        self.under_photo_pose=[]
        self.under_pre_pull_pose=[]
        self.under_pull_pose=[]
        self.under_pulling_pose=[]
        self.under_pulled_pose=[]
        self.under_point=[self.under_photo_pose, 
                        self.under_pre_pull_pose, 
                        self.under_pull_pose,
                        self.under_pulling_pose,
                        self.under_pulled_pose]
        
        
        
        
        
        self.T_mark_to_base = self.pose_to_matrix(self.base_to_mark_pos, self.base_to_mark_euler)
        self.T_camera_to_base = self.T_mark_to_base @ np.linalg.inv(self.T_camera)
        print("first=",self.T_camera_to_base)
        
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
    def check_point_callback(self,msg):
        if msg.data:
            cmd2=Bool()
            cmd2.data=True
            self.publisher_leave_listen.publish(cmd2)
        else:
            pass

        
    def listen_node_callback(self,msg):
        self.get_logger().info(f'received : {msg}', once=True)
        cmd2=Bool()
        cmd2.data=False
        self.publisher_leave_listen.publish(cmd2)
        self.get_logger().info(f'pub leave_node : {cmd2}', once=True)

    def data_callback(self,msg):
        if msg.data=="TOP":
            self.get_logger().info(f'received : {msg}', once=True)
            quat=self.euler_to_quaternion(self.top_photo_pose[3], self.top_photo_pose[4], self.top_photo_pose[5])
            self.position_msg.position.x=self.top_photo_pose[0]/1000.0
            self.position_msg.position.y=self.top_photo_pose[1]/1000.0
            self.position_msg.position.z=self.top_photo_pose[2]/1000.0
            self.position_msg.orientation.x=quat[0]
            self.position_msg.orientation.y=quat[1]
            self.position_msg.orientation.z=quat[2]  
            self.position_msg.orientation.w=quat[3]
            print("top_photo_pose")
            print("pub pose=",self.position_msg)   
            print("=====================================================")                
            self.publisher_position.publish(self.position_msg)
            time.sleep(10)
        #     for i in self.top_point:
        #         if i==self.top_pre_pull_pose:
        #             self.test_pos=i[0:3]
        #             self.test_euler=i[3:6]
        #             pos,euler=self.transform_pose(self.test_pos,self.test_euler,self.T_camera_to_base)
        #             quat=self.euler_to_quaternion(euler[0],euler[1],euler[2])
        #             self.position_msg.position.x=pos[0]/1000.0
        #             self.position_msg.position.y=pos[1]/1000.0
        #             self.position_msg.position.z=pos[2]/1000.0
        #             self.position_msg.orientation.x=quat[0]
        #             self.position_msg.orientation.y=quat[1]
        #             self.position_msg.orientation.z=quat[2]  
        #             self.position_msg.orientation.w=quat[3]
        #             print("top_pre_pull_pose")
        #             print("pub pose=",self.position_msg)   
        #             print("=====================================================")            
        #             self.publisher_position.publish(self.position_msg)
        #             time.sleep(10)
        #             cmd=Bool()
        #             cmd.data=True
        #             self.publisher_shelf_status.publish(cmd)     
        #             time.sleep(1)
        #         elif i==self.top_pull_pose3:
        #             self.test_pos=i[0:3]
        #             self.test_euler=i[3:6]
        #             pos,euler=self.transform_pose(self.test_pos,self.test_euler,self.T_camera_to_base)
        #             quat=self.euler_to_quaternion(euler[0],euler[1],euler[2])
        #             self.position_msg.position.x=pos[0]/1000.0
        #             self.position_msg.position.y=pos[1]/1000.0
        #             self.position_msg.position.z=pos[2]/1000.0
        #             self.position_msg.orientation.x=quat[0]
        #             self.position_msg.orientation.y=quat[1]
        #             self.position_msg.orientation.z=quat[2]  
        #             self.position_msg.orientation.w=quat[3]
        #             print("top_pull_pose2")
        #             print("pub pose=",self.position_msg)   
        #             print("=====================================================")
        #             self.publisher_position.publish(self.position_msg)
        #             time.sleep(10)
        #             a=Bool()
        #             a.data=True
        #             self.publisher_switch_controller.publish(self.position_msg)
                    
        #         elif i==self.top_photo_pose:
        #             quat=self.euler_to_quaternion(i[3], i[4], i[5])
        #             self.position_msg.position.x=i[0]/1000.0
        #             self.position_msg.position.y=i[1]/1000.0
        #             self.position_msg.position.z=i[2]/1000.0
        #             self.position_msg.orientation.x=quat[0]
        #             self.position_msg.orientation.y=quat[1]
        #             self.position_msg.orientation.z=quat[2]  
        #             self.position_msg.orientation.w=quat[3]
        #             print("top_photo_pose")
        #             print("pub pose=",self.position_msg)   
        #             print("=====================================================")                
        #             self.publisher_position.publish(self.position_msg)
        #             time.sleep(10)
        #             cmd2=Bool()
        #             cmd2.data=True
        #             self.publisher_leave_listen.publish(cmd2)
        #             time.sleep(10)
                    
        #         elif i == self.top_pulling_pose:
        #             # cmd=Bool()
        #             # cmd.data=True
        #             # self.publisher_shelf_status.publish(cmd)
        #             self.test_pos=i[0:3]
        #             self.test_euler=i[3:6]
        #             pos,euler=self.transform_pose(self.test_pos,self.test_euler,self.T_camera_to_base)
        #             quat=self.euler_to_quaternion(euler[0],euler[1],euler[2])
        #             self.position_msg.position.x=pos[0]/1000.0
        #             self.position_msg.position.y=pos[1]/1000.0
        #             self.position_msg.position.z=pos[2]/1000.0
        #             self.position_msg.orientation.x=quat[0]
        #             self.position_msg.orientation.y=quat[1]
        #             self.position_msg.orientation.z=quat[2]  
        #             self.position_msg.orientation.w=quat[3]
        #             print("top_pulling_pose")
        #             print("pub pose=",self.position_msg)   
        #             print("=====================================================")                
        #             self.publisher_position.publish(self.position_msg)
        #             time.sleep(10)
        #         elif i == self.top_pulled_pose:
        #             self.test_pos=i[0:3]
        #             self.test_euler=i[3:6]
        #             pos,euler=self.transform_pose(self.test_pos,self.test_euler,self.T_camera_to_base)
        #             quat=self.euler_to_quaternion(euler[0],euler[1],euler[2])
        #             self.position_msg.position.x=pos[0]/1000.0
        #             self.position_msg.position.y=pos[1]/1000.0
        #             self.position_msg.position.z=pos[2]/1000.0
        #             self.position_msg.orientation.x=quat[0]
        #             self.position_msg.orientation.y=quat[1]
        #             self.position_msg.orientation.z=quat[2]  
        #             self.position_msg.orientation.w=quat[3]
        #             print("top_pulled_pose")
        #             print("pub pose=",self.position_msg)   
        #             print("=====================================================")                
        #             self.publisher_position.publish(self.position_msg)
        #             time.sleep(10)   
        #             cmd=Bool()
        #             cmd.data=False
        #             self.publisher_shelf_status.publish(cmd)     
        #             time.sleep(1) 
        #             cmd2=Bool()
        #             cmd2.data=True
        #             self.publisher_leave_listen.publish(cmd2)
                    
                
        # elif msg=="under":
        #     self.get_logger().info(f'received : {msg}', once=True)
        #     for i in self.top_point:
        #         quat=self.euler_to_quaternion(i[3], i[4], i[5])
        #         self.position_msg.position.x=i[0]/1000.0
        #         self.position_msg.position.y=i[1]/1000.0
        #         self.position_msg.position.z=i[2]/1000.0
        #         self.position_msg.orientation.x=quat[0]
        #         self.position_msg.orientation.y=quat[1]
        #         self.position_msg.orientation.z=quat[2]  
        #         self.position_msg.orientation.w=quat[3]   
        #         print("pub pose=",self.position_msg)   
        #         print("=====================================================")                
        #         self.publisher_position.publish(self.position_msg)
        #         time.sleep(0.5)
            
    def connect(self,):
        # 啟動一個執行緒來接受連線和資料
        threading.Thread(target=self._accept_connection, daemon=True).start()

    def _accept_connection(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self.HOST, self.PORT))
        s.listen(5)
        print(f"Server started at {self.HOST}:{self.PORT}")
        print("Waiting for connection...")

        self.conn, self.addr = s.accept()
        print(f"Connected by {self.addr}")

        # 接收資料並呼叫 callback
        while True:
            indata = self.conn.recv(1024)
            if len(indata) % 4 == 0:
                    count = len(indata) // 4
                    floats = struct.unpack(f'{count}f', indata)
                    print('rec float[]:', floats)
                    print('======================================')
                    if floats==():
                        break
                    else:
                        base_pos = [floats[0], floats[1], floats[2]]
                        base_euler = [floats[3], floats[4], floats[5]] 
                        self.T_mark_to_base = self.pose_to_matrix(base_pos, base_euler)
                        self.T_camera_to_base = self.T_mark_to_base @ np.linalg.inv(self.T_camera)
                        print("second=",self.T_camera_to_base)
                        
        threading.Thread(target=self._accept_connection, daemon=True).start()


    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
