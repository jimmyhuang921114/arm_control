#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
from geometry_msgs.msg import Pose




class SocketService(Node):
    def __init__(self):
        super().__init__('socket_service')
        self.host = '0.0.0.0'
        self.port = 6000
        threading.Thread(target=self.start_server, daemon=True).start() 
        self.slider_location_pub = self.create_publisher(Pose,'slider_position',10)
        self.camera_position = self.create_publisher(Pose,'/cube_position',10)
        self.


        self.sta_response_sub = self.create_subscription(StaResponse, '/sta_response', self.listen_node_callback, 10)
        self.data_sub = self.create_subscription(Bool,'arrive_status',self.check_point_callback,10)
        
        #=======socket setup
        self.HOST = '0.0.0.0'
        self.PORT = '5001'
        self.conn = None
        self.addr = None
        self.connect()

        #========thing pose init
        self.camera_pose = [0, 0, 0]
        self.camera_euler = [0, 0, 0]
        ]
        

        #========point list==========


    def pose_to_matrix(self,position, euler_deg, order='xyz'):
        rot = R.from_euler(order, euler_deg, degrees=True).as_matrix()
        trans = np.eye(4)
        trans[:3, :3] = rot
        trans[:3, 3] = position
        return trans
    
    def listen_node__callback(self,msg):
        self.get_logger().info(f'received : {msg}', once=True)


    def data_callback(self,msg):
        if msg.data:
    
    def connect(self):
        threading.Thread(target=self._accept_connection, daemon=True).start()

    def _accept_connection(self):





def main(args=None):
    rclpy.init(args=args)
    node = SocketService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
