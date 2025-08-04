import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
from datetime import datetime
import threading
import time
import os
import queue


class MedicineMangementClient(Node):
    def __init__(self):
        super(),__init__('medicine_management_client')
        #=====get api info
        self.basic_med_info_API = 
        self.detail_med_info_API = 
        self.medicine_order_API = 
        #=====init parameter
        self.order_queue = queue.Queue()
        self.med_processing = True
        self.timers(1.0)
        self.lock = threading.Lock()

        self.medicine_order_srv = self.create_service(String,'medicine_order',self.medicine_order_callback)
        while not self.medicine_order_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Wait main service open")
    
    def check_order_loop(self):
        while rclpy.ok():
            try:
                res = requests.get(self.medicine_order_API)
                if res.status_code == 200:
                    orders = res.json().get("order",[])
                    for order in orders:
                        self.order_queue.put(order)
                        self.get_logger().inofo("New order")
                    self.try_process_next()
            except Exception as e:
                self.get_logger().warn("Can't process the order,{e}")
            time.sleep(5)

    def try_process_next(self):
        

    def medicine_order_callback(self,request,response):
        
    def 