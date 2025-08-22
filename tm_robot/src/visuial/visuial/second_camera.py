#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import threading
import time
from tm_robot_if.srv import SecondCamera
import numpy as np

class SecondCamera(Node):
    def __init__(self):
        super().__init__("second_camera")
        self.img_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        # self.cap = cv2.VideoCapture(0)
        self.cap = cv2.VideoCapture("/dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0")
        self.timer = self.create_timer(0.03, self.timer_callback)

        if not self.cap.isOpened():
            self.get_logger().error('can not open second camera')
            return
        else:
            self.get_logger().info('second camera open')
        # camera parameters
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, 100.0)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 1)
        self.cap.set(cv2.CAP_PROP_GAIN, 0.1)
        self.cap.set(cv2.CAP_PROP_GAMMA, 0.4)
        self.print_camera_params()
 
    def print_camera_params(self):
        param_names = [
            ("FRAME_WIDTH", cv2.CAP_PROP_FRAME_WIDTH),
            ("FRAME_HEIGHT", cv2.CAP_PROP_FRAME_HEIGHT),
            ("BRIGHTNESS", cv2.CAP_PROP_BRIGHTNESS),
            ("CONTRAST", cv2.CAP_PROP_CONTRAST),
            ("SATURATION", cv2.CAP_PROP_SATURATION),
            ("HUE", cv2.CAP_PROP_HUE),
            ("GAIN", cv2.CAP_PROP_GAIN),
            ("EXPOSURE", cv2.CAP_PROP_EXPOSURE),
            ("AUTO_EXPOSURE", cv2.CAP_PROP_AUTO_EXPOSURE),
            ("GAMMA", cv2.CAP_PROP_GAMMA),
            ("AUTO_WB", cv2.CAP_PROP_AUTO_WB),
            ("WHITE_BALANCE_BLUE_U", cv2.CAP_PROP_WHITE_BALANCE_BLUE_U),
            ("WHITE_BALANCE_RED_V", cv2.CAP_PROP_WHITE_BALANCE_RED_V),
            ("FPS", cv2.CAP_PROP_FPS),
        ]

        self.get_logger().info("=== camera parameter ===")
        for name, prop in param_names:
            val = self.cap.get(prop)
            self.get_logger().info(f"{name}: {val}")


    def timer_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.flip(frame, -1)
        if not ret:
            # self.get_logger().warn('can not read frame')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.img_pub.publish(msg)
        self.get_logger().debug('pub second camera image')
        


def main(args=None):
    rclpy.init(args=args)
    node = SecondCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()