#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading

class SocketService(Node):
    def __init__(self):
        super().__init__('socket_service')
        self.host = '0.0.0.0'
        self.port = 6000
        threading.Thread(target=self.start_server, daemon=True).start()

    def start_server(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.host, self.port))
            server.listen()
            self.get_logger().info(f"Socket server listening on port {self.port}")

            while True:
                conn, addr = server.accept()
                self.get_logger().info(f"Connected from {addr}")
                with conn:
                    while True:
                        try:
                            raw_data = conn.recv(1024)
                            if not raw_data:
                                break

                            self.get_logger().info(f"[RAW] {raw_data}")

                            try:
                                data = raw_data.decode(errors='ignore').strip().lower()
                            except Exception as e:
                                self.get_logger().warn(f"Decode error: {e}")
                                data = ""

                            if raw_data in [b'\x01', b'1'] or data == 'true':
                                self.get_logger().info("Received True")
                            elif raw_data in [b'\x00', b'0'] or data == 'false':
                                self.get_logger().info("Received False")
                            else:
                                self.get_logger().warn(f"Unrecognized message: {raw_data}")
                        except Exception as e:
                            self.get_logger().error(f"Error while receiving: {e}")
                            break

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
