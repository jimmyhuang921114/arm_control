import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import easyocr
import cv2
import numpy as np


class EasyOCRTopicNode(Node):
    def __init__(self):
        super().__init__('easyocr_topic_node')
        self.bridge = CvBridge()

        # 初始化 OCR（中英文）
        self.reader = easyocr.Reader(['en', 'ch_sim'])

        # 訂閱彩色影像
        self.subscription = self.create_subscription(
            Image,
            '/tm_robot/color_image',
            self.image_callback,
            10
        )

        # 發佈辨識後影像
        self.image_pub = self.create_publisher(Image, '/easyocr/result_image', 10)

        self.get_logger().info("EasyOCR 節點已啟動，正在接收 /tm_robot/color_image")

    def image_callback(self, msg):
        try:
            # 將 ROS Image 轉為 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 執行 OCR
            results = self.reader.readtext(cv_image)

            for (bbox, text, prob) in results:
                if prob < 0.3:
                    continue
                (tl, tr, br, bl) = bbox
                tl = tuple(map(int, tl))
                br = tuple(map(int, br))
                cv2.rectangle(cv_image, tl, br, (0, 255, 0), 2)
                cv2.putText(cv_image, text, (tl[0], tl[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                self.get_logger().info(f"OCR: {text} (score={prob:.2f})")

            # 將標記後影像發佈
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"OCR 失敗: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EasyOCRTopicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
