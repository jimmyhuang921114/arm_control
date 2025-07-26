import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tm_robot_if.srv import Paddle
from paddleocr import PaddleOCR
import numpy as np
import cv2
import os
from PIL import ImageDraw, ImageFont, Image

class PaddleOCRNode(Node):
    def __init__(self):
        super().__init__('paddleocr_node')
        self.bridge = CvBridge()

        # Service
        self.paddleocr_service = self.create_service(
            Paddle,
            'paddleocr_check',
            self.ocr_callback
        )

        # OCR 引擎
        self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')
        self.get_logger().info("PaddleOCR service 已啟動")

    def draw_ocr(self, img, boxes, txts, scores=None, drop_score=0.5, font_path="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if isinstance(img, np.ndarray):
            img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img)
        font = ImageFont.truetype(font_path, 20)
        for idx, (box, text) in enumerate(zip(boxes, txts)):
            if scores is not None and scores[idx] < drop_score:
                continue
            box = np.array(box).astype(np.int32).reshape((-1,))
            draw.rectangle([box[0], box[1], box[4], box[5]], outline=(255, 0, 0), width=2)
            draw.text((box[0], box[1]-20), text, font=font, fill=(0, 255, 0))
        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)

    def ocr_callback(self, request, response):
        try:
            # 1. 轉換 image
            cv_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')

            # 2. OCR 處理   
            result = self.ocr.ocr(cv_image, cls=True)

            boxes, texts, scores = [], [], []
            for line in result:
                for box, (text, score) in line:
                    boxes.append(box)
                    texts.append(text)
                    scores.append(score)

            response.ocr_detect_string = texts

            # 3. 繪圖
            image_with_box = self.draw_ocr(cv_image, boxes, texts, scores)

            # 4. 儲存圖片
            save_dir = '/workspace/tm_robot/src/visuial/visuial'
            os.makedirs(save_dir, exist_ok=True)
            save_path = os.path.join(save_dir, 'ocr_result.jpg')
            cv2.imwrite(save_path, image_with_box)

            self.get_logger().info(f"已儲存 OCR 結果圖像：{save_path}")
            return response

        except Exception as e:
            self.get_logger().error(f"OCR callback 錯誤: {e}")
            return response


def main(args=None):
    rclpy.init(args=args)
    node = PaddleOCRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
