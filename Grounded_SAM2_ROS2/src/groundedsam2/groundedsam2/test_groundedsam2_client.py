import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from interface.srv import GroundedSAM2Interface

import cv2
import numpy as np
import json
import pycocotools.mask as mask_util
import os

# ================================
# ✅ 可調參數設定區
# ================================
IMAGE_PATH = '/workspace/test_img/22_Color.png'   # ✅ 圖像路徑
PROMPT = 'medicament'                            # ✅ 提示詞
OUTPUT_PATH = './output_result.png'              # ✅ 儲存輸出檔案位置

CONFIDENCE_THRESHOLD = 0.2                        # ✅ 信心分數閾值
SIZE_THRESHOLD = 0.2                              # ✅ 遮罩最小尺寸閾值（相對比例）
SHOW_BBOX = True                                  # ✅ 是否顯示 bbox 框
# ================================


class GroundedSAM2Client(Node):
    def __init__(self):
        super().__init__('grounded_sam2_client')
        self.cli = self.create_client(GroundedSAM2Interface, 'grounded_sam2')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wait Grounded SAM2 Service Star...')

        self.bridge = CvBridge()

    def send_request(self, image_path, prompt, confidence_threshold, size_threshold):
        self.cv_image = cv2.imread(image_path)
        if self.cv_image is None:
            self.get_logger().error(f'無法讀取圖片: {image_path}')
            return

        req = GroundedSAM2Interface.Request()
        ros_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
        ros_image.header = Header()
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera"

        req.image = ros_image
        req.prompt = prompt
        req.confidence_threshold = float(confidence_threshold)
        req.size_threshold = float(size_threshold)

        self.future = self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    client = GroundedSAM2Client()
    client.send_request(IMAGE_PATH, PROMPT, CONFIDENCE_THRESHOLD, SIZE_THRESHOLD)

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print(f'\n📦 Bounding Boxes: {response.bbox}')
                print(f'🧩 Segmentations (RLE):')

                image = client.cv_image.copy()
                height, width, _ = image.shape

                num_boxes = len(response.bbox) // 4

                for i in range(num_boxes):
                    # --- Mask ---
                    rle_str = response.seg[i]
                    print(f'  Mask {i+1}: {rle_str[:60]}...')
                    seg_dict = json.loads(rle_str)
                    mask = mask_util.decode(seg_dict)  # shape: (H, W)

                    # --- Mask 顏色 ---
                    color = np.random.randint(0, 255, (3,), dtype=np.uint8)
                    image[mask == 1] = image[mask == 1] * 0.5 + color * 0.5

                    # --- Bounding Box ---
                    if SHOW_BBOX:
                        x1, y1, x2, y2 = response.bbox[i * 4:i * 4 + 4]
                        cv2.rectangle(
                            image,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            color=tuple(int(c) for c in color),
                            thickness=2
                        )

                # --- 顯示與儲存 ---
                cv2.imshow("Segmented Result", image)
                cv2.imwrite(OUTPUT_PATH, image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            except Exception as e:
                client.get_logger().error(f'服務呼叫失敗: {str(e)}')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
