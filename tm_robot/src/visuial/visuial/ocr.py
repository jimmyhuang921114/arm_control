#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OCR Verify (image-in) ROS 2 Service
- Service: /ocr_verify_img (tm_robot_if/srv/Paddle)
- Request:  image (sensor_msgs/Image), med_name (string)
- Response: matched (bool)

設計重點：
1) 不繼承 rclpy.node.Node，改用 rclpy.create_node 組合式，避開部分環境下的 __enter__ 例外。
2) 顯式取得 context，避免多 Python / 多 rclpy 混裝造成的異常。
3) easyocr.Reader 延遲初始化且單例化，避免重複開銷。
"""

import re
from typing import List, Tuple

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tm_robot_if.srv import Paddle  # 假定: Request 有 image/med_name，Response 有 matched
import easyocr


# ---------- 小工具 ----------
def normalize_text(s: str) -> str:
    if not s:
        return ""
    s = s.lower()
    # 去掉空白與常見符號
    return re.sub(r'[\s\.\,\-\_\(\)\[\]\{\}\|\/\\\:\;\!\?\+\=\*\'\"\`~^%$#@]+', '', s)


def split_tokens(s: str) -> List[str]:
    tokens: List[str] = []
    tokens += [t for t in re.findall(r'[a-z0-9]+', s.lower()) if len(t) >= 2]
    tokens += [ch for ch in s if '\u4e00' <= ch <= '\u9fff']
    return tokens


def crop_band(img: np.ndarray, y0: float, y1: float, x0: float, x1: float) -> Tuple[np.ndarray, Tuple[int, int]]:
    H, W = img.shape[:2]
    yy0, yy1 = int(H * min(y0, y1)), int(H * max(y0, y1))
    xx0, xx1 = int(W * min(x0, x1)), int(W * max(x0, x1))
    yy0 = max(0, yy0)
    yy1 = min(H, yy1)
    xx0 = max(0, xx0)
    xx1 = min(W, xx1)
    return img[yy0:yy1, xx0:xx1].copy(), (xx0, yy0)


def preprocess_variants(bgr: np.ndarray, do_binarize: bool = True, try_invert: bool = False) -> List[Tuple[str, np.ndarray]]:
    variants: List[Tuple[str, np.ndarray]] = []
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    L, A, B = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    L = clahe.apply(L)
    imgA = cv2.cvtColor(cv2.merge([L, A, B]), cv2.COLOR_Lab2BGR)
    variants.append(("clahe_bgr", imgA))

    if do_binarize:
        gray = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
        bw = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 15
        )
        variants.append(("adaptive_bw", cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)))

    if try_invert:
        gray = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
        inv = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 41, 15
        )
        variants.append(("adaptive_inv", cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)))

    return variants


def easyocr_once(reader: easyocr.Reader, img_bgr: np.ndarray) -> List[str]:
    res = reader.readtext(img_bgr, detail=1, paragraph=False)
    texts: List[str] = []
    for item in res:
        if len(item) < 3:
            continue
        texts.append(str(item[1]))
    return texts


# ---------- App（組合式 Node） ----------
class OcrVerifyServiceImgApp:
    def __init__(self, context=None):
        if context is None:
            context = rclpy.get_default_context()

        self.node = rclpy.create_node('ocr_verify_service_img', context=context)

        # 參數宣告（可由 ros2 param set 覆寫）
        self.node.declare_parameter('rows', 2)        # 垂直切幾列
        self.node.declare_parameter('cols', 2)        # 水平切幾行
        self.node.declare_parameter('band0', 0.50)    # ROI 上界 (0~1)
        self.node.declare_parameter('band1', 0.88)    # ROI 下界
        self.node.declare_parameter('roi_x0', 0.15)   # ROI 左界
        self.node.declare_parameter('roi_x1', 0.85)   # ROI 右界
        self.node.declare_parameter('do_binarize', True)
        self.node.declare_parameter('try_invert', False)
        self.node.declare_parameter('easyocr_langs', ['ch_tra', 'en'])
        self.node.declare_parameter('easyocr_gpu', False)  # 視環境調整

        self.bridge = CvBridge()
        self.reader: easyocr.Reader = None  # 延遲初始化

        self.srv = self.node.create_service(Paddle, 'ocr_verify_img', self.handle)
        self.node.get_logger().info("OCR Verify (image-in) Service ready: /ocr_verify_img")

    # 方便存取 Node API
    def get_parameter(self, name: str):
        return self.node.get_parameter(name)

    def get_logger(self):
        return self.node.get_logger()

    # 服務處理
    def handle(self, req: Paddle.Request, res: Paddle.Response) -> Paddle.Response:
        # 影像轉換
        try:
            bgr = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            res.matched = False
            return res

        # 讀參數
        rows = max(1, int(self.get_parameter('rows').get_parameter_value().integer_value))
        cols = max(1, int(self.get_parameter('cols').get_parameter_value().integer_value))
        band0 = float(self.get_parameter('band0').get_parameter_value().double_value)
        band1 = float(self.get_parameter('band1').get_parameter_value().double_value)
        rx0 = float(self.get_parameter('roi_x0').get_parameter_value().double_value)
        rx1 = float(self.get_parameter('roi_x1').get_parameter_value().double_value)
        do_b = bool(self.get_parameter('do_binarize').get_parameter_value().bool_value)
        inv = bool(self.get_parameter('try_invert').get_parameter_value().bool_value)
        langs = [s for s in self.get_parameter('easyocr_langs').get_parameter_value().string_array_value]
        use_gpu = bool(self.get_parameter('easyocr_gpu').get_parameter_value().bool_value)

        # 取 ROI
        roi, _ = crop_band(bgr, band0, band1, rx0, rx1)

        # 初始化 OCR（延遲、單例）
        if self.reader is None:
            try:
                self.get_logger().info(f"Initializing EasyOCR: langs={langs}, gpu={use_gpu}")
                self.reader = easyocr.Reader(langs, gpu=use_gpu)
            except Exception as e:
                self.get_logger().error(f"EasyOCR init failed: {e}")
                res.matched = False
                return res

        # 切格
        H, W = roi.shape[:2]
        cell_h = max(1, H // rows)
        cell_w = max(1, W // cols)

        name_norm = normalize_text(req.med_name or "")

        # 逐格 OCR + 比對（token in med_name(normalized)）
        try:
            for r in range(rows):
                for c in range(cols):
                    y1 = r * cell_h
                    x1 = c * cell_w
                    y2 = (r + 1) * cell_h if r < rows - 1 else H
                    x2 = (c + 1) * cell_w if c < cols - 1 else W
                    cell = roi[y1:y2, x1:x2]

                    tokens: List[str] = []
                    for _, imgv in preprocess_variants(cell, do_binarize=do_b, try_invert=inv):
                        texts = easyocr_once(self.reader, imgv)
                        for t in texts:
                            tokens.extend(split_tokens(t))

                    for t in tokens:
                        if t and t in name_norm:
                            self.get_logger().info(
                                f"[HIT] cell({r},{c}) token='{t}' in med_name='{req.med_name}'"
                            )
                            res.matched = True
                            return res

            # 沒命中
            self.get_logger().info("No hit in any cell.")
            res.matched = False
            return res

        except Exception as e:
            self.get_logger().error(f"OCR/compare failed: {e}")
            res.matched = False
            return res


def main(args=None):
    rclpy.init(args=args)
    ctx = rclpy.get_default_context()
    app = OcrVerifyServiceImgApp(context=ctx)
    try:
        rclpy.spin(app.node)
    finally:
        app.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
