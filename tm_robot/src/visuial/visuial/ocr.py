#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OCR Verify (image-in) ROS 2 Service (NO GRID, full-frame OCR)
- Services: /ocr_verify_img 及 /paddleocr_check (tm_robot_if/srv/Paddle)
- Request:  image (sensor_msgs/Image), med_name (string)
- Response: matched (bool)

不進行分割：直接對整張影像做多種前處理 → EasyOCR → 標註結果
命中 token（在 med_name 中出現）以「紅框」表示，其餘為「綠框」。
可選擇顯示視窗或輸出檔案（參數控制）。
"""

import os
import re
import time
from typing import List, Tuple, Optional

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tm_robot_if.srv import Paddle
import easyocr


# ---------- 文本處理 ----------
def normalize_text(s: str) -> str:
    if not s:
        return ""
    s = s.lower()
    return re.sub(r'[\s\.\,\-\_\(\)\[\]\{\}\|\/\\\:\;\!\?\+\=\*\'\"\`~^%$#@]+', '', s)


def split_tokens(s: str) -> List[str]:
    tokens: List[str] = []
    tokens += [t for t in re.findall(r'[a-z0-9]+', s.lower()) if len(t) >= 2]
    tokens += [ch for ch in s if '\u4e00' <= ch <= '\u9fff']
    return tokens


# ---------- 影像前處理 ----------
def preprocess_variants(bgr: np.ndarray, do_binarize: bool = True, try_invert: bool = False) -> List[Tuple[str, np.ndarray]]:
    """
    產生多個版本給 OCR 嘗試：
      - CLAHE 增強
      - 自適應二值
      - 反白（可選）
    """
    variants: List[Tuple[str, np.ndarray]] = []

    # CLAHE on L channel
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2Lab)
    L, A, B = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    L = clahe.apply(L)
    imgA = cv2.cvtColor(cv2.merge([L, A, B]), cv2.COLOR_Lab2BGR)
    variants.append(("clahe_bgr", imgA))

    if do_binarize:
        gray = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
        bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY, 41, 15)
        variants.append(("adaptive_bw", cv2.cvtColor(bw, cv2.COLOR_GRAY2BGR)))

    if try_invert:
        gray = cv2.cvtColor(imgA, cv2.COLOR_BGR2GRAY)
        inv = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                    cv2.THRESH_BINARY_INV, 41, 15)
        variants.append(("adaptive_inv", cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)))

    return variants


def easyocr_read(reader: easyocr.Reader, img_bgr: np.ndarray):
    """回傳 [(box(4點), text, conf), ...]"""
    return reader.readtext(img_bgr, detail=1, paragraph=False)


class OcrVerifyServiceImgApp:
    def __init__(self, context=None):
        if context is None:
            context = rclpy.get_default_context()

        self.node = rclpy.create_node('ocr_verify_service_img', context=context)

        # 參數（不做分割，所以沒有 rows/cols）
        self.node.declare_parameter('do_binarize', True)
        self.node.declare_parameter('try_invert', False)
        self.node.declare_parameter('easyocr_langs', ['ch_tra', 'en'])
        self.node.declare_parameter('easyocr_gpu', True)

        # 除錯顯示 / 存檔
        self.node.declare_parameter('debug_show_window', False)  # imshow 視窗
        self.node.declare_parameter('debug_save', True)          # 是否存檔
        self.node.declare_parameter('debug_scale', 0.9)          # 顯示/存檔縮放
        self.node.declare_parameter('debug_dir', '/workspace/tm_robot/src/visuial/visuial')

        self.bridge = CvBridge()
        self.reader: Optional[easyocr.Reader] = None

        # 兩個 service 名稱，向下相容
        self.srv1 = self.node.create_service(Paddle, 'ocr_verify_img', self.handle)
        self.srv2 = self.node.create_service(Paddle, 'paddleocr_check', self.handle)
        self.node.get_logger().info("OCR Verify Service ready: /ocr_verify_img & /paddleocr_check")

    # 便捷取參 & logger
    def P(self, name: str):
        return self.node.get_parameter(name).get_parameter_value()

    def log(self):
        return self.node.get_logger()

    # 視覺輸出：可顯示、可存檔
    def _output_image(self, title: str, img: np.ndarray):
        if img is None:
            return

        # 縮放
        try:
            scale = float(self.P('debug_scale').double_value)
        except Exception:
            scale = 1.0
        if scale != 1.0:
            h, w = img.shape[:2]
            img = cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))))

        # 視窗
        try:
            show_win = bool(self.P('debug_show_window').bool_value)
        except Exception:
            show_win = False
        if show_win:
            self.get_logger().info(f"Show")
            # cv2.imshow(title, img)
            # cv2.waitKey(1)

        # 存檔
        try:
            save_flag = bool(self.P('debug_save').bool_value)
        except Exception:
            save_flag = True
        if save_flag:
            out_dir = self.P('debug_dir').string_value or '/tmp'
            os.makedirs(out_dir, exist_ok=True)
            ts = time.strftime('%Y%m%d-%H%M%S')
            path = os.path.join(out_dir, f'{title}_{ts}.jpg')
            try:
                # cv2.imwrite(path, img)
                self.log().info(f"Saved: {path}")
            except Exception as e:
                self.log().warn(f"Save failed ({path}): {e}")

    def _annotate_results(self, frame_bgr: np.ndarray, results, name_norm: str):
        out = frame_bgr.copy()
        hit_found = False
        first_hit_token = None

        for item in results:
            try:
                box, txt, conf = item
            except Exception:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    box, txt = item[0], item[1]
                    conf = 0.0
                else:
                    continue

            tokens = split_tokens(str(txt))
            is_hit = any(t and t in name_norm for t in tokens)

            # 畫多邊形框
            try:
                pts = np.array(box, dtype=np.int32).reshape(-1, 1, 2)
                color = (0, 0, 255) if is_hit else (0, 255, 0)
                cv2.polylines(out, [pts], True, color, 2)
                # 標籤：文字 + 置信度
                label = f"{str(txt)} ({float(conf):.2f})" if isinstance(conf, (float, int)) else str(txt)
                p = tuple(pts[0, 0])
                cv2.putText(out, label, (p[0] + 5, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)
            except Exception:
                # 後備：畫整幅矩形
                h, w = out.shape[:2]
                color = (0, 0, 255) if is_hit else (0, 255, 0)
                cv2.rectangle(out, (2, 2), (w - 2, h - 2), color, 2)

            if is_hit and not hit_found:
                hit_found = True
                for t in tokens:
                    if t and t in name_norm:
                        first_hit_token = t
                        break

        return out, hit_found, first_hit_token

    # 主要 handler：整張圖直接做 OCR（不分割）
    def handle(self, req: Paddle.Request, res: Paddle.Response) -> Paddle.Response:
        # 1) 取影像
        try:
            frame = self.bridge.imgmsg_to_cv2(req.image, desired_encoding='bgr8')
        except Exception as e:
            self.log().error(f"cv_bridge failed: {e}")
            res.matched = False
            return res

        # 2) 取參數
        do_b = bool(self.P('do_binarize').bool_value)
        inv = bool(self.P('try_invert').bool_value)
        langs = [s for s in self.P('easyocr_langs').string_array_value]
        use_gpu = bool(self.P('easyocr_gpu').bool_value)

        # 3) 初始化 OCR
        if self.reader is None:
            try:
                self.log().info(f"Initializing EasyOCR: langs={langs}, gpu={use_gpu}")
                self.reader = easyocr.Reader(langs, gpu=use_gpu)
                self.log().info("EasyOCR ready.")
            except Exception as e:
                self.log().error(f"EasyOCR init failed: {e}")
                res.matched = False
                return res

        name_norm = normalize_text(req.med_name or "")

        # 4) 產生多種版本，逐一 OCR + 標註
        try:
            best_annotated = None
            best_hit = False
            best_tag = None

            variants = preprocess_variants(frame, do_binarize=do_b, try_invert=inv)
            # 在第一張原圖（未處理）也跑一次
            variants.insert(0, ("raw_bgr", frame))

            for vname, imgv in variants:
                results = easyocr_read(self.reader, imgv)
                annotated, hit, hit_token = self._annotate_results(imgv, results, name_norm)

                # 輸出每一個版本的結果（方便比對）
                self._output_image(f'ocr_{vname}', annotated)

                if hit and not best_hit:
                    best_hit = True
                    best_annotated = annotated
                    best_tag = (vname, hit_token)
                    # 命中即可提早結束（若想看全部版本，可移除此 break）
                    break

            if best_hit:
                vname, token = best_tag
                self.log().info(f"[HIT] token='{token}' in med_name='{req.med_name}' via {vname}")
                res.matched = True
                # 再存一次標註到「命中」名稱
                if best_annotated is not None:
                    self._output_image('ocr_hit', best_annotated)
                return res

            self.log().info("No hit on full-frame OCR.")
            res.matched = False
            return res

        except Exception as e:
            self.log().error(f"OCR/compare failed: {e}")
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
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
