# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import re, os, argparse, sys
# from typing import List, Tuple
# from datetime import datetime
# import cv2
# import numpy as np
# import easyocr
# import pyrealsense2 as rs

# # 寫死存檔目錄
# SAVE_DIR = "/workspace/tm_robot/src/visuial/visuial"

# def normalize_text(s: str) -> str:
#     if not s: return ""
#     s = s.lower()
#     return re.sub(r'[\s\.\,\-\_\(\)\[\]\{\}\|\/\\\:\;\!\?\+\=\*\'\"\`~^%$#@]+', '', s)

# def split_tokens(s: str) -> List[str]:
#     toks: List[str] = []
#     toks += [t for t in re.findall(r'[a-z0-9]+', s.lower()) if len(t) >= 2]
#     toks += [ch for ch in s if '\u4e00' <= ch <= '\u9fff']
#     return toks

# def capture_realsense_color(width=1280, height=720, fps=30) -> np.ndarray:
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
#     try:
#         pipeline.start(config)
#         color_frame = None
#         for _ in range(10):  # 等自動曝光/白平衡穩定
#             frames = pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#         if color_frame is None:
#             raise RuntimeError("無法取得彩色影像")
#         return np.asanyarray(color_frame.get_data())
#     finally:
#         try: pipeline.stop()
#         except: pass

# def annotate(img: np.ndarray, results: List[Tuple], conf_th: float) -> np.ndarray:
#     vis = img.copy()
#     for bbox, text, conf in results:
#         if conf < conf_th: 
#             continue
#         # bbox 為 4 點：[ [x1,y1], [x2,y2], [x3,y3], [x4,y4] ]
#         xs = [int(p[0]) for p in bbox]
#         ys = [int(p[1]) for p in bbox]
#         x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
#         cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
#         label = f"{text} ({conf:.2f})"
#         y_text = max(0, y1 - 6)
#         cv2.putText(vis, label, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 1, cv2.LINE_AA)
#     return vis

# def main():
#     parser = argparse.ArgumentParser(description="RealSense + EasyOCR 驗證藥名，固定存檔到指定資料夾，stdout 只印 true/false。")
#     parser.add_argument("--name", required=True, help="要核對的藥名（字串）")
#     parser.add_argument("--th", type=float, default=0.5, help="OCR 信心值門檻 (default: 0.5)")
#     parser.add_argument("--gpu", action="store_true", help="啟用 GPU（若你的環境支援）")
#     parser.add_argument("--langs", nargs="+", default=["ch_tra", "en"], help="EasyOCR 語言 (default: ch_tra en)")
#     args = parser.parse_args()

#     # 確保資料夾存在
#     try:
#         os.makedirs(SAVE_DIR, exist_ok=True)
#     except Exception as e:
#         print(f"[err] 建立資料夾失敗: {SAVE_DIR} | {e}", file=sys.stderr)

#     try:
#         img = capture_realsense_color()
#         # 存一份原圖
#         ts = datetime.now().strftime("%Y%m%d_%H%M%S")
#         raw_path = os.path.join(SAVE_DIR, f"ocr_raw_{ts}.jpg")
#         try:
#             cv2.imwrite(raw_path, img)
#             print(f"[info] 原圖已存: {raw_path}", file=sys.stderr)
#         except Exception as e:
#             print(f"[warn] 原圖存檔失敗: {e}", file=sys.stderr)

#         reader = easyocr.Reader(args.langs, gpu=args.gpu)
#         results = reader.readtext(img, detail=1, paragraph=False)

#         name_norm = normalize_text(args.name)
#         matched = False

#         for bbox, text, conf in results:
#             if conf < args.th:
#                 continue
#             text_norm = normalize_text(text)
#             if text_norm and (text_norm in name_norm or name_norm in text_norm):
#                 matched = True
#                 break
#             for tok in split_tokens(text):
#                 if tok and tok in name_norm:
#                     matched = True
#                     break
#             if matched:
#                 break

#         # 產生標註圖並固定存檔
#         try:
#             vis = annotate(img, results, args.th)
#             res_tag = "ok" if matched else "ng"
#             vis_path = os.path.join(SAVE_DIR, f"ocr_annot_{res_tag}_{ts}.jpg")
#             cv2.imwrite(vis_path, vis)
#             print(f"[info] 標註圖已存: {vis_path}", file=sys.stderr)
#         except Exception as e:
#             print(f"[warn] 標註圖存檔失敗: {e}", file=sys.stderr)

#         # 只印 true/false 到 stdout（給上層流程使用）
#         print("true" if matched else "false")

#     except Exception as e:
#         # 任一環節失敗就視為不匹配
#         print(f"[err] 例外: {e}", file=sys.stderr)
#         print("false")

# if __name__ == "__main__":
#     main()


import easyocr

# 這行會自動檢查並下載需要的檔案
reader = easyocr.Reader(['ch_sim','en'], download_enabled=True)
