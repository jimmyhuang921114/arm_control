import pyrealsense2 as rs
import numpy as np
import cv2
import os
from ultralytics import YOLO

# === 儲存資料夾 ===
# save_dir = "./tm_robot/src/visiual/visiual/graspnet-base/doc/example_data"
save_dir = "/workspace/tm_robot/src/graspnet/sample_data"
os.makedirs(save_dir, exist_ok=True)

# === 初始化 RealSense 相機 ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
align = rs.align(rs.stream.color)

# === 等待影像穩定 ===
for _ in range(10):
    pipeline.wait_for_frames()

# === 擷取對齊後的幀 ===
frames = pipeline.wait_for_frames()
frames = align.process(frames)

color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()
if not color_frame or not depth_frame:
    raise RuntimeError("無法擷取 RealSense 影像")

color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())
print("color_frame:", color_frame)
print("color_frame.get_data():", type(color_frame.get_data()))
print("color_image:", type(color_image), color_image.shape if isinstance(color_image, np.ndarray) else "不是 numpy")

# === 儲存 color / depth 圖像 ===
cv2.imwrite(os.path.join(save_dir, "color.png"), color_image)
cv2.imwrite(os.path.join(save_dir, "depth.png"), depth_image)
print("已儲存 color.png 與 depth.png")

# === 關閉相機 ===
pipeline.stop()

# === 載入 YOLOv8 模型 ===
# model = YOLO("yolov8m-seg.pt")  # 可改為 yolov8s-seg.pt 等
model = YOLO("/workspace/tm_robot/src/visuial/resource/best.pt")

# === 執行 YOLOv8 segmentation 推論 ===
results = model(color_image)[0]

# === 合併所有物件的遮罩 ===
h, w = results.orig_shape  # 原始 color 圖像尺寸
combined_mask = np.zeros((h, w), dtype=np.uint8)

if results.masks is not None:
    masks = results.masks.data.cpu().numpy()  # shape: (N, 384, 640)
    for i, mask in enumerate(masks):
        # 先將遮罩 resize 回原圖尺寸
        mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
        combined_mask[mask_resized > 0.5] = 255

    cv2.imwrite(os.path.join(save_dir, "workspace_mask.png"), combined_mask)
    print("YOLO 遮罩已儲存 workspace_mask.png")

    # 顯示遮罩疊加圖（可選）
    overlay = color_image.copy()
    overlay[combined_mask == 255] = [0, 255, 0]
    cv2.imwrite(os.path.join(save_dir, "overlay_mask.png"), overlay)

else:
    print("YOLO 未偵測到任何遮罩")


print("所有資料已儲存至:", save_dir)
