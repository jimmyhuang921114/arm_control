import numpy as np
import scipy.io as scio
import os

# === 儲存目錄 ===
save_dir = "/workspace/tm_robot/src/graspnet/graspnet/sample_data"
os.makedirs(save_dir, exist_ok=True)

# === 更新後的 Intrinsic Matrix ===
intrinsic_matrix = np.array([
    [906.2919387834035, 0.0,                635.405341241147],
    [0.0,               905.4488695181474,  369.7903967839225],
    [0.0,               0.0,                1.0]
], dtype=np.float64)

# === 更新後的 Distortion Coefficients ===
distortion = np.array([
    0.10467326942788567,
    0.03975389017241552,
    0.0036263056134482282,
   -0.001096905985066281,
   -0.81705358696144
], dtype=np.float64)

# === 深度縮放比例（mm → m）===
depth_scale = 0.001
factor_depth = 1.0 / depth_scale  # = 1000.0

# === 儲存為 meta.mat ===
scio.savemat(os.path.join(save_dir, "meta.mat"), {
    'intrinsic_matrix': intrinsic_matrix,
    'distortion': distortion,
    'factor_depth': factor_depth
})

print("meta.mat 已儲存（已更新 intrinsic, distortion, factor_depth）")
