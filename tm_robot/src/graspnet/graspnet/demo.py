import os
import yaml
import torch
import numpy as np
import open3d as o3d
import scipy.io as scio
from PIL import Image
from scipy.spatial.transform import Rotation as R

# ========== GraspNet 模型與工具 ==========
from graspnet import GraspNet, pred_decode
from graspnetAPI import GraspGroup
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image

# ========== 設定路徑 ==========
checkpoint_path = '/workspace/tm_robot/src/graspnet/resource/checkpoint-rs.tar'
data_dir = '/workspace/tm_robot/src/graspnet/graspnet/sample_data'

# checkpoint_path = './resource/checkpoint-rs.tar'

# ========== 初始化模型 ==========
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
net = GraspNet(
    input_feature_dim=0,
    num_view=300,
    num_angle=12,
    num_depth=4,
    cylinder_radius=0.05,
    hmin=-0.02,
    hmax_list=[0.01, 0.02, 0.03, 0.04],
    is_training=False
)
net.to(device)
checkpoint = torch.load(checkpoint_path)
net.load_state_dict(checkpoint['model_state_dict'])
net.eval()
print("GraspNet 模型載入完成")

# ========== 載入資料 ==========
color = np.array(Image.open(os.path.join(data_dir, 'color.png')))
depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))
workspace_mask = np.array(Image.open(os.path.join(data_dir, 'workspace_mask.png')))
meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))

intrinsic = meta['intrinsic_matrix']
factor_depth = float(meta['factor_depth'].squeeze())

camera = CameraInfo(
    width=1280.0, height=720.0,
    fx=intrinsic[0][0], fy=intrinsic[1][1],
    cx=intrinsic[0][2], cy=intrinsic[1][2],
    scale=factor_depth
)

# ========== 建立點雲 ==========
cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
# mask = (workspace_mask > 0) & (depth > 0)
mask = (depth > 0)
cloud_masked = cloud[mask]
color_masked = color[mask]

# ========== 下採樣 ==========
num_points = 20000
if len(cloud_masked) >= num_points:
    print("over sample points")
    idxs = np.random.choice(len(cloud_masked), num_points, replace=False)
else:
    print("under sample points")
    idxs1 = np.arange(len(cloud_masked))
    idxs2 = np.random.choice(len(cloud_masked), num_points - len(cloud_masked), replace=True)
    idxs = np.concatenate([idxs1, idxs2], axis=0)

cloud_sampled = cloud_masked[idxs]
color_sampled = color_masked[idxs]

# ========== 推論 ==========
cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(device)
end_points = {
    'point_clouds': cloud_tensor,
    'cloud_colors': color_sampled
}

with torch.no_grad():
    end_points = net(end_points)
    grasp_preds = pred_decode(end_points)

gg = GraspGroup(grasp_preds[0].detach().cpu().numpy())
gg.sort_by_score()

# ========== 儲存 Top-10 grasp ==========
grasp_list = []
for i, grasp in enumerate(gg[:10]):
    pose_matrix = np.eye(4)
    pose_matrix[:3, :3] = grasp.rotation_matrix
    pose_matrix[:3, 3] = grasp.translation
    grasp_list.append({
        'id': i,
        'pose': pose_matrix.tolist(),
        'score': float(grasp.score),
        'width': float(grasp.width)
    })
with open("top10_grasps.yaml", "w") as f:
    yaml.dump(grasp_list, f)

# ========== 印出最佳 grasp ==========
best = gg[0]
quat = R.from_matrix(best.rotation_matrix).as_quat()
print("Best Grasp Pose:")
print(f"Position: {best.translation}")
print(f"Orientation (quaternion): {quat}")
print(f"Score: {best.score:.4f}")
print(f"Width: {best.width:.4f}")

# ========== Open3D 顯示 ==========
# grippers = gg.to_open3d_geometry_list()
grippers = [best.to_open3d_geometry()]

cloud_o3d = o3d.geometry.PointCloud()
cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
cloud_o3d.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32) / 255.0)
o3d.visualization.draw_geometries([cloud_o3d, *grippers])
