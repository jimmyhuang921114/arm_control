#!/bin/bash

# 使用 GNOME Terminal 開 5 個分頁，每個執行不同的 Docker 啟動腳本

gnome-terminal \
  --tab --title="Main Container" -- bash -c "cd /home/iclab/tm_robot/docker && ./run.sh; exec bash" \
  --tab --title="GroundedSAM2" -- bash -c "cd /home/iclab/tm_robot/Grounded_SAM2_ROS2/docker/ubuntu && ./run.sh; exec bash" \
  --tab --title="tm_ws Ubuntu22" -- bash -c "cd /home/iclab/tm_ws/ubuntu22.04_ros2 && ./run.sh; exec bash" \
  --tab --title="Extra1" -- bash -c "cd ~/ && bash; exec bash" \
  --tab --title="Extra2" -- bash -c "cd ~/ && bash; exec bash"
