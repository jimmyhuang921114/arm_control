#!/bin/bash

# Default image name (must match the name in build.sh)
IMAGE_NAME="arm"

# 定義顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # 無顏色

# Get the directory to mount (parent of the script's location)
# MOUNT_DIR=$(dirname "$(dirname "$(cd "$(dirname "$0")" && pwd)")")
MOUNT_DIR=$(dirname "$(cd "$(dirname "$0")" && pwd)")
# Check if GPU is available
if command -v nvidia-smi &> /dev/null; then
  GPU_FLAG="--gpus all"
  echo -e "${GREEN}GPU detected.${NC} Enabling GPU support for Docker."
else
  GPU_FLAG=""
  echo -e "${YELLOW}Warning:${NC} GPU not detected. Running without GPU support."
fi

# Run the Docker container
echo -e "${YELLOW}Running Docker container from image '$IMAGE_NAME'...${NC}"
# 在執行前開放 X11 顯示給 root
xhost +local:root

docker run -it --rm \
    --env-file open_ai.env \
    --env ROS_DOMAIN_ID=10 \
    --privileged \
    --net=host \
    --ipc=host \
    -v /dev:/dev \
    -v "$MOUNT_DIR":/workspace \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${HOME}/.Xauthority:/root/.Xauthority \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=/root/.Xauthority \
    $GPU_FLAG \
    --user work \
    --workdir /workspace \
    "$IMAGE_NAME" \
    bash --login



# Check if the container ran successfully
if [ $? -eq 0 ]; then
  echo -e "${GREEN}Success:${NC} Docker container exited successfully."
else
  echo -e "${RED}Error:${NC} Docker container failed to run."
  exit 1
fi
