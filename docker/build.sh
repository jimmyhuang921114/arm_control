#!/bin/bash

# Default image name
IMAGE_NAME="arm"

# 定義顏色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 切換到正確的 build context（tm_robot/）
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR/.."

# 驗證 image 名稱
if [[ "$IMAGE_NAME" =~ [^a-z0-9_.-] ]]; then
  echo -e "${RED}Error:${NC} Image name '$IMAGE_NAME' contains invalid characters."
  exit 1
fi

# 檢查是否已有 image
if docker images --format "{{.Repository}}" | grep -q "^${IMAGE_NAME}$"; then
  echo -e "${YELLOW}Warning:${NC} Image '$IMAGE_NAME' already exists."
  read -p "Do you want to overwrite it? (Y/n): " RESPONSE
  RESPONSE=${RESPONSE,,}
  if [[ "$RESPONSE" == "n" ]]; then
    echo -e "${RED}Aborted:${NC} Build cancelled."
    exit 1
  else
    echo -e "${YELLOW}Info:${NC} Overwriting image '$IMAGE_NAME'."
  fi
fi

# 執行 build
echo -e "${YELLOW}Building Docker image '$IMAGE_NAME'...${NC}"
docker build -f docker/Dockerfile -t "$IMAGE_NAME" .

# 檢查是否成功
if [ $? -eq 0 ]; then
  echo -e "${GREEN}Success:${NC} Docker image '$IMAGE_NAME' built successfully!"
else
  echo -e "${RED}Error:${NC} Failed to build Docker image."
  exit 1
fi
