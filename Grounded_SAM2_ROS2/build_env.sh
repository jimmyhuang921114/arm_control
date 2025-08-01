#!/bin/bash
git clone https://github.com/IDEA-Research/Grounded-SAM-2.git

cd Grounded-SAM-2/checkpoints
bash download_ckpts.sh

cd ../gdino_checkpoints
bash download_ckpts.sh

cd ..
pip install -e .
pip install --no-build-isolation -e grounding_dino

cd ..
colcon build
source install/setup.bash

GREEN='\033[1;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}==========Environment setup completed=========="
echo -e "${NC}Start up service: ${YELLOW}ros2 run groundedsam2 service"
echo -e "${NC}Test service: ${YELLOW}ros2 run groundedsam2 test_client"
