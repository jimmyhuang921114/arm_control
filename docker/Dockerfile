# ========== Base: CUDA + Ubuntu ==========
FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu22.04
# FROM pytorch/pytorch:2.3.1-cuda12.1-cudnn8-devel


# ========== Env Vars ==========
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8
ENV PATH=/usr/local/bin:$PATH
ENV TORCH_CUDA_ARCH_LIST="8.6"
ENV GENICAM_GENTL64_PATH=/workspace/spinnaker/lib
ENV SPINNAKER_ROOT=/workspace/spinnaker
ENV FLIR_ACCEPT_EULA=1
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/workspace/ros/${ROS_DISTRO}
ENV ROS_SETUP=${ROS_ROOT}/setup.bash
ENV PYTHONUNBUFFERED=1
ENV CPLUS_INCLUDE_PATH=/usr/include/k4a
ENV LIBRARY_PATH=/usr/lib/x86_64-linux-gnu
ENV CUDA_HOME=/usr/local/cuda-12.1

# ========== System Dependencies ==========
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential curl wget git vim nano ca-certificates gnupg \
    libssl-dev libffi-dev tmux sudo usbutils bash-completion \
    python3 python3-pip python3-dev \
    libgl1-mesa-glx libglib2.0-0 udev xz-utils \
    v4l-utils ffmpeg software-properties-common \
    libgtk2.0-0 libgtkmm-2.4-1v5 libglademm-2.4-1v5 libglademm-2.4-dev \
    libgtkglextmm-x11-1.2-dev libglade2-0 libglibmm-2.4-1v5 \
    libsigc++-2.0-0v5 xcb libxcb-cursor0 libxkbcommon-x11-0 \
    libqt6gui6 libqt6widgets6 qt6-wayland \
 && ln -s /usr/bin/python3 /usr/bin/python \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# ========== Python Packages ==========
RUN python -m pip install --no-cache-dir --upgrade pip setuptools


# 加上 --ignore-installed blinker
RUN pip install --no-cache-dir \
    --ignore-installed blinker \
    numpy==1.24.4 \
    opencv-python \
    PyYAML \
    ultralytics==8.3.49 \
    matplotlib \
    paddleocr \
    PySide6 \
    open3d



# ========== PyTorch（CUDA 12.1） ==========
RUN pip install --no-cache-dir \
    torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
    --index-url https://download.pytorch.org/whl/cu121
RUN pip install torch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 --index-url https://download.pytorch.org/whl/cu121



# ========== Deformable DETR ==========
WORKDIR /opt
RUN git clone https://github.com/fundamentalvision/Deformable-DETR.git
WORKDIR /opt/Deformable-DETR
RUN pip install -r requirements.txt

WORKDIR /workspace
# Build ms_deform_attn CUDA extensipytorch:on
# WORKDIR /opt/Deformable-DETR/models/ops
# RUN python setup.py build install
# RUN python setup.py build

# # ========== GroundingSAM ==========
# WORKDIR /opt
# RUN git clone https://github.com/IDEA-Research/Grounded-Segment-Anything.git
# WORKDIR /opt/Grounded-Segment-Anything
# RUN pip install -r requirements.txt
# RUN pip install -e segment_anything
# RUN pip install -e GroundingDINO


# ========== ROS 2 Humble ==========
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions python3-ament-package && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ========== RealSense SDK ==========
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || true && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
    apt-get update && apt-get install -y \
    librealsense2-utils librealsense2-dev && \
    pip install --no-cache-dir pyrealsense2==2.55.1.6486

# ========== PaddleOCR GPU ==========
RUN pip install --no-cache-dir \
    paddlepaddle-gpu==2.5.2 -f https://www.paddlepaddle.org.cn/whl/linux/mkl/avx/stable.html




# ## 安裝 torch 套件（指定 PyTorch 的索引）
RUN pip install --no-cache-dir \
    torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
    --index-url https://download.pytorch.org/whl/cu121
# RUN python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"

# =========GroundSam2==========
WORKDIR /workspace
RUN git clone https://github.com/IDEA-Research/Grounded-SAM-2 /tmp/GroundSam2
RUN echo "CUDA_HOME=/usr/local/cuda-12.1/" >> ~/.bashrc
WORKDIR /tmp/GroundSam2
# RUN pip install -e .
# RUN pip install torch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 \
#     --index-url https://download.pytorch.org/whl/cu121

RUN git clone https://github.com/IDEA-Research/GroundingDINO.git /tmp/grounding_dino
WORKDIR /tmp/grounding_dino
# RUN sed -i '/^def install_torch()/,/^install_torch()/d' setup.py
# RUN pip install -e . --no-build-isolation

# WORKDIR /tmp/GroundSam2/grounding_dino

# # === 安裝 GroundingDINO 前 patch 掉強制安裝 torch 的程式碼 ===
# RUN sed -i '/^def install_torch()/,/^install_torch()/d' setup.py
# RUN pip install --no-build-isolation -e .


# RUN git clone https://github.com/TKUwengkunduo/Grounded_SAM2_ROS2.git /workspace/Grounded_SAM2_ROS2
# RUN bash /workspace/Grounded_SAM2_ROS2/build_env.sh 
RUN pip install supervision pycocotools transformers addict yapf timm

# =========LLM==========
# WORKDIR /workspace
# RUN git clone 



# ========== FlyCapture2 SDK ==========
RUN mkdir -p /opt/flycapture && \
    curl -L "https://flir.netx.net/file/asset/52868/original/attachment" -o /tmp/flycapture2.tar.gz && \
    tar -xzf /tmp/flycapture2.tar.gz -C /opt/flycapture --strip-components=1

WORKDIR /opt/flycapture
RUN dpkg -i *.deb || apt-get install -fy && \
    groupadd -f flirimaging && \
    usermod -aG flirimaging work && \
    echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", GROUP="flirimaging", MODE="0660"' > /etc/udev/rules.d/99-flir-usb.rules && \
    udevadm control --reload-rules || true


# === 安裝 Azure Kinect SDK v1.4.2（完整）===
# WORKDIR /tmp/kinect

# # 安裝必要工具
# RUN apt-get update && \
#     apt-get install -y wget curl gnupg software-properties-common apt-transport-https && \
#     apt-get install 

# # 新增 Microsoft APT 來源（供 dev/tools 使用）
# RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \
#     add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main"

# # === 解壓 libk4a1.4_1.4.2 主套件 ===
# RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_amd64.deb && \
#     dpkg -x libk4a1.4_1.4.2_amd64.deb ./libk4a_extract && \
#     cp -r ./libk4a_extract/usr/* /usr/ && \
#     ldconfig

# # === 手動安裝 libsoundio===
# RUN wget http://mirrors.ustc.edu.cn/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb && \
#     dpkg -i libsoundio1_1.1.0-1_amd64.deb

# # === 安裝 dev headers + k4a-tools ===
# RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4-dev_1.4.2_amd64.deb && \
#     wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.2_amd64.deb && \
#     dpkg -i libk4a1.4-dev_1.4.2_amd64.deb k4a-tools_1.4.2_amd64.deb || apt-get install -f -y

# # === 加入 Azure Kinect 的 UDEV 規則（僅 Depth 模組）===
# RUN echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097c", MODE:="0666"' > /etc/udev/rules.d/99-k4a.rules

# # === 手動複製 k4a-tools 二進位===
# RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.2_amd64.deb && \
#     dpkg -x k4a-tools_1.4.2_amd64.deb ./k4a_tools_extract && \
#     cp ./k4a_tools_extract/usr/bin/k4a* /usr/local/bin/ && \
#     chmod +x /usr/local/bin/k4a*


# # === 清理暫存（可選）===
# RUN rm -rf /tmp/kinect

# ===Chat API
WORKDIR /workspace

# 1. Clone + 安裝 openai_api_wrapper
RUN git clone https://github.com/TKUwengkunduo/openai_api_wrapper.git && \
    pip install -e openai_api_wrapper

# 2. Clone llm_drug_identification_system 到 /workspace/llm_drug_identification_system
RUN git clone https://github.com/TKUwengkunduo/llm_drug_identification_system.git

# ========== User & Workspace ==========
RUN useradd -ms /bin/bash work && echo "work ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p /workspace && chown work:work /workspace && \
    usermod -aG dialout,video work
WORKDIR /workspace

# ========== bashrc ==========
RUN echo "source /opt/ros/humble/setup.bash" >> /home/work/.bashrc && \
    echo "source /usr/share/bash-completion/bash_completion" >> /home/work/.bashrc && \
    chown work:work /home/work/.bashrc


USER work
CMD ["bash"]
