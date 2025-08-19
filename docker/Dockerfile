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
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
    open3d \
    pyserial \
    crcmod \
    fastapi \
    uvicorn \
    easyocr




# ========== PyTorch（CUDA 12.1） ==========
RUN pip install --no-cache-dir \
    torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
    --index-url https://download.pytorch.org/whl/cu121
RUN pip install torch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 --index-url https://download.pytorch.org/whl/cu121



# ========== Deformable DETR ==========
# WORKDIR /opt
# RUN git clone https://github.com/fundamentalvision/Deformable-DETR.git
# WORKDIR /opt/Deformable-DETR
# RUN pip install -r requirements.txt

# WORKDIR /workspace



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
# RUN pip install --no-cache-dir \
#     torch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
#     --index-url https://download.pytorch.org/whl/cu121
# RUN python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"

# =========GroundSam2==========
# WORKDIR /workspace
# RUN git clone https://github.com/IDEA-Research/Grounded-SAM-2 /tmp/GroundSam2
# RUN echo "CUDA_HOME=/usr/local/cuda-12.1/" >> ~/.bashrc
# WORKDIR /tmp/GroundSam2OPENAI_API_KEY

# RUN git clone https://github.com/IDEA-Research/GroundingDINO.git /tmp/grounding_dino
# WORKDIR /tmp/grounding_dino

# RUN pip install supervision pycocotools transformers addict yapf timm




# ========== FlyCapture2 SDK ==========
# RUN mkdir -p /opt/flycapture && \
#     curl -L "https://flir.netx.net/file/asset/52868/original/attachment" -o /tmp/flycapture2.tar.gz && \
#     tar -xzf /tmp/flycapture2.tar.gz -C /opt/flycapture --strip-components=1

# WORKDIR /opt/flycapture
# RUN dpkg -i *.deb || apt-get install -fy && \
#     groupadd -f flirimaging && \
#     usermod -aG flirimaging work && \
#     echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", GROUP="flirimaging", MODE="0660"' > /etc/udev/rules.d/99-flir-usb.rules && \
#     udevadm control --reload-rules || true

# # ===Chat API
# WORKDIR /tmp

# 1. Clone + 安裝 openai_api_wrapper
RUN git clone https://github.com/TKUwengkunduo/openai_api_wrapper.git /tmp/openai_api_wrapper
WORKDIR /tmp/openai_api_wrapper
RUN pip install -e .


# ========== ROS Packages ==========
RUN apt install -y ros-humble-rmw-cyclonedds-cpp
RUN pip install --no-cache-dir --upgrade openai

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
