# Grounded SAM2 ROS2

This git repository mainly packages Grounded SAM2 into ROS2 service.

---

## Installation

### 1. Install this repository 
```bash
git clone https://github.com/TKUwengkunduo/Grounded_SAM2_ROS2.git
```

### 2. Docker
```bash
cd docker/ubuntu
bash build.sh
bash run.sh
```


### 3. Setup and download
```bash
# Execute in docker
bash build_env.sh
```



---

## Execution example
serviceCompile package
```bash
ros2 run groundedsam2 service
```

### 2. test
```bash
ros2 run groundedsam2 test_client
```