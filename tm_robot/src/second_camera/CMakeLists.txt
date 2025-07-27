cmake_minimum_required(VERSION 3.8)
project(second_camera)

# === 基本依賴 ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_robot_if REQUIRED)

# === Include 頭文件目錄 ===
include_directories(
  /usr/include/flycapture
  ${OpenCV_INCLUDE_DIRS}
)

# === 如果 flycapture 是共享庫，需要告知路徑 ===
link_directories(/usr/lib)

# === 編譯執行檔 ===
add_executable(second_camera src/second_camera.cpp)

# === 連結必要的函式庫 ===
target_link_libraries(second_camera
  ${OpenCV_LIBS}
  flycapture
)

# === 設定 ROS 2 相依套件 ===
ament_target_dependencies(second_camera
  rclcpp
  OpenCV
  cv_bridge
  sensor_msgs
  tm_robot_if
)

# === 安裝路徑 ===
install(TARGETS second_camera
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
