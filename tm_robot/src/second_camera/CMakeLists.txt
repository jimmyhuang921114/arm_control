cmake_minimum_required(VERSION 3.8)
project(second_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tm_robot_if REQUIRED)

include_directories(
  /usr/include/flycapture
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(/usr/lib)

add_executable(second_camera src/second_camera.cpp)

target_link_libraries(second_camera ${OpenCV_LIBS} flycapture)

ament_target_dependencies(second_camera
  rclcpp
  tm_robot_if
)

install(TARGETS second_camera
  DESTINATION lib/${PROJECT_NAME})

ament_package()


