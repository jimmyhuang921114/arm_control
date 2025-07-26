cmake_minimum_required(VERSION 3.8)
project(tm_robot_if)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmMove.srv"
  "srv/LastCheck.srv"
  "srv/MedicineOrder.srv"
  "srv/SecondCheck.srv"
  "srv/SetPositions.srv"
  "srv/SuctionPosition.srv"
  "srv/PoseSrv.srv"
  "srv/GroundedSAM2Interface.srv"
  "srv/Paddle.srv"
  "srv/DrugIdentify.srv"
  "srv/CaptureImage.srv"
  "srv/SecondCamera.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
