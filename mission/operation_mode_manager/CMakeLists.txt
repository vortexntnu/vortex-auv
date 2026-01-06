cmake_minimum_required(VERSION 3.8)
project(operation_mode_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(operation_mode_manager_cpp operation_mode_manager/operation_mode_manager.cpp)

ament_target_dependencies(
  operation_mode_manager_cpp
  rclcpp
  std_msgs
  geometry_msgs
  vortex_msgs
)

install(
  TARGETS operation_mode_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
