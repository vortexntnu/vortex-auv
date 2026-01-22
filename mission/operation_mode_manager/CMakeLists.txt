cmake_minimum_required(VERSION 3.8)
project(operation_mode_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(operation_mode_manager_cpp src/operation_mode_manager.cpp)

target_include_directories(operation_mode_manager_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  operation_mode_manager_cpp
  rclcpp
  std_msgs
  geometry_msgs
  vortex_msgs
  vortex_utils_ros
)

install(
  TARGETS operation_mode_manager_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
