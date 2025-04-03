cmake_minimum_required(VERSION 3.8)
project(ekf_pose_filtering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(ekf_pose_filtering_node
src/ekf_pose_filtering_ros.cpp)


ament_target_dependencies(ekf_pose_filtering_node
rclcpp
geometry_msgs
tf2
tf2_ros
tf2_geometry_msgs
  message_filters
  vortex_filtering
  std_srvs)

install(TARGETS
  ekf_pose_filtering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
