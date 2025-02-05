cmake_minimum_required(VERSION 3.8)
project(guidance_vtf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(guidance_vtf_node
  src/guidance_vtf_node.cpp
  src/guidance_vtf_ros.cpp
  src/guidance_vtf_utils.cpp
)

ament_target_dependencies(guidance_vtf_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
  tf2_geometry_msgs
)

install(TARGETS
  guidance_vtf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
