cmake_minimum_required(VERSION 3.8)
project(guidance_velocity_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_executable(guidance_node
  src/guidance_node.cpp
  src/guidance_ros.cpp
  src/guidance.cpp
)

ament_target_dependencies(guidance_node
  rclcpp
  rclcpp_action
  geometry_msgs
  vortex_msgs
  Eigen3
  tf2
  tf2_geometry_msgs
  spdlog
  fmt
)

target_link_libraries(guidance_node fmt::fmt)

install(TARGETS
  guidance_node
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