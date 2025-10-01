cmake_minimum_required(VERSION 3.8)
project(eskf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(stonefish_ros2 REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

add_executable(eskf_node
  src/eskf.cpp
  src/eskf_ros.cpp
  src/eskf_node.cpp
  src/eskf_utils.cpp
)

ament_target_dependencies(eskf_node
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
  spdlog
  fmt
  stonefish_ros2
)

target_link_libraries(eskf_node
  fmt::fmt
)

install(TARGETS
  eskf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
