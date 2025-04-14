cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

add_executable(waypoint_manager_node
  src/waypoint_manager_node.cpp
  src/waypoint_manager.cpp
)

ament_target_dependencies(waypoint_manager_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  vortex_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
  spdlog
)
target_link_libraries(waypoint_manager_node fmt::fmt)

install(TARGETS
  waypoint_manager_node
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
