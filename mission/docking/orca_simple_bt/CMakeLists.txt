cmake_minimum_required(VERSION 3.8)
project(orca_simple_bt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)

add_executable(orca_simple_bt_node src/orca_simple_bt.cpp)
ament_target_dependencies(orca_simple_bt_node
  rclcpp rclcpp_action behaviortree_cpp_v3 vortex_msgs
)

install(TARGETS orca_simple_bt_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
