cmake_minimum_required(VERSION 3.8)
project(orca_reference_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

add_executable(reference_filter_client src/reference_filter_client.cpp)
ament_target_dependencies(reference_filter_client
  rclcpp rclcpp_action geometry_msgs vortex_msgs
)

install(TARGETS reference_filter_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
