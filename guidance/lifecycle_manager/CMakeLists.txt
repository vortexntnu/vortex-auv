cmake_minimum_required(VERSION 3.8)
project(lifecycle_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(include)

add_executable(lifecycle_manager_node
  src/lifecycle_service_client.cpp
  src/lifecycle_manager_node.cpp
)

ament_target_dependencies(lifecycle_manager
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  vortex_msgs
  vortex_utils
)

install(TARGETS
  lifecycle_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
