cmake_minimum_required(VERSION 3.8)
project(thruster_interface_auv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/thruster_interface_auv_node.cpp
  src/thruster_interface_auv_ros.cpp
  src/thruster_interface_auv_driver.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  vortex_msgs
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
