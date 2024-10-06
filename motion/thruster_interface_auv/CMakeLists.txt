cmake_minimum_required(VERSION 3.8)
project(thruster_interface_auv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(vortex_msgs REQUIRED)

ament_python_install_package(thruster_interface_auv)

install(DIRECTORY
  launch
  resources
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  thruster_interface_auv/thruster_interface_auv_node.py
  thruster_interface_auv/thruster_interface_auv_driver_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
