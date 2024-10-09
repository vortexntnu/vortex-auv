cmake_minimum_required(VERSION 3.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  thruster_interface_auv/thruster_interface_auv_node.py
  thruster_interface_auv/thruster_interface_auv_driver_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
