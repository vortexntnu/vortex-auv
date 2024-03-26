cmake_minimum_required(VERSION 3.8)
project(blackbox)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

ament_python_install_package(blackbox)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  blackbox/blackbox_log_data.py
  blackbox/blackbox_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
