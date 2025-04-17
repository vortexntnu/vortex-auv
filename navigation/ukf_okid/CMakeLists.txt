cmake_minimum_required(VERSION 3.8)
project(ukf_python)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ukf_python/ukf_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
