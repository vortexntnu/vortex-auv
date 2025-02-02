cmake_minimum_required(VERSION 3.8)
project(joystick_interface_auv)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/${PROJECT_NAME}_node.py
  ${PROJECT_NAME}/joystick_utils.py
  ${PROJECT_NAME}/joystick_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
