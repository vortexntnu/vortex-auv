cmake_minimum_required(VERSION 3.8)
project(blackbox_auv)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/blackbox_node.py
  ${PROJECT_NAME}/blackbox_log_data.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
