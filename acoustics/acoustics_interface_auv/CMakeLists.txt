cmake_minimum_required(VERSION 3.8)
project(acoustics_interface_auv)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/acoustics_interface_node.py
  ${PROJECT_NAME}/acoustics_interface_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
