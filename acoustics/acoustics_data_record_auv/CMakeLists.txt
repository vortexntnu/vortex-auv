cmake_minimum_required(VERSION 3.8)
project(acoustics_data_record_auv)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/acoustics_data_record_lib.py
  ${PROJECT_NAME}/acoustics_data_record_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
