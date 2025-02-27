cmake_minimum_required(VERSION 3.8)
project(internal_status_auv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(internal_status_auv)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  internal_status_auv/power_sense_module_lib.py
  internal_status_auv/power_sense_module_node.py
  internal_status_auv/pressure_sensor_lib.py
  internal_status_auv/pressure_sensor_node.py
  internal_status_auv/temperature_sensor_lib.py
  internal_status_auv/temperature_sensor_node.py
  internal_status_auv/gripper_feedback_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
