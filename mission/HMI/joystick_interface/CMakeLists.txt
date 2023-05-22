cmake_minimum_required(VERSION 3.0.2)
project(joystick_interface)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
  vortex_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/joystick_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
