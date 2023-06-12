cmake_minimum_required(VERSION 3.0.2)
project(tau_converter)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tau_converter
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/tau_converter.py
  scripts/sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

