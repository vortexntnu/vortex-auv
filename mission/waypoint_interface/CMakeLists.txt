cmake_minimum_required(VERSION 3.0.2)
project(waypoint_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  rospy
  std_msgs
  tf
  vortex_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib geometry_msgs rospy std_msgs tf vortex_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/waypoint_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
