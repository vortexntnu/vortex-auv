cmake_minimum_required(VERSION 3.0.2)
project(tac_tasks)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  smach
  smach_ros
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

install(PROGRAMS
  scripts/docking_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)