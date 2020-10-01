cmake_minimum_required(VERSION 2.8.3)
project(joystick_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  vortex_msgs
  roslint
)

roslint_python()

catkin_package(
  CATKIN_DEPENDS
  rospy
  vortex_msgs
)
