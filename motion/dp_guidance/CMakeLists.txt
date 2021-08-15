cmake_minimum_required(VERSION 3.0.2)
project(dp_guidance)

find_package(catkin REQUIRED COMPONENTS
  #message_generation
  roscpp
  rospy
  nav_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dp_guidance
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



