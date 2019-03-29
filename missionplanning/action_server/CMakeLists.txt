cmake_minimum_required(VERSION 2.8.3)
project(action_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
)

## Add actions
#add_action_files(DIRECTORY action 
#FILES 
#Fibonacci.action
#)

## Generate messages
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

## Define catkin exports
catkin_package(
  CATKIN_DEPENDS rospy actionlib
)

## Setup include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## Install scripts and executables
install(PROGRAMS
  #scripts/fibonacci_client.py
  scripts/fibonacci_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS #averaging_server averaging_client fibonacci_server fibonacci_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
