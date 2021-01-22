cmake_minimum_required(VERSION 2.8.3)
project(vortex)

# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vortex_msgs
  sensor_msgs
  geomtery_msgs
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS
  include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)