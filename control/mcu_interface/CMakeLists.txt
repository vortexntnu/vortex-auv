cmake_minimum_required(VERSION 2.8.3)
project(mcu_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vortex_msgs
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  )

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${vortex_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}_node
  src/mcu_interface_node.cpp
  src/McuInterface.cpp
  src/crc.c
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )

add_dependencies(
  ${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  )
