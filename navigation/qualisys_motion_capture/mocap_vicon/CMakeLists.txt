cmake_minimum_required(VERSION 2.8.12)
project(mocap_vicon)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  tf_conversions
  mocap_base
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mocap_vicon_driver ViconDataStreamSDK_CPP
  CATKIN_DEPENDS roscpp geometry_msgs tf tf_conversions mocap_base
  DEPENDS Boost EIGEN3
)

add_subdirectory(src/vicon_sdk)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mocap_vicon_driver
  src/ViconDriver.cpp
)
target_link_libraries(mocap_vicon_driver
  ViconDataStreamSDK_CPP
  ${VICON_BOOST_LIBS}
  ${catkin_LIBRARIES}
)
add_dependencies(mocap_vicon_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(mocap_vicon_node
  src/vicon.cpp
)
target_link_libraries(mocap_vicon_node
  mocap_vicon_driver
  ${catkin_LIBRARIES}
)
