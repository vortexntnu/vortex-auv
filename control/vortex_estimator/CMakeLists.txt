cmake_minimum_required(VERSION 2.8.3)
project(vortex_estimator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  roslint
  vortex_msgs
)

find_package(Eigen3 REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  eigen_conversions
  roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/estimator_node.cpp
  src/simple_estimator.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
