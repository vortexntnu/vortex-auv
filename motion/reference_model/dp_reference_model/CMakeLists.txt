cmake_minimum_required(VERSION 3.0.2)
project(dp_reference_model)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  eigen_conversions
  geometry_msgs
  roscpp
  tf
  tf_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/reference_model_node.cpp
  src/reference_model.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)