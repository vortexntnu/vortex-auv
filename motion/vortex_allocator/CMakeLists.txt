cmake_minimum_required(VERSION 2.8.3)
project(vortex_allocator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  vortex
  vortex_msgs
  roslint
)

find_package(Eigen3 REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  eigen_conversions
  geometry_msgs
  roscpp
  vortex
  vortex_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/allocator_node.cpp
  src/allocator_ros.cpp
  src/pseudoinverse_allocator.cpp
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(${PROJECT_NAME}-test
    test/allocator_test.test
    test/allocator_test.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
  )

  roslint_add_test()
endif()
