cmake_minimum_required(VERSION 2.8.3)
project(vortex_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  eigen_conversions
  nav_msgs
  roscpp
  std_msgs
  vortex
  vortex_msgs
  roslint
)

find_package(Eigen3 REQUIRED)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  dynamic_reconfigure
  eigen_conversions
  nav_msgs
  roscpp
  std_msgs
  vortex
  vortex_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(${PROJECT_NAME}_node
  src/controller_node.cpp
  src/controller_ros.cpp
  src/quaternion_pd_controller.cpp
  src/setpoints.cpp
  src/state.cpp
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
    test/controller_test.test
    test/controller_test.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
  )

  roslint_add_test()
endif()
