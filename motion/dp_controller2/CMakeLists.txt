cmake_minimum_required(VERSION 3.0.2)
project(dp_controller2)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  eigen_conversions
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  vortex_msgs
)

#find_package(Eigen3 REQUIRED)
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   nav_msgs#   std_msgs#   vortex_msgs
# )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dp_controller2
  CATKIN_DEPENDS 
    actionlib
    eigen_conversions
    geometry_msgs
    nav_msgs
    roscpp 
    std_msgs 
    vortex_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   include/dp_controller2/eigen_typedefs.h
# )


add_executable(${PROJECT_NAME}_node
 src/dp_controller_node.cpp
 src/dp_controller_ros.cpp
 src/quaternion_dp_controller.cpp
 )


add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_LIBRARIES}
)

# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(  
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dp_controller2.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
