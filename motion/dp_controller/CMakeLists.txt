cmake_minimum_required(VERSION 3.0.2)
project(dp_controller)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  eigen_conversions
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  vortex_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
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
#add_action_files(
#  DIRECTORY action
#  FILES dp.action
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs    #nav_msgs       vortex_msgs
#  #std_msgs
#  actionlib_msgs
#)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dp_controller
  CATKIN_DEPENDS 
    actionlib
    actionlib_msgs
    message_generation
    message_runtime
    eigen_conversions
    geometry_msgs
    nav_msgs
    roscpp 
    std_msgs 
    vortex_msgs
    dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

# add_library(${PROJECT_NAME}
#   include/dp_controller/eigen_typedefs.h
# )

# DP controller
add_executable(${PROJECT_NAME}_node
 src/dp_controller_node.cpp
 src/dp_controller_ros.cpp
 src/quaternion_dp_controller.cpp
 src/dp_action_server.cpp
#  src/dp_action_test_client.cpp
 )


add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  #${${PROJECT_NAME}_EXPORTED_TARGETS}    #does not want to build with this
  ${catkin_LIBRARIES}
)

# DP server (maybe add this to the other node)
# add_executable(${PROJECT_NAME}_server
#  src/dp_action_server.cpp
# )

# target_link_libraries(${PROJECT_NAME}_server
# #${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_LIBRARIES}
# )
# add_dependencies(${PROJECT_NAME}_server
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#  ${catkin_EXPORTED_TARGETS})
 
 
 
# DP test client
add_executable(${PROJECT_NAME}_test_client
 src/dp_action_test_client.cpp
)

target_link_libraries(${PROJECT_NAME}_test_client
#${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_test_client
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)



# CFG server
add_executable(${PROJECT_NAME}_cfg_server
 src/cfg_server.cpp
)

target_link_libraries(${PROJECT_NAME}_cfg_server
#${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_cfg_server
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dp_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
