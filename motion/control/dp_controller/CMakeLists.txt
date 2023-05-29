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

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

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

# DP controller
add_executable(${PROJECT_NAME}_node
 src/dp_controller_node.cpp
 src/dp_controller_ros.cpp
 src/quaternion_dp_controller.cpp
 src/dp_action_server.cpp
 )

add_dependencies(${PROJECT_NAME}_node
 ${${PROJECT_NAME}_EXPORTED_TARGETS} 
 ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  #${${PROJECT_NAME}_EXPORTED_TARGETS}    #does not want to build with this
  ${catkin_LIBRARIES}
)
 
# DP test client
add_executable(${PROJECT_NAME}_client
 src/dp_action_client.cpp
)

target_link_libraries(${PROJECT_NAME}_client
${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}_client
${${PROJECT_NAME}_EXPORTED_TARGETS} 
${catkin_EXPORTED_TARGETS}
)

