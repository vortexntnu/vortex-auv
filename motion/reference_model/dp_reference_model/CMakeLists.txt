cmake_minimum_required(VERSION 3.0.2)
project(dp_reference_model)


# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  tf
  tf_conversions
  vortex_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ReferenceModel.cfg
)

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
#   geometry_msgs#   vortex_msgs
# )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dp_reference_model
  CATKIN_DEPENDS
  eigen_conversions 
  geometry_msgs 
  roscpp 
  tf 
  tf_conversions 
  vortex_msgs
  dynamic_reconfigure
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)


# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 add_executable(${PROJECT_NAME}_node
  src/dp_reference_model_node.cpp
  src/reference_model.cpp
)


# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

