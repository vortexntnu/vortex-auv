cmake_minimum_required(VERSION 3.0.2)
project(dp_reference_model)




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
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)





 add_executable(${PROJECT_NAME}_node
  src/dp_reference_model_node.cpp
  src/reference_model.cpp
)



 target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)