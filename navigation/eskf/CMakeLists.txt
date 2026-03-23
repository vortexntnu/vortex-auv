cmake_minimum_required(VERSION 3.8)
project(eskf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

set(CORE_LIB_NAME "${PROJECT_NAME}")

add_library(${CORE_LIB_NAME} SHARED
  src/lib/eskf.cpp
)

target_include_directories(${CORE_LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${CORE_LIB_NAME}
  Eigen3
  vortex_utils
)

add_library(${PROJECT_NAME}::${CORE_LIB_NAME} ALIAS ${CORE_LIB_NAME})

set(COMPONENT_LIB_NAME "${PROJECT_NAME}_component")

set(LANDMARK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../landmark_egomotion")

add_library(${COMPONENT_LIB_NAME} SHARED
  src/ros/eskf_ros.cpp
  ${LANDMARK_DIR}/src/lib/landmark_egomotion.cpp
)

target_include_directories(${COMPONENT_LIB_NAME} PRIVATE
  ${LANDMARK_DIR}/include
)

ament_target_dependencies(${COMPONENT_LIB_NAME}
  rclcpp
  rclcpp_components
  Eigen3
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  vortex_msgs
  vortex_utils
  vortex_utils_ros
  spdlog
  fmt
)

target_link_libraries(${COMPONENT_LIB_NAME} ${PROJECT_NAME})

rclcpp_components_register_node(
  ${COMPONENT_LIB_NAME}
  PLUGIN "ESKFNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(TARGETS ${CORE_LIB_NAME} ${COMPONENT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY ${LANDMARK_DIR}/include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  Eigen3
  vortex_utils
)
ament_export_include_directories(include)
ament_export_libraries(${CORE_LIB_NAME})

ament_package()
