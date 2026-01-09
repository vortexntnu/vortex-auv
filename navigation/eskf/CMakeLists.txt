cmake_minimum_required(VERSION 3.8)
project(eskf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

if(NOT DEFINED EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/eskf.cpp
  src/eskf_ros.cpp
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
  vortex_utils
  spdlog
  fmt
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "ESKFNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME})

# --- THE FIX: INSTALL THE EXECUTABLE ---
install(TARGETS 
  ${LIB_NAME}
  ${PROJECT_NAME}_node  
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
