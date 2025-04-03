cmake_minimum_required(VERSION 3.8)
project(dp_adapt_backs_controller)

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
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/dp_adapt_backs_controller.cpp
  src/dp_adapt_backs_controller_ros.cpp
  src/dp_adapt_backs_controller_utils.cpp)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  fmt
  spdlog
  vortex_msgs
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "DPAdaptBacksControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME})

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
