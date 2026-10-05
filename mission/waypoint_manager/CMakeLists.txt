cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

set(LIB_NAME "${PROJECT_NAME}_component")
set(CORE_LIB_NAME "${PROJECT_NAME}_core")

# ROS-free logic (testable without rclcpp)
add_library(${CORE_LIB_NAME} SHARED
  src/frame_resolver.cpp)

ament_target_dependencies(${CORE_LIB_NAME} PUBLIC
  vortex_utils
)

add_library(${LIB_NAME} SHARED
  src/waypoint_manager_ros.cpp)

target_link_libraries(${LIB_NAME} PUBLIC ${CORE_LIB_NAME})

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  rclcpp_action
  std_msgs
  geometry_msgs
  vortex_msgs
  vortex_utils_ros
  tf2_geometry_msgs
  spdlog
  fmt
)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "WaypointManagerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME} ${CORE_LIB_NAME}
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
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
