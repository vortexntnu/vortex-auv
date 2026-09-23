cmake_minimum_required(VERSION 3.8)
project(landmark_targets)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(spdlog REQUIRED)

include_directories(include)

set(CORE_LIB_NAME ${PROJECT_NAME})
set(ROS_LIB_NAME ${PROJECT_NAME}_ros)

# ROS-free core: target geometry, state machine and course geometry.
add_library(${CORE_LIB_NAME} SHARED
  src/landmark_target.cpp
  src/geometry.cpp
  src/slalom.cpp
)
ament_target_dependencies(${CORE_LIB_NAME} PUBLIC Eigen3 vortex_utils)

# Thin ROS helpers: LandmarkTrack conversion and TF lookups.
add_library(${ROS_LIB_NAME} SHARED
  src/ros_helpers.cpp
)
target_link_libraries(${ROS_LIB_NAME} PUBLIC ${CORE_LIB_NAME})
ament_target_dependencies(${ROS_LIB_NAME} PUBLIC
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  vortex_msgs
  vortex_utils_ros
)

# Example: a scenario runner that uses the library the way BT nodes do.
add_executable(${PROJECT_NAME}_scenario_node examples/scenario_node.cpp)
target_link_libraries(${PROJECT_NAME}_scenario_node PRIVATE ${ROS_LIB_NAME} spdlog::spdlog)
ament_target_dependencies(${PROJECT_NAME}_scenario_node PUBLIC
  rclcpp rclcpp_action nav_msgs vortex_msgs vortex_utils_ros tf2_ros)
install(TARGETS ${PROJECT_NAME}_scenario_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${CORE_LIB_NAME} ${ROS_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 vortex_utils vortex_utils_ros vortex_msgs rclcpp tf2_ros
  tf2_geometry_msgs geometry_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
