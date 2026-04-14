cmake_minimum_required(VERSION 3.8)
project(landmark_drift_correction)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

# Core library: DriftCorrector
set(CORE_LIB_NAME "${PROJECT_NAME}")

add_library(${CORE_LIB_NAME} SHARED
  src/lib/drift_corrector.cpp
)

target_include_directories(${CORE_LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${CORE_LIB_NAME} Eigen3::Eigen)

# Component library: ROS node + graph visualizer
set(COMPONENT_LIB_NAME "${PROJECT_NAME}_component")

add_library(${COMPONENT_LIB_NAME} SHARED
  src/ros/landmark_drift_correction_ros.cpp
  src/ros/graph_visualizer.cpp
)

ament_target_dependencies(${COMPONENT_LIB_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  visualization_msgs
  spdlog
)

target_link_libraries(${COMPONENT_LIB_NAME}
  Eigen3::Eigen
  ${CORE_LIB_NAME}
)

rclcpp_components_register_node(
  ${COMPONENT_LIB_NAME}
  PLUGIN "vortex::navigation::drift_correction::LandmarkDriftCorrectionNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS
    ${CORE_LIB_NAME}
    ${COMPONENT_LIB_NAME}
  EXPORT export_${PROJECT_NAME}
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_libraries(${CORE_LIB_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
