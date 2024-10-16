cmake_minimum_required(VERSION 3.8)
project(ekf_filtering)

# === C++ standard ===
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vortex_filtering REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/aruco_detector.cpp
  src/aruco_detector_ros.cpp
  src/aruco_file_logger.cpp
)

target_link_libraries(${LIB_NAME} PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  cv_bridge
  vortex_filtering
)

rclcpp_components_register_node(
    ${LIB_NAME}
    PLUGIN "vortex::aruco_detector::ArucoDetectorNode"
    EXECUTABLE ${PROJECT_NAME}_node
)

# Export the target for other packages to use
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
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

ament_package()