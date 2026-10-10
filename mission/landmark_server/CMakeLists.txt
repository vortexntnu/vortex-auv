cmake_minimum_required(VERSION 3.8)
project(landmark_server)

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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)

set(LIB_NAME "${PROJECT_NAME}_component")

add_library(${LIB_NAME} SHARED
  src/config.cpp
  src/graph.cpp
  src/association.cpp
  src/premap.cpp
  src/targets.cpp
  src/landmark_server_node.cpp)

target_include_directories(${LIB_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  vortex_msgs
  vortex_utils_ros
  tf2
  tf2_ros
  spdlog
  fmt
)
target_link_libraries(${LIB_NAME} PUBLIC gtsam yaml-cpp Boost::boost)

rclcpp_components_register_node(
  ${LIB_NAME}
  PLUGIN "vortex::landmark_server::LandmarkServerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch foxglove
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/competition_map_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
