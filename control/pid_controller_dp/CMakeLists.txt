cmake_minimum_required(VERSION 3.8)
project(pid_controller_dp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)
set(LIB_NAME ${PROJECT_NAME}_lib)

add_library(${LIB_NAME} SHARED
  src/pid_controller.cpp
  src/pid_controller_utils.cpp
  src/pid_controller_conversions.cpp
)

ament_target_dependencies(${LIB_NAME} PUBLIC
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
  rcl_interfaces
  vortex_utils
  spdlog
  fmt
)


install(TARGETS
  ${LIB_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(pid_controller_node
  src/pid_controller_node.cpp
  src/pid_controller_ros.cpp
)

ament_target_dependencies(pid_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  tf2
  vortex_msgs
  rcl_interfaces
  vortex_utils
  spdlog
  fmt
)

target_link_libraries(
    pid_controller_node 
    ${LIB_NAME}
    spdlog::spdlog
    # vortex_utilis::vortex_utils
)

ament_export_targets(export_${LIB_NAME})

install(TARGETS ${LIB_NAME}
  EXPORT export_${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  pid_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
