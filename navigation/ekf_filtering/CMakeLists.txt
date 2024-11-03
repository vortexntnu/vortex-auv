cmake_minimum_required(VERSION 3.8)
project(ekf_filtering)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vortex_filtering REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(ekf_filtering_node 
src/ekf_filtering_ros.cpp)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_target_dependencies(ekf_filtering_node 
  rclcpp 
  geometry_msgs 
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  message_filters
  vortex_filtering
  std_srvs)

install(TARGETS
  ekf_filtering_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
