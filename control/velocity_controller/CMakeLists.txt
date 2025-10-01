cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

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

add_executable(velocity_controller_node src/velocity_controller.cpp)
add_executable(test_VC_node src/test_VC.cpp)
target_include_directories(velocity_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS velocity_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS test_VC_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_target_dependencies(velocity_controller_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
)
ament_target_dependencies(test_VC_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
)
