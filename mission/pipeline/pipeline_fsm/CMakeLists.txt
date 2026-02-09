cmake_minimum_required(VERSION 3.8)
project(pipeline_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_viewer REQUIRED)
find_package(vortex_msgs REQUIRED)

include_directories(include)

add_executable(mission_controller_node
  src/mission_controller_node.cpp
  src/mock_states.cpp
  src/waypoint_manager_states.cpp
)

ament_target_dependencies(mission_controller_node
  rclcpp
  rclcpp_action
  std_srvs
  yasmin
  yasmin_ros
  yasmin_viewer
  vortex_msgs
)

install(TARGETS
  mission_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
