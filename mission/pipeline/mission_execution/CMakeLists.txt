cmake_minimum_required(VERSION 3.8)
project(mission_execution)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

include_directories(include)

# Build the plugin library with all nodes
add_library(mission_execution_plugin SHARED
  src/mission_execution_gate.cpp
  src/set_mission_phase.cpp
  src/start_waypoint_action.cpp
  src/stop_waypoint_action.cpp
  src/landmark_enum_polling.cpp
  src/landmark_converge.cpp
  src/search_pattern.cpp
  src/pipeline_following.cpp
  src/plugin_register.cpp
)

target_compile_definitions(mission_execution_plugin PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(mission_execution_plugin PUBLIC
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  vortex_msgs
  behaviortree_ros2
  behaviortree_cpp
)

target_link_libraries(mission_execution_plugin PUBLIC
  behaviortree_ros2::behaviortree_ros2
  behaviortree_cpp::behaviortree_cpp
)

# Build the executor executable
add_executable(mission_execution_executor
  src/mission_execution_executor.cpp
)

ament_target_dependencies(mission_execution_executor PUBLIC
  rclcpp
  behaviortree_ros2
)

target_link_libraries(mission_execution_executor PUBLIC
  behaviortree_ros2::behaviortree_ros2
)

# Install the plugin library
install(TARGETS mission_execution_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

# Install the executor
install(TARGETS mission_execution_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install behavior trees
install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

