cmake_minimum_required(VERSION 3.5)
project(guidance_action_server)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

# Generate the action interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateWaypoints.action"
  DEPENDENCIES geometry_msgs action_msgs std_msgs
)

# Install the Python package with a unique target name
ament_python_install_package(${PROJECT_NAME}_python 
  PACKAGE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/guidance_action_server)

# Package exports
ament_package()