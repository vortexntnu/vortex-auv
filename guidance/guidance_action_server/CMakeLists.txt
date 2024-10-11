cmake_minimum_required(VERSION 3.5)
project(guidance_action_server)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  'action/NavigateWaypoints.action'
  DEPENDENCIES geometry_msgs action_msgs std_msgs
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
