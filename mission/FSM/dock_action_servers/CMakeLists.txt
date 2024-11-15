cmake_minimum_required(VERSION 3.5)
project("dock_action_servers")

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # To generate the action interfaces

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToDock.action"                        # Add the path to your action file
  "action/FindDock.action"
  DEPENDENCIES builtin_interfaces
)

# Install Python nodes
install(
  PROGRAMS
    dock_action_servers/go_to_dock_server.py
    dock_action_servers/find_dock_server.py
    dock_action_servers/dock_server.py
    dock_action_servers/return_home_server.py
    dock_action_servers/go_over_dock_server.py
    dock_action_servers/go_down_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
