cmake_minimum_required(VERSION 3.5)
project("go_to_dock_action")

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # To generate the action interfaces

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToDock.action"                        # Add the path to your action file
  DEPENDENCIES builtin_interfaces
)

# Install Python nodes
install(
  PROGRAMS
    go_to_dock_action/go_to_dock_client.py 
    go_to_dock_action/go_to_dock_server.py        
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
