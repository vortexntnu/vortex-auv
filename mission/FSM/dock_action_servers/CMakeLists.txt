cmake_minimum_required(VERSION 3.5)
project("dock_action_servers")

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
    dock_action_servers/fsm_state_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
