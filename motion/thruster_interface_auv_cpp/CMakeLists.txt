cmake_minimum_required(VERSION 3.5)
project(thruster_interface_auv_cpp)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_node
  src/thruster_interface_auv_node.cpp
  src/thruster_interface_auv_driver.cpp
  src/thruster_interface_auv_ros.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  vortex_msgs
  ament_index_cpp
  Eigen3
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# # Define the driver library
# add_library(thruster_interface_auv_driver
#   src/thruster_interface_auv_driver.cpp
# )
# target_include_directories(thruster_interface_auv_driver PRIVATE include)
# ament_target_dependencies(thruster_interface_auv_driver rclcpp std_msgs vortex_msgs ament_index_cpp Eigen3)

# # Define the ROS interface library
# add_library(thruster_interface_auv_ros
#   src/truster_interface_auv_ros.cpp
# )
# target_link_libraries(thruster_interface_auv_ros thruster_interface_auv_driver)
# target_include_directories(thruster_interface_auv_ros PRIVATE include)
# ament_target_dependencies(thruster_interface_auv_ros rclcpp std_msgs vortex_msgs ament_index_cpp Eigen3)

# # Define the executable for the ROS node
# add_executable(thruster_interface_auv_node
#   src/thruster_interface_auv_node.cpp
# )
# target_link_libraries(thruster_interface_auv_node thruster_interface_auv_ros)
# ament_target_dependencies(thruster_interface_auv_node rclcpp std_msgs vortex_msgs ament_index_cpp Eigen3)

# # Install targets
# install(TARGETS
#   thruster_interface_auv_driver
#   thruster_interface_auv_ros
#   thruster_interface_auv_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# # Install headers
# install(DIRECTORY include/
#   DESTINATION include/
# )

# # Install launch files
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Install config files (coeffs.yaml)
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/config
# )

ament_package()
