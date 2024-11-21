cmake_minimum_required(VERSION 3.8)
project(vtf_guidance)


# Find dependencies
find_package(ament_cmake_python REQUIRED) # For Python support
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED) # ROS 2 dependencies
find_package(std_msgs REQUIRED)


# Install Python package and scripts
ament_python_install_package(${PROJECT_NAME}) # Installs the Python module

# Install Python scripts (if any)
install(
  PROGRAMS
  vtf_guidance/vtf_guidance.py # Add your Python script here
  DESTINATION lib/${PROJECT_NAME}
)


# Mark the package as complete
ament_package()
