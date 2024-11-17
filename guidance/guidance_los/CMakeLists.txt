cmake_minimum_required(VERSION 3.8)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${guidance_los})

# Install Python executable scripts
install(
  PROGRAMS
    scripts/los_guidance_action_server.py
  RENAME
    los_guidance_action_server
  DESTINATION lib/${guidance_los}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${guidance_los}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
