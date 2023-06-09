cmake_minimum_required(VERSION 3.0.2)
project(topside_launch)

find_package(catkin REQUIRED COMPONENTS
  joystick_interface
  task_manager
  waypoint_interface
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
