cmake_minimum_required(VERSION 3.0.2)
project(task_manager)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/task_manager.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)