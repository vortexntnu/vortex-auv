cmake_minimum_required(VERSION 2.8.3)
project(sensor_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  vortex_msgs
  message_generation
  roslint
)

add_service_files(FILES
  SaveImuCalibration.srv
  LoadImuCalibration.srv
)

roslint_python(scripts/bno055_interface.py  scripts/ms5837_interface.py)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  rospy
  vortex_msgs
  std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
