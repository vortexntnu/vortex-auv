cmake_minimum_required(VERSION 2.8.3)
project(anomaly_detection)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure #message_generation
	
)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
	cfg/BatterySimulator.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES los_guidance
#  CATKIN_DEPENDS nav_msgs rospy sensor_msgs vortex_msgs
#  DEPENDS system_lib
)

