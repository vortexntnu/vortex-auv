#!/bin/bash

echo "##AUV LAUNCH##"
source /opt/ros/noetic/setup.bash
source /home/ubuntu/vortex_ws/devel/setup.bash
roslaunch auv_setup beluga.launch
