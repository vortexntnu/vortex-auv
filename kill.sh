#!/bin/bash

echo "##KILLING NODES##"
source /opt/ros/noetic/setup.bash
rosnode kill -a
killall -9 roscore
killall -9 rosmaster
