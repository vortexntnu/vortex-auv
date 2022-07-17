#!/bin/bash

# The purpose of this file is to automate launching of 
# all nodes found in this repository FOR ROBOSUB 2022

roslaunch auv_setup beluga.launch
roslaunch finite_state_machine robosub.launch