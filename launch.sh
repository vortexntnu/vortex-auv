#!/bin/bash

# The purpose of this file is to automate launching of 
# all nodes found in this repository FOR ROBOSUB 2022

echo "##AUV Setup##"
sleep 10s
roslaunch auv_setup beluga.launch & 
sleep 10s
echo "##Activate thruster##"
./activate_thrusters.sh & 
sleep 10s
echo "##Starting FSM##"
roslaunch finite_state_machine robosub_fsm.launch
