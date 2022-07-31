#!/bin/bash

# The purpose of this file is to automate launching of 
# all nodes found in this repository FOR ROBOSUB 2022

echo "##Activate thruster##"
./activate_thrusters.sh & 
sleep 5s
echo "##Starting FSM##"
roslaunch finite_state_machine robosub_fsm.launch
