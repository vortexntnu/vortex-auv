# Finite State Machine

## Package Description

The finite state machine determines the execution of the AUV's mission. The AUV's mission behaviour is divided into different states, each with one or more tasks to complete, depending on the mission specifics. 

## Overview of Robosub 2020 state machine

todo

## How to run a mission 

### in simulator

1. Open a window and start gazebo:
	```bash
	$ roslaunch simulator_launch cybernetics_pool.launch
	```

2. Open another window and lauch nodes neccessary for operating auv:
	```bash
	$ roslaunch auv_setup manta_simulator.launch 
	```

3. Open a third window and lauch the state machine: 
	```bash
	$ roslaunch finite_state_machine simtest.launch
	```

### on AUV

1. Lauch nodes neccessary for operating auv:
	```bash
	$ roslaunch auv_setup manta_real.launch 
	```

1. Lauch the desired state machine (eg. pooltest): 
	```bash
	$ roslaunch finite_state_machine pooltest.launch
	```