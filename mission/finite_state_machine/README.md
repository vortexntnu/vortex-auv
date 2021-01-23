# Finite State Machine

## Subscribes to:
* odometry/filtered
* object_positions_out

## Action servers
* guidance/move

## Package Description

The finite state machine determines the execution of the AUV's mission. The AUV's mission behaviour is divided into different states, each with one or more tasks to complete, depending on the mission specifics. The state machine is implemented with the python library SMACH. The state machine script which is under development is scripts/simulator_state_machine.py. Its inputs are the object positions published by the landmarks node as well as the odometry data being published on the odometry/filtered. The state machine controls the movement of the AUV by sending goals containing the target position to the guidance/move-action server located in the guidance interface.

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

3. Open a third window and lauch the desired state machine script: 
	```bash
	$ roslaunch finite_state_machine simulator_state_machine.launch
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