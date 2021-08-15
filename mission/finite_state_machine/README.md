# Finite State Machine

## Subscribes to:
* odometry/filtered
* object_positions_out

## Action servers
* guidance/move

## Package Description

The finite state machine determines the execution of the AUV's mission. The AUV's mission behaviour is divided into different states, each with one or more tasks to complete, depending on the mission specifics. The state machine is implemented with the python library SMACH. The state machine script which is under development is scripts/simulator_state_machine.py. Its inputs are the object positions published by the landmarks node as well as the odometry data being published on the odometry/filtered. The state machine controls the movement of the AUV by sending goals containing the target position to the guidance/move-action server located in the guidance interface.

## Scripts
 - **simulator_state_machine.py:** The main state machine script which is under development.
 - **four_corner_mission.py:** State machine script intended for testing/data collection.
 - **fsm_helper.py:** Contains functions used by the state machine scripts, mainly los_move() and dp_move()
 - **goal_pose_helper.py:** ROS-Node which publishes a hard coded position message (used for state machine testing).

## src/sm_classes
Contains classes which will be the code running in some of the states.

## tests
Unit test scripts for state machine development (work in progress)

## Overview of Robosub 2021 state machine
todo

## Helper code

In addition to the state machines, this package contains a helper script, 'fsm_helper'. This makes the communication with the guidance interface
easier and the state-machine code simpler. It takes care of the action server/client and supplies *guidancemode*_move() functions for use in the state machine code. It (currently) also implements any other specific logic required to execute a task.


## How to run a mission 

### in simulator

1. Open a window and start gazebo:
	```bash
	$ roslaunch simulator_launch <world_of_choice>.launch
	```

2. Open another window and lauch nodes neccessary for operating auv:
	```bash
	$ roslaunch auv_setup auv.launch type:=simulator 
	```

3. Open a third window and lauch the desired state machine script: 
	```bash
	$ roslaunch finite_state_machine <state_machine_of_choice>.launch
	```

### on AUV

1. Lauch nodes neccessary for operating auv:
	```bash
	$ roslaunch auv_setup auv.launch type:=real
	```

1. Lauch the desired state machine (eg. pooltest): 
	```bash
	$ roslaunch finite_state_machine <state_machine_of_choice>.launch
	```
