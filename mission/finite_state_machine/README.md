# Finite State Machine

## Package Description

The finite state machine determines the execution of the AUV's mission. The AUV's mission behaviour is divided into different states, each with one or more tasks to complete, depending on the mission specifics. 

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

3. Open a third window and lauch the state machine: 
	```bash
	$ roslaunch finite_state_machine <state_machine_of_choice.launch>
	```

### on AUV

1. Lauch nodes neccessary for operating auv:
	```bash
	$ roslaunch auv_setup auv.launch type:=real
	```

1. Lauch the desired state machine (eg. pooltest): 
	```bash
	$ roslaunch finite_state_machine <state_machine_of_choice.launch>
	```