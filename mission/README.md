## Vortex NTNU, Manta AUV repository

## How to run the mission ##
-------------------------

1. Open a window and run dp-controller:
	```bash
	$ roslaunch vortex dp_control.launch 
	```
2. Open a window and change the controller mode: 
	```bash
	$ rosrun finite_state_machine service_client.py
	```
3. Open a third window and run:
	```bash
	$ rosrun finite_state_machine patrol_fsm.py
	```



