## Vortex NTNU, Manta AUV repository

## How to run the waypoint client node ##
-------------------------

1. Open a window and run Gazebo world, spawn Manta, thruster manager and navigation by executing: 
	```bash
	$ roslaunch waypoint_action_client load_waypoints_file.launch
	```

2. Open a second window and run dp-controller:
	```bash
	$ roslaunch vortex dp_control.launch 
	```
  
3. Open a third window and launch the path generator client.
	```bash
	$ roslaunch waypoint_action_client send_waypoints_file.launch
	```
## How to run the LOS guidance ##
-------------------------
1. Open a window and run the following line:
	```bash
	$ rosrun los_guidance los_guidance_euler.py
	```



