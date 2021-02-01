## Navigation

The navigation system contains all the packages involved in position/orientation estimation, pathplanning, odometry etc..
Currently, navigation is composed of the following packages:

### Estimation
<details>
<summary>robot_localization</summary>

* A package containing a set of  nonlinear state estimation nodes.
</details>

<details>
<summary>underwater_odom</summary>

* A package containing a node that collects pressure and dvl data and collects it in a common topic.
* Also calculates a depth estimate from the pressure sensor data.
</details>


### Path planning
<details>
<summary>path_guidance</summary>

* WIP
</details>


<details>
<summary>pathplanning</summary>

* WIP
</details>


### Guidance (To be moved to motion potentially)
<details>
<summary>inspect_point</summary>

* A package that implements a way for the AUV to inspect an unknown point, in search for recognizable objects.
* A package that is half-guidance and half-mission, really.
</details>

<details>
<summary>waypoint_action_server</summary>

* Currently an empty package.
</details>

<details>
<summary>waypoint_action_client</summary>

* A WIP of a module that would allow for guidance by set waypoints.
</details>

### Misc
<details>
<summary>navigation_launch</summary>

* Contains the launch file for the navigation system required by either the simulated or real AUV.
</details>

<details>
<summary>sensor_interface</summary>

* Currently not in use.
* A sensor interface for:
	* BNO055 IMU
	* MS5837 digital pressure sensor

</details>





#### How to run the waypoint client node (OUTDATED) ##
-------------------------
<details>
<summary>OUTDATED</summary>
Outdated, but kept here just in case.

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
</details>


