[![GitHub stars](https://img.shields.io/github/stars/vortexntnu/manta-auv.svg?style=social&label=Star&maxAge=2592000)](https://GitHub.com/vortexntnu/manta-auv/stargazers/)
[![GitHub watchers](https://img.shields.io/github/watchers/vortexntnu/manta-auv.svg?style=social&label=Watch&maxAge=2592000)](https://GitHub.com/vortexntnu/manta-auv/watchers/)
[![GitHub forks](https://img.shields.io/github/forks/vortexntnu/manta-auv.svg?style=social&label=Fork&maxAge=2592000)](https://GitHub.com/vortexntnu/manta-auv/network/)
## Manta AUV software - Vortex NTNU
[![Website shields.io](https://img.shields.io/website-up-down-green-red/http/shields.io.svg)](http://vortexntnu.no)
[![version](https://img.shields.io/badge/version-1.0.0-blue)](https://GitHub.com/vortexntnu/manta-auv/releases/)
![ViewCount](https://views.whatilearened.today/views/github/vortexntnu/manta-auv.svg)
[![GitHub contributors](https://img.shields.io/github/contributors/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/contributors/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/commit-activity)
[![GitHub pull-requests](https://img.shields.io/github/issues-pr/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![GitHub pull-requests closed](https://img.shields.io/github/issues-pr-closed/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

![MANTA](docs/manta_v1.png)


## Documentation
A Master's thesis explaining the project and features of the Manta software is found in the docs folder: https://github.com/vortexntnu/manta-auv/blob/master/docs/master_thesis_manta_v1_kristoffer_solberg_compressed.pdf

## Installation

For instructions on how to install Vortex-AUV and Vortex-Simulator check out the [wiki page](https://github.com/vortexntnu/Vortex-AUV/wiki/Software-installation) 

## Run Manta V1 in Simulation with Gazebo, Smach viewer, Camera pop-up windows etc ##
-------------------------

1. Run your simulation world. This will upload Manta (w/ sensor, camera, thrusters etc) and launch robot localization as well. i.e :
	```bash
	 roslaunch simulator_launch cybernetics_pool.launch
	```

2. Launch all modules required for operating Manta:
	```bash
	 roslaunch auv_setup auv.launch type:=simulator
	```

2. Execute your state machine of choice. i.e: 
	```bash
	 roslaunch finite_state_machine simtest.launch
	```

## Run Manta V1 in Linux minimal on your drone without Gazebo, Smach viewer, Camera pop-up windows etc ##

![MANTA](docs/hardware.png)

Figure by: Kristoffer Rakstad Solberg

1. The main computer for Manta AUV is a ODROID. Find the IP-address of the ODROID:
	```bash
	 nmap 10.42.0.1/24
	```
2. SSH into the ODROID:
	```bash
	 ssh root@10.42.*INSERT*
	```
3. Specify your static transforms and initial states in robot_localization/launch and robot_localization/params

![MANTA](docs/coordinate_frame.png)

Figure by: Kristoffer Rakstad Solberg

4. Run the robot localization for extended Kalman Filter for Aided Inertial Navigation:
	```bash
	 roslaunch robot_localization ekf_novembertest.launch
	```
5. ARM the thrusters (system specific). For the Manta AUV it will be:
	```bash
	 rostopic pub /mcu_arm std_msgs/String "data: 'arm'"
	```
6. Run your state machine of choice. This will activate all modules in Manta V1 architecture. i.e:
	```bash
	 roslaunch finite_state_machine odroid_sm
	```
7. DISARM the thrusters (system specific). For the Manta AUV it will be:
	```bash
	 rostopic pub /mcu_arm std_msgs/String "data: 'ben'"
	```

