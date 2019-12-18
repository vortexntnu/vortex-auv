## Vortex NTNU, Manta AUV repository

![MANTA](docs/manta_v1.png)

## Prerequisites

Linux distributions such as Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8)<br />
C++ 11 compiler or newer.

## 1. Install ROS kinetic for Ubuntu (If you do not have it already) ##

###### This should take no more than 3 minutes. If you have another version of linux but Ubuntu, follow this guide: http://wiki.ros.org/kinetic/Installation. It is very important that you follow the installation guide and instructions on how to run the guide EXACTLY as stated or you will get running errors. Simply copy and paste the commands to your terminal command window ######

Robot operating system (ROS) provides services designed for heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management. The main ROS client libraries (C++, Python, and Lisp) are geared toward a Unix-like system, primarily because of their dependence on large collections of open-source software dependencies.


-------------------------

1. Setup your computer to accept software from packages.ros.org:
	```bash
	$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	```

2. Set up your keys:
	```bash
	$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
	```

3. Update:
	```bash
	$ sudo apt-get update
  
4. Installation:
	```bash
	$ sudo apt-get install ros-kinetic-desktop-full
	```

5. Before you can use ROS, you will need to initialize rosdep: 
	```bash
	$ sudo rosdep init
	$ rosdep update
  

6. Environment setup:
	```bash
	$ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	$ source ~/.bashrc

## 2. Install the necessary dependencies to interface with drivers, Gazebo etc. ##
-------------------------

Run the shell script for install dependencies:

	```bash
	$ sh install-prereq.sh

or install manually the remaining dependencies

1. Install the protobuf library, which is used as interface to Gazebo.:
	```bash
	$ sudo apt-get install protobuf-compiler

2. Install rosbridge-server to interface with sensor and actuator drivers on the physical Manta.
	```bash
	$ sudo apt-get install ros-kinetic-rosbridge-server
  
3. Install tf. tf is a package that lets the user keep track of multiple coordinate frames over time:
	```bash
	$ sudo apt-get install ros-kinetic-message-to-tf

4. Install tf. tf is a package that lets the user keep track of multiple coordinate frames over time:
	```bash
	$ sudo apt-get install ros-kinetic-geographic-msgs

5. Install move-base-msgs. This is necessary to perform some actions:
	```bash
	$ sudo apt-get install ros-kinetic-move-base
	$ sudo apt-get install ros-kinetic-move-base-msgs 

## 3. Now that you have ROS Kinetic installed. Create ROS workspace ##
###### This is necessary to be able to run the simulation package that I have created
-------------------------

1. creating a catkin workspace:
	```bash
	$ mkdir -p ~/manta_ws/src
	$ cd ~/manta_ws/src
	$ catkin_init_workspace
  
2. building the workspace created:
	```bash
	$ cd ~/manta_ws/
	$ catkin build
  
3. source the current workspace and Gazebo model:
	```bash
	$ echo "source manta_ws/devel/setup.bash" >> ~/.bashrc
	$ echo "export GAZEBO_MODEL_PATH=/home/youruser/manta_ws/src/manta_gazebo:$GAZEBO_MODEL_PATH" >> ~/.bashrc 
	$ echo "export ROS_PACKAGE_PATH=/home/youruser/manta_ws:$ROS_PACKAGE_PATH" >> ~/.bashrc

	
3. close the current window.

4. Open a new window. To make sure the workspace is properly overlayed:
	```bash
	$ echo $ROS_PACKAGE_PATH
	  /home/youruser/manta_ws/src:/opt/ros/kinetic/share 


## 4. Download and build Manta V1 ##
-------------------------
1. Enter the folder where you want to clone the repostory:
	```bash
	$ cd manta_ws/src
	```

2. Clone the repository: 
	```bash
	$  git clone https://github.com/vortexntnu/Manta-AUV.git
	$  git clone https://github.com/vortexntnu/vortex_msgs.git
	```
Ps. You can also manually download the zip-folder in the up-right corner and extract the file <br />
inside the src-folder of you workspace

3. Compile the code by running "catkin build" inside the workspace:
	```bash
	$ cd ~/manta_ws/
	$ catkin build vortex_msgs
	$ catkin build
  
## 5. Download and build the customized UUV simulator for Manta AUV ##
-------------------------

![MANTA](docs/manta_underwater_robosub.png)

1. Enter the folder where you want to clone the repostory:
	```bash
	$ cd manta_ws/src
	```

2. Clone the repository: 
	```bash
	$ git clone https://github.com/vortexntnu/uuv-simulator.git
	```

3. Clone the repository. WARNING: HIGH CPU LOAD, you might want to build packages separately the first time: 
	```bash
	$ catkin build
	```
## 6. Run Manta V1 in Simulation with Gazebo, Smach viewer, Camera pop-up windows etc ##
-------------------------

1. Run your simulation world. This will upload Manta (w/ sensor, camera, thrusters etc) and launch robot localization as well. i.e :
	```bash
	$ roslaunch simulator_launch cybernetics_pool.launch
	```

2. Execute your state machine of choice. This will activate all modules in Manta V1 architecture. i.e: 
	```bash
	$ roslaunch finite_state_machine gazebo_sm.launch
	```

## 7. Alternative: Run Manta V1 in Linux minimal on your drone without Gazebo, Smach viewer, Camera pop-up windows etc ##

![MANTA](docs/hardware.png)

1. The main computer for Manta AUV is a ODROID. Find the IP-address of the ODROID:
	```bash
	$ nmap 10.42.0.1/24
	```
2. SSH into the ODROID:
	```bash
	$ ssh root@10.42.*INSERT*
	```
3. Run the robot localization for Aided Inertial Navigation:
	```bash
	$ roslaunch robot_localization ekf_novembertest.launch
	```
4. ARM the thrusters (system specific). For the Manta AUV it will be:
	```bash
	$ rostopic pub /mcu_arm std_msgs/String "data: 'arm'"
	```
5. Run your state machine of choice. This will activate all modules in Manta V1 architecture. i.e:
	```bash
	$ roslaunch finite_state_machine odroid_sm
	```
6. DISARM the thrusters (system specific). For the Manta AUV it will be:
	```bash
	$ rostopic pub /mcu_arm std_msgs/String "data: 'ben'"
	```

