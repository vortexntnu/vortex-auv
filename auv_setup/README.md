## AUV setup

This package contains global configs and the main launchfiles. The reason for putting these in a separate package is to be able to reach the aforementioned files from anywhere in a simple manner.

### Config
The config folder contains physical parameters related to the AUV and the environment

* environments: Atmosphere, water density, gravity etc..
* robots: Any and all physical constants directly related to the AUV body
* thrusters: Thruster configs for different thruster types

### Launch
This package contains a launchfile for each specific AUV. Additionally the topside.launch file is to
be used on the topside computer that the joystick is connected to, for ROV operations.

### Description
The description folder contains the URDF and xacro files for the AUVs. The main description launch file is drone_description.launch.py, which makes all static transforms available to the ros graph.
