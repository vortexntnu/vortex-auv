## Vortex AUV software - Vortex NTNU
[![Website shields.io](https://img.shields.io/website-up-down-green-red/http/shields.io.svg)](http://vortexntnu.no)
[![version](https://img.shields.io/badge/version-1.0.0-blue)](https://GitHub.com/vortexntnu/manta-auv/releases/)
![ViewCount](https://views.whatilearened.today/views/github/vortexntnu/manta-auv.svg)
[![GitHub contributors](https://img.shields.io/github/contributors/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/contributors/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/vortexntnu/manta-auv/graphs/commit-activity)
[![GitHub pull-requests](https://img.shields.io/github/issues-pr/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![GitHub pull-requests closed](https://img.shields.io/github/issues-pr-closed/vortexntnu/manta-auv.svg)](https://GitHub.com/vortexntnu/manta-auv/pulls)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# *VORTEX BANNER*

This repo contains software for operating UUVs, developed by students at NTNU. The software is based on the ROS Melodic framework, and aims to be hardware independent. Although the main focus of Vortex is autonomous operation, this software stack supports both AUV and ROV operations.

## Overview
Provided below is a brief summary how the software stack is divided.
| Folder           |  Contents  |https://github.com/vortexntnu/Vortex-AUV
|------------------|--------------------------------------------------------------------------------------------------------------------------------|
| auv_setup        | Provides a wrapper for drone parameters and any other physical parameters, as well as the launchfiles for each specific drone. |
| manipulators     | Contains code related to drone manipulators, such as grippers. |
| mission          | Contains the state machine that dictates drone behavior. |
| motion           | Anything related to physical motion of the drone, such as guidance and control systems. |
| navigation       | This folder contains localization, mapping and path planning packages. | 
| object_detection | Contains packages for visually detecting known objects, and estimating their pose. |


A more detailed description of the system can be found [here](https://miro.com/app/board/o9J_lV3eIZc=/)

## Usage
Instructions for installation can be found [here](https://github.com/vortexntnu/Vortex-AUV/wiki/Software-installation)

To launch a drone, execute
```
roslaunch auv_setup <drone>.launch
```
to prepare the drone for operation.

Next,you have to choices:
For autonomous operation, execute the desired mission script:
 ```
roslaunch finite_state_machine <mission_script>.launch
```

For manual operation, execute the joystick nodes on the topside computer connected to the drone:
```
roslaunch auv_setup pc.launch
```

## Documentation
* TODO: Drivers and hardware specifics for each drone will be added to the wiki. Link them here.
* TODO: How to adapt the software stack to new hardware.
* A collection of master theses:
  *   [Manta v1: A Deliberative Agent SoftwareArchitecture for AutonomousUnderwater Vehicles]()
* Diagrams

