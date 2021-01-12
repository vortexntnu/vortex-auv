## DP controller
This package provices an implementation of _Fjellstad & Fossen 1994: Quaternion Feedback Regulation of Underwater Vehicles_,
a nonlinear PD position and orientation controller, which in this implementation has been expanded with integral effect.
It provides the AUV with the ability to hold fixed setpoints in pose, heading, depth, or a combination of these.

The core of the technical implementation lies in quaternion_pd_controller. controller_ros implements the interface layer between
the controller implementation with the rest of the ROS-based system.

The controller is currently used by the dp_guidance block to provide one of the following control modes:
* Open loop         
* Pose hold         
* Heading hold   
* Depth+Heading hold
* Depth hold      
* Pose+Heading hold

