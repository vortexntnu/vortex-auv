# ROS Driver for Motion Capture Systems
![VICON Logo](http://www.awn.com/sites/default/files/styles/inline_medium/public/image/featured/1025139-vicon-delivers-motion-capture-innovations-siggraph-2015.jpg?itok=vsH7Prwo)

![QUALISYS Logo](https://cdn-content.qualisys.com/2021/01/qualisys-logo-530x.png)

This package contains ROS drivers for two different motion capture systems,**VICON** And **QUALISYS**.
**NOTE:** This fork focuses on the Qualisys ROS driver. Some alterations have been made to the Vicon package to ensure compatability with the mutual interface.

## License
For the VICON driver, we use the [offical SDK](http://www.vicon.com/products/software/datastream-sdk).

For the QUALISYS driver, we use the [Qualisys CPP SDK](https://github.com/qualisys/qualisys_cpp_sdk).

For the rest of the software, the license is Apache 2.0 wherever not specified.

## Compiling
This is a catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. Drivers for different motion capture system can be independently compiled.

```
cd your_work_space
catkin_make --pkg mocap_{sys} --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This will compile the drivers for `{sys}`

## Example Usage

**Common Parameters**

`server_address` (`string`)

IP address of the server of the motion capture system to be connected.

`server_base_port` (`string`, `default: 22222`)
The port used for setting up the connection with the motion capture server. 
Should be left to the default value.

`frame_rate` (`int`, `default: 0`)

The frame rate of the motion capture system. A value of 0 uses whatever framerate 
the motion capture server is transmiting at. If lower frame rates are requested the
server will drop frames to get close to the requested rate. For example, if the server
is capturing at 100 FPS and 80 FPS is requested, every other frame will be dropped 
resulting in 50 FPS. 

`max_accel` (`double`, `default: 10.0`)

The max possible acceleration which serves to construct the noise parameters.

`publish_tf` (`bool`, `default: false`)

If set to true, tf msgs for the subjects are published.

`fixed_frame_id` (`string`, `default: mocap`)

The fixed frame ID of the tf msgs for each subject. Note that the child frame id is automatically set to the name of the subject.

`udp_port` (`int`, `default: 0`)

The UDP port that the server will send data to. If set to 0 a random valid port is used. If set to -1 
TCP will be used instead of UDP.

`qtm_protocol_version` (`int`, `default: 18`)

The minor version of the QTM protocol that should be used (the only valid major version is 1).
Might have to be lowered for very old versions of QTM. should not be set lower than 8 due to a bug in 
the protocol version 1.7 and prior causing the rotations to be incorrectly searialized.  

`model_list` (`vector<string>`, `default: []`)

A vector of subjects of interest. Leave the vector empty if all subjects are to be tracked.

**Published Topics**

`/{mocap_sys}/{subject_name}/odom` (`nav_msgs/Odometry`)

Odometry message for each specified subject in `model_list`.

To be compatible with the name of the topics published of `vicon_odom` in [vicon repo of KumarRobotics](https://github.com/KumarRobotics/vicon), you can uncomment the following line in the launch file:
`<remap from="{mocap_sys}/{subject_name}/odom" to="/{subject_name}/odom">`

**Node**

`roslaunch {mocap_sys} {sys}.launch`

For example,

`roslaunch mocap_qualisys qualisys.launch`

A node for making comparisions between the mocap system and for example the internal odometry of a robot
is also provided
`roslaunch mocap_qualisys comp_qualisys.launch`
The settings will ofcourse have to be tweaked to suit your purpose. 

## FAQ

1. Will the msgs be delayed if the driver is handling several subjects?
   Possibly. It's not recommended to for example, try to run the node for two dozen bodies on a raspberry pie.
   The multithreading in earlier versions were removed since it often caused more overhhead than actual gain. 

2. How to calibrate the transformation between the subject frame (centered at the centroid of the markers) and the body frame of a robot?
   

## Bug Report

Open an issue.
