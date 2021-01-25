## Joystick guidance

#### Subscribes to:
* /joystick/joy

#### Publishes to:
* /manta/thruster_manager/input

This package contains a conversion from joystick input to thrust vector. In the future it may serve as a
way to 



## Setup
The joystick may require some configuring before use. Take special note of the **Configuration** step.

### Installation
```
sudo apt-get install ros-<ros_distro>-joy
```

### Configuration
Here is a TLDR of the full [joystick configuration guide](http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick):

1. Connect your joystick to your computer. It should then show up under `/dev/input/jsX`,
where X will be a number. The joystick can be tested with
```
sudo jstest /dev/input/jsX
```
2. Make the joystick accessible for the ROS joy node:
```
sudo chmod a+rw /dev/input/jsX
```
3. If your joystick is not `/dev/input/js0` you will need to edit the device parameter used by the joystick driver:
```
roscore
rosparam set joy_node/dev "/dev/input/jsX"
```

### Launch

The joystick guidance is only in use when the AUV is run in "ROV mode". Because of this, you will need to launch
the ROV specific top-level launchfile:

```
roslaunch auv_setup rov.launch
```

Additionally, you will not need to run any mission script to operate the drone.