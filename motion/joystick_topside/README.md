## Joystick topside

#### Subscribes to:
* /joystick/joy

#### Publishes to:
* /joystick/input

This package contains a mapping between the joystick raw data and what we consider to be "inputs". "Inputs are defined as 
movement of the joysticks in all 6 DOFs.

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
This interface is to be launched on the topside computer that the Xbox-controller is connected to. Note that you have
to be on the same network as the drone (via the ethernet tether) for things to work.