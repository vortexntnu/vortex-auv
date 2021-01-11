## Microcontroller interface
This node acts as an interface between the ROS-based system and an on-board multi-purpose microcontroller.
Functionality includes:
* Arming thrusters
* Controlling light
* Setting thruster speed
* Reading leak detection sensors

The MCU is addressed from the '/mcu/device' parameter, defaulting to '/dev/ttySAC0', and can be accessed
by serial read/write.
