# Thruster Interface AUV

This package provides an interface for controlling the thrusters of orca converting forces values to pwm values. The mapping is based on a piecewise third order polynomial approximating the datasheet .csv table found in /resources. Values send via i2c protocol.

*NOTE: Possibility to extend the handling based on the current operating voltage exists, not implemented as for now.*

## Usage

```sh
source install/setup.bash
ros2 launch thruster_interface_auv thruster_interface_auv.launch.py
```

## Structure

### Nodes

1. `thruster_interface_auv_node.cpp` contains the main loop of the node

2. `thruster_interface_auv_ros.cpp` contains the implementation of the node dependent on ros2 creating a subscriber for thruster forces, a publisher for pwm values, and a driver for handling the mapping. Initialize everything extracting all the parameters from .yaml file found in `../auv_setup/config/robots/orca.yaml` and `/config/thruster_interface_auv_config.yaml`.

3. `thruster_interface_auv_driver.cpp` contains the pure .cpp implementation for the mapping, conversion for i2c data format, and sending those values.

### Topics

- *subscribe to:* `/thrust/thruster_forces  [vortex_msgs/msg/ThrusterForce]` -- array of forces to apply to each thruster.

- *publish:* `/pwm  [std_msgs/msg/Int16MultiArray]` -- pwm command value to apply that force.

## Config

1. Edit `../auv_setup/config/robots/orca.yaml` for thruster parameters like mapping, direction, pwm_min and max.
2. Edit `/config/thruster_interface_auv_config.yaml` for i2c bus and address, topic names and polynomial coefficients for the mapping.

## Contact

For questions or support, please contact [albertomorselli00@gmail.com](mailto:albertomorselli00@gmail.com).
