## AUV setup

This package contains global configs and the main launchfiles. The reason for putting these in a separate package is to be able to reach the aforementioned files from anywhere in a simple manner.

### Config
The config folder contains physical parameters related to the AUV and the environment

* environments: Atmosphere, water density, gravity etc..
* robots: Any and all physical constants directly related to the AUV body
* thrusters: Thruster configs for different thruster types

### Launch
This package contains a launchfile for each specific AUV. Additionally the pc.launch file is to
be used on the topside computer that the joystick is connected to, for ROV operations.

For the AUV launchfiles, the following parameters can be used:

| Parameter | Options         | Default   |
| ----------|-----------------|-----------|
| type      | real, simulator | simulator |

#### pc.launch
The pc.launch file is to be run on the topside computer, which needs to be connected to the same network as the drone.
The drone will be the master, and the topside computer needs to know how to connect to the drone. To configure this, do the following:

1. Find the IP of the Xavier. When running the Xavier on the drone, this should be `10.42.0.1`.
2. On the topside computer, execute
```echo "source vortex_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://X.X.X.X:11311" >> ~/.bashrc
```
where X.X.X.X is the IP of the Xavier.

3. Source the newly edited file.
```
source ~/.bashrc
```

### Sensors
Currently, the AUV launchfiles also contain the sensor driver launches, including the remapping of them:

#### Manta sensor mapping
| Sensor   | x     | y      | z      | u       | v       | w |
| ---------|-------|--------|--------|---------|---------|---|
| IMU      | 0     | 0      | 0      | 3.12159 | 0       | 0 |
| DVL      |-0.035 |-0.017  |-0.211  | 3.14159 | 3.14159 | 0 |
| Pressure | 0     | 0      | 0      | 0       | 0       | 0 |

#### Gladlaks sensor mapping
| Sensor   | x     | y      | z      | u       | v       | w    |
| ---------|-------|--------|--------|---------|---------|------|
| IMU      | 0     | 0      | 0      | TODO    | TODO    | TODO |
| DVL      |TODO   |TODO    |TODO    | TODO    | TODO    | 0    |
| Pressure | 0     | 0      | 0      | 0       | 0       | 0    |
