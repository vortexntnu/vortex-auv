## Overview:
---
Contains the LQR controller package for the AUV Orca. The controller utilizes an LQR optimal controller (imported from the python control library), and controls pitch, yaw and surge. The controller is meant to traverse larger distances.

#### Tuning of Parameters:
 To tune parameters look at the config/param_velocity_controller_lqr.yaml file:


#### Launching the package:
```bash
1. inside ros2ws/colcon build --packages-select velocity_controller_lqr
```

```bash
2. Inisde ros2ws/: source install/setup.bash
```

```bash
3. ros2 launch velocity_controller_lqr velocity_controller_lqr.launch.py
```

#### Transitioning between states manually:
The ROS2 node is implemented using lifecycle nodes, which are managed externally by a lifecycle manager i.e a finite state machine. If you want to manually test the node do the following:

**From Unconfigured ---> Inactive**
```bash
ros2 lifecycle set /velocity_controller_lqr_node configure
```

**From Configured ---> Active**
```bash
ros2 lifecycle set /velocity_controller_lqr_node activate
```

**From Active ---> Inactive**
```bash
ros2 lifecycle set /velocity_controller_lqr_node deactivate
```

For the full state diagram you can refer to the figure below, sourced from the official ROS2 Documentation:
![image info](./figures/ros2_transition_diagram.png)


### Theory
---
