# PID controller

The PID controller is defined

```math
\tau = -J_{q}^{\dagger}(K_p \tilde{\eta} + K_d \dot{\tilde{\eta}} + K_i \int^t_0 \tilde{\eta}(\tau)d\tau)
```

where:

- $\tilde{\eta} = \eta - \eta_d$ is the pose error (7D quaternion representation),
- $J_q$ is the quaternion Jacobian (7×6), and $J_q^{\dagger}$ is its (pseudo-)inverse,
- $K_p$, $K_d$, $K_i$ are 6×6 gain matrices.

## PID controller (pid_controller_dp)

This package implements a 6-DOF PID controller that operates on the
6-dimensional control vector
$$\tau = [X, Y, Z, K, M, N]^T$$
and uses a quaternion-based 7D pose representation for attitude.

## Build


This package is built as part of the workspace. From the workspace root:

```bash
colcon build --packages-select pid_controller_dp 
```

To run tests for this package only:

```bash
colcon test --packages-select pid_controller_dp && colcon test-result --verbose
```

## Usage (ROS 2 node)

The package provides a node `pid_controller_node` that subscribes to pose,
twist and guidance topics and publishes wrench (tau) commands.


- `topics.pose` (type: `geometry_msgs/PoseWithCovarianceStamped`) — vehicle pose input
- `topics.twist` (type: `geometry_msgs/TwistWithCovarianceStamped`) — velocity input
- `topics.guidance.dp` (type: `vortex_msgs/ReferenceFilter`) — desired states (pose/vecocity)
- `topics.wrench_input` (type: `geometry_msgs/WrenchStamped`) — output wrench

Parameters expose PID gains (Kp, Ki, Kd) as per-component values which are
assembled into diagonal gain matrices inside the node. See the node source
(`src/pid_controller_ros.cpp`) for parameter names.

## Examples

Start the node (after sourcing workspace):

1. Run the simulation

 ```bash
 ros2 launch stonefish_sim simulation.launch.py drone:=nautilus 
 ```
To test with navigation node, also set `mock_odom:=false`

2. Run the thrust allocation node and the joystick node

 ```bash
 ros2 launch stonefish_sim drone_sim.launch.py drone:=nautilus
 ```

3. Run the controller

 ```bash
 ros2 launch  pid_controller_dp pid_controller_dp.launch.py
 ```

### Joystick Control

#### Buttons

| Button | Mode | Description |
| ------ | ---- | ----------- |
| A | Manual | Direct wrench control — joystick axes apply force/torque immediately |
| B | Kill | Software killswitch — cuts all thruster commands |
| Y | Autonomous (reference model) | Guided control — joystick sets position/orientation targets via the reference filter |
| LB | — | Roll left |
| RB | — | Roll right |

#### Axes — directional mapping

| Stick / Trigger | Axis | Motion | Positive direction |
| --------------- | ---- | ------ | ------------------ |
| Left stick — vertical | surge | Forward / backward | Push up → forward |
| Left stick — horizontal | sway | Left / right | Push right → port (left) |
| RT − LT (triggers) | heave | Up / down | RT → ascend, LT → descend |
| Right stick — horizontal | yaw | Rotate left / right | Push right → turn left |
| Right stick — vertical | pitch | Nose up / down | Push up → nose up |
| LB / RB (shoulders) | roll | Bank left / right | LB → roll left, RB → roll right |

> **Note:** When plotting, the displayed axis and the actual command may not align because the plot uses the `odom` frame while the controller operates in the robot `body_frame`.

## Tuning

The `rqt_reconfigure` can be used to change the controller gains.

```bash
ros2 run rqt_reconfigure rqt_reconfigure 
```
