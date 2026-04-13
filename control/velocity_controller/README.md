# velocity_controller

ROS2 lifecycle node for velocity control of an AUV (autonomous underwater vehicle). Supports two control strategies — a PID controller and an LQR controller.

---

## Overview

The package implements a `Velocity_node` that subscribes to odometry and guidance inputs, computes thrust commands, and publishes them as `WrenchStamped` messages. The node is managed as a ROS2 lifecycle node, meaning it can be managed by a lifecycle manager, however if you do not want to use a lifecycle manager you can change the parameter autostart in the parameter file so that it automaticly goes into active state.

The LQR controller linearizes the vehicle dynamics around the current state at each timestep (gain-scheduled LQR), using a body-frame model that includes linear hydrodynamic damping, Coriolis effects, and integral action for steady-state error rejection. The PID controller serves as a simpler backup.

---

## Dependencies

| Dependency | Purpose |
|---|---|
| `rclcpp` / `rclcpp_lifecycle` | ROS2 node and lifecycle management |
| `Eigen3` | Matrix math for LQR |
| `control_toolbox` (`ct::optcon`) | Riccati equation solver for LQR gain |
| `CasADi` | Used in utilities (NMPC-related) |
| `vortex_msgs` | Custom guidance message (`LOSGuidance`) |
| `nav_msgs` | Odometry input |
| `geometry_msgs` | Thrust output (`WrenchStamped`) |

---

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `topic_thrust` | `geometry_msgs/WrenchStamped` | Published | Force and torque commands to thruster allocator |
| `topic_guidance` | `vortex_msgs/LOSGuidance` | Subscribed | Desired surge, pitch, yaw from guidance system |
| `topic_odometry` | `nav_msgs/Odometry` | Subscribed | Current vehicle state from state estimator |

Topic names are configurable via ROS2 global parameter file.

---

## Parameters

All parameters are loaded in the constructor via `get_new_parameters()`.

### Controller selection

| Parameter | Type | Description |
|---|---|---|
| `controller_type` | `int` | `1` = PID, `2` = LQR |
| `publish_rate` | `int` | Control loop frequency in Hz |
| `max_force` | `double` | Saturation limit applied to all outputs (N / Nm) |

### LQR parameters

| Parameter | Type | Size | Description |
|---|---|---|---|
| `Q` | `double[]` | 8 | Diagonal of state weight matrix. States: `[surge_err, pitch_err, yaw_err, pitch_rate_err, yaw_rate_err, ∫surge, ∫pitch, ∫yaw]` |
| `R` | `double[]` | 3 | Diagonal of input weight matrix. Inputs: `[Fx, Ty, Tz]` |
| `inertia_matrix` | `double[]` | 36 | Row-major 6x6 rigid body inertia matrix |
| `dampening_matrix_low` | `double[]` | 36 | Row-major 6x6 hydrodynamic damping matrix at low speed |

### PID parameters

Each axis (surge, pitch, yaw) takes a parameter vector of the form `[Kp, Ki, Kd]`.

| Parameter | Description |
|---|---|
| `surge_params` | PID gains for surge |
| `pitch_params` | PID gains for pitch |
| `yaw_params` | PID gains for yaw |

### Behaviour flags

| Parameter | Type | Description |
|---|---|---|
| `reset_on_new_ref` | `bool` | Reset integrators when a new guidance reference arrives |
| `anti_overshoot` | `bool` | Enable anti-overshoot logic |
| `odometry_dropout_guard` | `bool` | Stop publishing if odometry stops arriving |
| `auto_start` | `bool` | If true, node self-transitions to active on startup |


---

## Lifecycle states

The node uses the standard ROS2 managed lifecycle:

```
Unconfigured → [configure] → Inactive → [activate] → Active
                                       ← [deactivate] ←
```

If `auto_start` is set to `true` in the parameters, the node will automatically call configure and activate itself after startup without needing an external lifecycle manager.

To manually manage the node:

```bash
# Configure
ros2 lifecycle set /velocity_controller configure

# Activate
ros2 lifecycle set /velocity_controller activate

# Deactivate
ros2 lifecycle set /velocity_controller deactivate

# Cleanup
ros2 lifecycle set /velocity_controller cleanup

# Shutdown
ros2 lifecycle set /velocity_controller shutdown

```

---

## Controller details

### LQR

The LQR controller uses an 8-state augmented model in the body frame:

```
x = [surge_err, pitch_err, yaw_err, pitch_rate_err, yaw_rate_err, ∫surge, ∫pitch, ∫yaw]
u = [Fx, Ty, Tz]
```

The system matrix `A` is re-linearized around the current state every control timestep. Guidance references in NED are converted to body-frame errors using the rotation matrix method before being passed to the controller — not by angle subtraction.

The gain `K` is computed by solving the continuous-time algebraic Riccati equation via `ct::optcon::LQR`. The control law is:

```
u = K * x_error
```

where `ct::optcon` produces `K` such that this is equivalent to `u = -K * (x - x_ref)`.

If the Riccati solver fails (e.g. due to an unstabilizable operating point), the node automatically falls back to PID and logs an error.

### PID

Three independent PID controllers run on surge, pitch, and yaw. Each supports anti-windup via integrator clamping. The derivative term can be computed either from the error signal or from a separately provided error derivative, depending on which `calculate_thrust` overload is called.

---

## Building

```bash
colcon build --packages-select velocity_controller --symlink-install
source install/setup.bash
```
or with

```bash
colcon build --packages-up-to velocity_controller --symlink-install
source install/setup.bash
```
Done in root of workspace
---

## Running

Via a launch file with a parameter file:

```bash
ros2 launch velocity_controller velocity_controller.launch.py
```

---

## Tests
There are system tests and a helper node that generates a reference for the controller to follow.
Tests are build like this:
```bash
colcon build --packages-select velocity_controller --symlink-install --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```
System tests are run with the command

```bash
colcon test
```

Helper node is run with

```bash
ros2 launch velocity_controller VCnTest.launch.py
```

## Notes for new team members

- The guidance input is expected in NED frame (north-east-down). The controller handles the NED-to-body conversion internally.
- All angle errors are wrapped to `[-π, π]` using `ssa()` (smallest signed angle) before being fed to the controller.
- The LQR Q matrix ordering matters — the 8 diagonal values correspond exactly to `[surge_err, pitch_err, yaw_err, pitch_rate_err, yaw_rate_err, ∫surge, ∫pitch, ∫yaw]` in that order.
- If the vehicle behaves oddly, check that `interval_` (the control timestep) is being set correctly — a value of `0` disables integral action silently.

## Adding new controllers
After adding the hpp file, add the calculation to calc_thrust function in a new switch case, add to the reset_controller function, with options to reset only one integral, lastly update documentation. Remeber to intialize correctly, either in 'on_configure' or in constructor, add the appropriate parameters, and update alle the {drone}_params.yaml files.

## Adding new drones
Copy a {drone}_params.yaml file and change the name to the new name of the drone. Add the appropriate matrices, and tune to satisfying behaviour.