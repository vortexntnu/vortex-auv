# PID controller

The controller operates on a **6D quaternion error state** — the scalar part $\tilde{q}_w$ is dropped and only the vector part is retained, keeping all matrices square (6×6).

The control law is

```math
\tau = -\left(K_p\, e_b + K_d\, \dot{e}_b + K_i \int^t_0 e_b\, d\tau\right)
```

where the body-frame error and velocity error are

```math
e_b = J^{-1}\,\tilde{\varepsilon}, \qquad
\dot{e}_b = \nu - J^{-1}\,\dot{\eta}_d
```

**Error state** $\tilde{\varepsilon} \in \mathbb{R}^6$:

```math
\tilde{\varepsilon} = \begin{bmatrix} \Delta p \\ \varepsilon_q \end{bmatrix}, \quad
\Delta p = p - p_d \in \mathbb{R}^3, \quad
\varepsilon_q = \begin{bmatrix} \tilde{q}_x \\ \tilde{q}_y \\ \tilde{q}_z \end{bmatrix}
```

where $\tilde{q} = q \ominus q_d$ is the quaternion error. The sign convention $\tilde{q}_w \geq 0$ is enforced to guarantee the shortest-path rotation.

**6×6 Jacobian** $J = \operatorname{blockdiag}(R,\, T_{33})$:

- $R \in SO(3)$ — rotation matrix from body to world frame.
- $T_{33} \in \mathbb{R}^{3 \times 3}$ — lower three rows of the quaternion kinematic matrix mapping body angular rates to $\dot{\varepsilon}_q$.

**Inverse Jacobian** $J^{-1} = \operatorname{blockdiag}(R^\top,\, I_3)$:

The exact inverse of $T_{33}$ is approximated by $I_3$. When the Jacobian is near-singular the right Moore–Penrose pseudoinverse is used instead.


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
 ros2 launch stonefish_sim simulation.launch.py scenario:=default
 ```

2. Run the thrust allocation node:

 ```bash
 ros2 launch thrust_allocator_auv thrust_allocator_auv.launch.py
 ```

3. To move the robot, run the joystick node

 ```bash
 ros2 launch stonefish_sim orca_sim.launch.py
 ```

4. Run the controller

 ```bash
 ros2 launch  pid_controller_dp pid_controller_dp.launch.py
 ```

Use the joy stick to move the robot. The key mappings are:

- B - kill
- Y - autonomous mode (reference model)
- A - manual mode

Note: When plotting, the axis plotted and actual command might not align since the plotting is based on the joy controller frame (`odom`), whereas the controller works on the robot frame (`body_frame`)

## Tuning

The `rqt_reconfigure` can be used to change the controller gains.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
