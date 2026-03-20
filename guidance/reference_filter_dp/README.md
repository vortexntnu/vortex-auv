## Reference filter

The reference filter (or reference model) is of third order and provides reference for pose, velocity and acceleration. So that $x_d := [\eta_d^\top, \dot{\eta}_d^\top, \ddot{\eta}_d^\top] \in \mathbb{R}^{3n}$.

The state-space representation is, according to (Fossen, 2021),
```math
\dot{x}_d = A_d x_d + B_d r
```
where
```math
A_d = \begin{bmatrix}
0_{n\times n} & I_n & 0_{n\times n} \\
0_{n\times n} & 0_{n \times n} & I_n \\
-\Omega^3 & -(2 \Delta + I_n) \Omega^2 & -(2 \Delta + I_n) \Omega
\end{bmatrix}
```
and
```math
B_d = \begin{bmatrix}
0_{n \times n} \\
0_{n \times n} \\
\Omega^3
\end{bmatrix}.
```

The steady-state pose for a constant reference signal $r$ is
```math
\lim_{t \to \infty} \eta_d = r
```

The state at the next time step is calculated using forward Euler integration
```math
x_{i+1} = x_i + \dot{x}_i * dt
```

## Waypoint modes

Each waypoint has a mode that determines which degrees of freedom are controlled by the reference filter. The mode also determines how convergence is measured.

| Mode | Controlled DOFs | Convergence metric |
|---|---|---|
| `FULL_POSE` | All 6 DOF (position + orientation) | Euclidean norm of position and angle errors |
| `ONLY_POSITION` | x, y, z (orientation holds current value) | Euclidean norm of position error |
| `FORWARD_HEADING` | x, y, z + yaw toward target (roll/pitch = 0) | Euclidean norm of position error and yaw error |
| `ONLY_ORIENTATION` | roll, pitch, yaw (position holds current value) | Euclidean norm of angle errors |

For all modes, convergence is reached when the error metric drops below the `convergence_threshold` specified in the action goal.

## Action Server

The action server is responsible for handling goal requests and publishing guidance commands. The server will always prioritize new goal requests, and will abort ongoing requests when getting a new request. The action definition can be found [here](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action).

- Action name: /reference_filter
- Goal type: PoseStamped
- Result type: bool
- Guidance topic: /dp/reference

### Overwriting the reference during an action

While an action is executing, the reference goal can be updated at any time by publishing a `geometry_msgs/msg/PoseStamped` to the reference pose topic. This allows external nodes to adjust the target pose mid-action without canceling and resending the goal. The convergence check will use the updated reference, so the action completes when the vehicle reaches the latest reference.
