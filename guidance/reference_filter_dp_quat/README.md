## Reference filter (quaternion)

### Third-order reference filter

The underlying filter is the third-order model (Fossen, 2021):
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

The state is integrated using forward Euler: $x_{i+1} = x_i + \dot{x}_i \cdot dt$.

### Error-state formulation

This package avoids the singularity issues of Euler angles by using a quaternion-based error state.

**Nominal pose** — a `Pose` (position + quaternion) representing the current best estimate of the reference trajectory. This is the actual output.

**Error state** — the 18D filter state, where the first 6 elements are small errors relative to the nominal:
```math
x = \begin{bmatrix} \delta p \\ \delta \phi \\ \dot{\eta} \\ \ddot{\eta} \end{bmatrix} \in \mathbb{R}^{18}
```

| Indices | Meaning |
|---|---|
| `x[0:3]` | Position error $\delta p$ from nominal (meters) |
| `x[3:6]` | Orientation error $\delta \phi$ from nominal (rotation vector, radians) |
| `x[6:9]` | World-frame linear velocity (m/s) |
| `x[9:12]` | World-frame angular velocity (rad/s) |
| `x[12:18]` | Accelerations (linear + angular) |

### Step cycle

Each time step performs three operations:

1. **Compute reference error** — The 6D reference $r$ is the error between the waypoint goal and the nominal pose:
   - $r_{0:3} = p_{goal} - p_{nominal}$
   - $r_{3:6} = \text{quaternion\_error}(q_{nominal},\ q_{goal})$

   The `quaternion_error` function returns $2 \cdot \text{vec}(q_{nominal}^{-1} \otimes q_{goal})$, which for small angles approximates the rotation vector from nominal to goal.

2. **Integrate** — The standard filter step:
   ```math
   \dot{x} = A_d x + B_d r, \quad x \leftarrow x + \dot{x} \cdot dt
   ```

3. **Reset** — Absorb position and orientation errors into the nominal pose, then zero them:
   - $p_{nominal} \leftarrow p_{nominal} + \delta p, \quad \delta p \leftarrow 0$
   - $q_{nominal} \leftarrow q_{nominal} \otimes \exp(\delta \phi), \quad \delta \phi \leftarrow 0$

   where $\exp(\delta \phi)$ converts the rotation vector to a quaternion via `AngleAxis`.

The reset step keeps $\delta p$ and $\delta \phi$ near zero at all times, ensuring the linearized quaternion error remains accurate. The velocity and acceleration states ($x_{6:18}$) are **not** reset — they carry over to provide smooth, continuous motion.

### Output message (`ReferenceFilterQuat`)

| Field | Source | Meaning |
|---|---|---|
| `x`, `y`, `z` | Nominal pose position | Desired position in world frame (m) |
| `qw`, `qx`, `qy`, `qz` | Nominal pose quaternion | Desired orientation (unit quaternion) |
| `x_dot`, `y_dot`, `z_dot` | `x[6:9]` | World-frame linear velocity (m/s) |
| `roll_dot`, `pitch_dot`, `yaw_dot` | `x[9:12]` | World-frame angular velocity (rad/s) |




### Waypoint modes

Each waypoint has a mode that determines which degrees of freedom are controlled by the reference filter. The mode also determines how convergence is measured.

| Mode | Controlled DOFs | Convergence metric |
|---|---|---|
| `FULL_POSE` | All 6 DOF (position + orientation) | Position error norm + quaternion error norm |
| `ONLY_POSITION` | x, y, z (orientation holds current value) | Position error norm |
| `FORWARD_HEADING` | x, y, z + yaw toward target | Position error norm + yaw component of quaternion error |
| `ONLY_ORIENTATION` | Orientation only (position holds current value) | Quaternion error norm |

For all modes, convergence is reached when the error metric drops below the `convergence_threshold` specified in the action goal.

### Action Server

The action server handles goal requests and publishes guidance commands. The server always prioritizes new goal requests, aborting ongoing requests when a new one arrives.

- Action name: `/reference_filter`
- Goal type: Waypoint (pose + mode + convergence threshold)
- Result type: bool
- Guidance topic: `/dp/reference`

### Overwriting the reference during an action

While an action is executing, the reference goal can be updated by publishing a `geometry_msgs/msg/PoseStamped` to the reference pose topic. The convergence check will use the updated reference.
