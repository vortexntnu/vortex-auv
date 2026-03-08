# 3D LOS Guidance Library (Read me draft)

This package implements several **Line-of-Sight (LOS) guidance algorithms** for **3D path following**.  
The guidance system computes the **desired yaw** $\psi_d$ and **desired pitch** $\theta_d$ so a vehicle can follow a path between waypoints.

The vehicle surge speed is kept **constant** during path following and is defined in the configuration file using the parameter

```
u_desired
```

This value represents the desired forward velocity of the vehicle.

---

# Implemented LOS Methods

The library supports four LOS guidance algorithms.

| Mode | Method |
|-----|------|
| 0 | Proportional LOS |
| 1 | Integral LOS |
| 2 | Adaptive LOS |
| 3 | Vector Field LOS |

The guidance method can be **changed during runtime** using a ROS service.

---

# Adaptive LOS (ALOS)

Adaptive LOS estimates **crab angles caused by disturbances** such as ocean currents or wind.

```math
\psi_d = \pi_h - \hat{\beta}_c - \tan^{-1}\left(\frac{y_e^p}{\Delta_h}\right)
```

```math
\dot{\hat{\beta}}_c =
\gamma_h
\frac{\Delta_h}{\sqrt{\Delta_h^2 + (y_e^p)^2}}
y_e^p
```

```math
\theta_d =
\pi_v +
\hat{\alpha}_c +
\tan^{-1}\left(\frac{z_e^p}{\Delta_v}\right)
```

```math
\dot{\hat{\alpha}}_c =
\gamma_v
\frac{\Delta_v}{\sqrt{\Delta_v^2 + (z_e^p)^2}}
z_e^p
```

where

- $\Delta_h$ is the horizontal lookahead distance
- $\Delta_v$ is the vertical lookahead distance
- $\gamma_h$ and $\gamma_v$ are the adaptive gains
- $y_e^p$ is the cross-track error
- $z_e^p$ is the vertical-track error

Adaptive LOS is generally the **most robust method** and works well for:

- curved trajectories  
- long paths  
- environments with disturbances.

---

# Proportional LOS (PLOS)

Proportional LOS is the simplest LOS guidance law.

```math
\psi_d =
\pi_h -
\tan^{-1}\left(\frac{y_e^p}{\Delta_h}\right)
```

```math
\theta_d =
\pi_v +
\tan^{-1}\left(\frac{z_e^p}{\Delta_v}\right)
```

It is best suited for

- simple waypoint following
- calm environments with little disturbance.

However, steady-state tracking error may occur if disturbances are present.

---

# Integral LOS (ILOS)

Integral LOS adds integral action to remove steady-state error.

```math
u_h =
k_{p,h}y_e^p +
k_{i,h}\int y_e^p dt
```

```math
u_v =
k_{p,v}z_e^p +
k_{i,v}\int z_e^p dt
```

```math
\psi_d =
\pi_h -
\tan^{-1}(u_h)
```

```math
\theta_d =
\pi_v +
\tan^{-1}(u_v)
```

Integral LOS performs well when there are **constant disturbances**, such as steady ocean currents or wind.

---

# Vector Field LOS (VF-LOS)

Vector Field LOS generates a **bounded approach angle** toward the path.

```math
\psi_d = \pi_h - \chi_h
```

```math
\chi_h =
\psi_{max}
\frac{2}{\pi}
\tan^{-1}(k_p y_e^p)
```

This method is best suited for

- long straight path following
- corridor tracking.

However it performs worse when the path contains **sharp turns**.

---

# Path Geometry

The path between two waypoints defines the reference angles used by the guidance law.

```math
\pi_h =
\text{atan2}(y_{i+1}^n - y_i^n, x_{i+1}^n - x_i^n)
```

```math
\pi_v =
\text{atan2}
\left(
-(z_{i+1}^n - z_i^n),
\sqrt{(x_{i+1}^n - x_i^n)^2 +
(y_{i+1}^n - y_i^n)^2}
\right)
```

where

- $P_i^n = (x_i^n, y_i^n, z_i^n)$ is the previous waypoint in the north-east-down frame
- $P_{i+1}^n = (x_{i+1}^n, y_{i+1}^n, z_{i+1}^n)$ is the next waypoint.

---

# Path Frame Errors

The along-, cross- and vertical-track errors in the path-tangential frame are found by

```math
\begin{bmatrix}
x_e^p \\
y_e^p \\
z_e^p
\end{bmatrix}
=
\mathbf{R}_{y,\pi_v}^\top
\mathbf{R}_{z,\pi_h}^\top
\left(
\begin{bmatrix}
x^n \\
y^n \\
z^n
\end{bmatrix}
-
\begin{bmatrix}
x_i^n \\
y_i^n \\
z_i^n
\end{bmatrix}
\right)
```

where

- $x_e^p$ is the along-track error  
- $y_e^p$ is the cross-track error  
- $z_e^p$ is the vertical-track error  

and

- $P^n = (x^n, y^n, z^n)$ is the current position of the vehicle.

---

# Sending a LOS Goal

A waypoint can be sent to the guidance node using the action interface:

```
ros2 action send_goal /orca/los_guidance \
vortex_msgs/action/LOSGuidance \
"{goal: {header: {frame_id: world_ned}, point: {x: 10.0, y: 5.0, z: -2.0}}}"
```

This command instructs the guidance node to start following a path toward the waypoint.

---

# Switching LOS Method

The active LOS guidance method can be changed during runtime.

```
ros2 service call /orca/set_los_mode \
vortex_msgs/srv/SetLosMode "{mode: X}"
```

Where

| X | Method |
|---|---|
| 0 | Proportional LOS |
| 1 | Integral LOS |
| 2 | Adaptive LOS |
| 3 | Vector Field LOS |

Example:

```
ros2 service call /orca/set_los_mode vortex_msgs/srv/SetLosMode "{mode: 2}"
```

This switches the guidance system to **Adaptive LOS**.

---

# ROS Topics

### Subscribed Topics

| Topic | Message |
|------|------|
| `/orca/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| `/orca/odom` | `nav_msgs/Odometry` |
| `/orca/waypoint` | `geometry_msgs/PointStamped` |

### Published Topics

| Topic | Message |
|------|------|
| `/orca/guidance/los` | `vortex_msgs/LOSGuidance` |
| `/los_debug` | `vortex_msgs/LOSGuidance` |
| `/state_debug` | `vortex_msgs/LOSGuidance` |