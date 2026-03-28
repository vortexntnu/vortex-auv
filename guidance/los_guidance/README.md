# 3D LOS Guidance Library

This package implements several **Line-of-Sight (LOS) guidance algorithms** for **3D path following**.

The guidance system computes the **desired yaw** $\psi_d$ and **desired pitch** $\theta_d$ that allow a vehicle to follow a path between waypoints.

The node receives

- vehicle pose
- waypoint information

and computes guidance references based on the selected LOS algorithm. The resulting guidance commands consist of

- desired yaw
- desired pitch
- desired surge velocity.

The vehicle surge speed is kept **constant during path following** and is defined in the configuration file using the parameter `u_desired`.

---

# Implemented LOS Methods

The library supports four LOS guidance algorithms.

| Mode | Method |
|-----|------|
| 0 | Proportional LOS |
| 1 | Integral LOS |
| 2 | Adaptive LOS |
| 3 | Vector Field LOS |

The guidance method can be preconfigured in the package configuration file before startup, and changed during runtime using a ROS service.

---


# Sending a LOS Goal

A waypoint can be sent to the guidance node using the action interface:

```
ros2 action send_goal /drone_name/los_guidance \
vortex_msgs/action/LOSGuidance \
"{goal: {header: {frame_id: world_ned}, point: {x: 0.0, y: 0.0, z: 0.0}}}"
```

This command instructs the guidance node to start following a path toward the waypoint.

---

# Switching LOS Method

The active LOS guidance method can be configured in advance through the package config file, and it can also be changed during runtime.

At runtime, the LOS guidance method can be switched with:

```
ros2 service call /drone_name/set_los_mode \
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
ros2 service call /drone_name/set_los_mode vortex_msgs/srv/SetLosMode "{mode: 2}"
```

This switches the guidance system to **Adaptive LOS**.

---
## Path Geometry

The path between two waypoints defines the **path direction angles** used by the guidance law.

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

- $\pi_h$ is the **horizontal path angle (azimuth angle)**
- $\pi_v$ is the **vertical path angle (elevation angle)**

These angles define the **direction of the current path segment** between two waypoints.

Additionally

- $P_i^n = (x_i^n, y_i^n, z_i^n)$ is the previous waypoint in the north-east-down (NED) frame
- $P_{i+1}^n = (x_{i+1}^n, y_{i+1}^n, z_{i+1}^n)$ is the next waypoint.

---

## Path Frame Errors

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

## Adaptive LOS (ALSO)

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

The terms

- $\hat{\beta}_c$ is the **estimated horizontal crab angle**
- $\hat{\alpha}_c$ is the **estimated vertical crab angle**

These angles represent the **estimated disturbance-induced deviation** between the vehicle heading and the actual direction of motion.
They allow the guidance system to **compensate for disturbances such as currents or wind**.

Adaptive LOS is generally the **most robust method** and works well for

- curved trajectories
- long paths
- environments with disturbances.

---

## Proportional LOS (PLOS)

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

where 

- $\Delta_h$ horizontal lookahead distance
- $\Delta_v$ vertical lookahead distance

The lookahead distances determine **how aggressively the vehicle corrects path errors**.

- small values → aggressive corrections
- large values → smoother but slower convergence

### Use case

PLOS works best for

- simple waypoint following
- environments with minimal disturbances.

However, it may suffer from **steady-state tracking error** when disturbances are present.

---

## Integral LOS (ILOS)

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

where

- $k_{p,h}$ horizontal proportional gain
- $k_{p,v}$ vertical proportional gain
- $k_{i,h}$ horizontal integral gain
- $k_{i,v}$ vertical integral gain

The integral term allows the controller to **eliminate steady-state cross-track errors** caused by constant disturbances.

### Use case

ILOS works well when there are **persistent disturbances**, such as

- steady ocean currents
- constant wind disturbances.

---

## Vector Field LOS (VF-LOS)

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

where

- $\psi_{max}$ maximum allowed approach angle
- $k_p$ proportional gain controlling path convergence

The bounded approach angle prevents excessively aggressive heading changes.

### Use case

VF-LOS works best for

- long straight path following
- corridor tracking
- inspection missions along pipelines or cables.

However it performs worse when the path contains **sharp turns or rapidly changing path segments**.

---

## Testing with `guidance_test.launch.py`

To test the entire los system from just one place a simple, low-effort launch script has been implemented. The file is  named `los_guidance/launch/guidance_test.launch.py` It launches the necessary nodes for simulation and autopilot and lets you choose a test scenario through the `test_scenario` argument.The sceraios that have been implemented are:

| Scenario | Use |
|----------|------|
| Circle | ends the drone in a circular path |
| Square |sends the drone through a square waypoint pattern|
| opposite_point | sends the drone toward a waypoint and then toward a waypoint in the opposite direction (180 degrees) |

Example of use:

```
ros2 launch los_guidance guidance_test.launch.py test_scenario:=circle
```

---


## ROS Interfaces

| Interface | Name | Type | Message-Type |
|----------|------|------|---------|
| Action Server | `/drone_name/los_guidance` | Goal input | `vortex_msgs/action/LOSGuidance` |
| Subscriber | `/drone_name/pose` | Vehicle pose | `geometry_msgs/PoseWithCovarianceStamped` |
| Subscriber | `/drone_name/odom` | Vehicle velocity | `nav_msgs/Odometry` |
| Publisher | `/drone_name/guidance/los` | Guidance reference (yaw, pitch, surge) | `vortex_msgs/LOSGuidance` |
| Publisher | `/los_debug` | LOS debug output | `vortex_msgs/LOSGuidance` |
| Publisher | `/state_debug` | Vehicle state debug | `vortex_msgs/LOSGuidance` |

---

## Guidance Node Architecture
The LOS guidance node computes reference commands for the vehicle based on
the current vehicle state and the active waypoint.

The process works as follows

1. The node receives the **vehicle pose**.
2. The current **path segment** is defined between two waypoints.
3. The **path geometry** is computed to determine the path direction.
4. The **cross-track and vertical-track errors** are calculated in the path frame.
5. The selected **LOS guidance algorithm** computes desired yaw and pitch.
6. The node publishes the resulting **guidance reference**.

### Data Flow

```
Vehicle Pose + Waypoint
           │
           ▼
     Path Geometry
   (π_h , π_v angles)
           │
           ▼
   Path Frame Errors
 (x_e^p , y_e^p , z_e^p)
           │
           ▼
      LOS Algorithm
 (PLOS / ILOS / ALSO / VF-LOS)
           │
           ▼
     Guidance Output
 (ψ_d , θ_d , u_desired)
```

The resulting guidance command is published as a `vortex_msgs/LOSGuidance`
message containing

- desired yaw
- desired pitch
- desired surge velocity.

---
