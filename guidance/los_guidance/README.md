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

## Adaptive LOS (ALOS)

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

### Parameters

- $\Delta_h$ — horizontal lookahead distance  
- $\Delta_v$ — vertical lookahead distance  

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

### Parameters

- $k_{p,h}$ — horizontal proportional gain  
- $k_{p,v}$ — vertical proportional gain  
- $k_{i,h}$ — horizontal integral gain  
- $k_{i,v}$ — vertical integral gain  

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

### Parameters

- $\psi_{max}$ — maximum allowed approach angle  
- $k_p$ — proportional gain controlling path convergence

The bounded approach angle prevents excessively aggressive heading changes.

### Use case

VF-LOS works best for

- long straight path following  
- corridor tracking  
- inspection missions along pipelines or cables.

However it performs worse when the path contains **sharp turns or rapidly changing path segments**.