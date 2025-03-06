## ALSO Guidance Law for 3D Path Following

The guidance law gives calculates the desired heading angle $\psi_d$ and desired pitch angle $\theta_d$. The crab angles $\beta_c$ and $\alpha_c$ are estimated adaptively. The guidance law looks like

```math
\psi_d = \pi_h - \hat{\beta}_c - \tan^{-1}\left(\frac{y_e^p}{\Delta_h}\right)

```

```math
\dot{\hat{\beta}}_c = \gamma_h \frac{\Delta_h}{\sqrt{\Delta_h^2 + (y_e^p)^2}} y_e^p
```

```math
\theta_d = \pi_v + \hat{\alpha}_c + \tan^{-1}\left(\frac{z_e^p}{\Delta_v}\right)
```

```math
\dot{\hat{\alpha}}_c = \gamma_v \frac{\Delta_v}{\sqrt{\Delta_v^2 + (z_e^p)^2}} z_e^p
```

where

- $\Delta_h$ is the horizontal lookahead distance
- $\Delta_v$ is the vertical lookahead distance
- $\gamma_h$ and $\gamma_v$ are the adaptive gains
- $y_e^p$ is the cross-track error
- $z_e^p$ is the vertical-track error

The azimuth angle $\pi_v$ and the elevation angle $\pi_h$ can be found by

```math
\pi_h = \text{atan2}(y_{i+1}^n - y_i^n, x_{i+1}^n, - x_i^n)
```
```math
\pi_v = \text{atan2}(-(z_{i+1}^n - z_i^n), \sqrt{(x_{i+1}^n - x_i^n)^2 + (y_{i+1}^n - y_i^n)^2})
```

where $P_i^n = (x_i^n, y_i^n, z_i^n)$ is the previous waypoint in the north-east-down frame and $P_{i+1}^n = (x_{i+1}^n, y_{i+1}^n, z_{i+1}^n)$ is the next waypoint in north-east-down frame.

The along-, cross- and vertical-track errors in the path-tangential frame are found by

```math
\begin{bmatrix}
x_e^p \\ y_e^p \\ z_e^p
\end{bmatrix} = \mathbf{R}_{y, \pi_v}^\top \mathbf{R}_{z, \pi_h}^\top \left( \begin{bmatrix}
x^n \\ y^n \\ z^n
\end{bmatrix} - \begin{bmatrix}
x_i^n \\ y_i^n \\ z_i^n
\end{bmatrix}
\right)
```
where $P^n = (x^n, y^n, z^n)$ is the current position of the drone.
