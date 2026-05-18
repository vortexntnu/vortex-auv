## DP Adaptive Backstepping Controller — Quaternion Formulation

This package implements a dynamic positioning (DP) adaptive backstepping controller for the Nautilus AUV. It uses a **unit-quaternion error-state formulation** for the attitude kinematics, which eliminates the gimbal-lock singularity that occurs at ±90° pitch in the Euler-angle version. The adaptive terms estimate the linear and quadratic damping online and compensate for unmodelled disturbances, in the same spirit as a model reference adaptive controller. Stability and convergence are proven via a composite Lyapunov function.

### Overview
- Uses the backstepping control method for position and orientation control
- Replaces the Euler-angle Jacobian $J(\eta)$ with the **quaternion error-state Jacobian** $J_e(\eta)$ (called `L` in the code), which is square, smooth, and invertible for all attitude errors smaller than 180°
- Includes adaptive terms to estimate linear and nonlinear (quadratic) damping and external disturbances

### Model for AUV

**Kinematics (error-state form):**

```math
\dot{z}_1 = J_e(\eta)\,\nu
```

where the error state $z_1 \in \mathbb{R}^6$ is defined below, and the error-state Jacobian is:

```math
J_e(\eta) =
\begin{bmatrix}
R(q) & 0_{3\times 3} \\
0_{3\times 3} & T_e(q_e)
\end{bmatrix}, \quad
T_e(q_e) = \eta_e I_3 + S(\varepsilon_e)
```

Here $R(q) \in SO(3)$ is the rotation matrix from NED to body, $q_e = q_d^* \otimes q$ is the error quaternion (scalar part $\eta_e$, vector part $\varepsilon_e$), and $S(\cdot)$ is the skew-symmetric (cross-product) matrix.

**Dynamics (Newton–Euler, body frame):**

```math
M\dot{\nu} + C(\nu)\,\nu - F(\nu,\Theta^\star) = \tau + d^\star
```

- $\nu \in \mathbb{R}^6$: body-fixed velocity (linear and angular)
- $M$: constant, symmetric, positive-definite mass-inertia matrix
- $C(\nu)$: Coriolis and centripetal matrix
- $F(\nu, \Theta^\star) = Y(\nu)\Theta^\star$: damping in regressor form (linear and quadratic per DOF)
- $\tau$: control wrench
- $d^\star$: lumped disturbances and unmodelled dynamics

**Notation note:** The symbol $\eta_e$ denotes the scalar part of the error quaternion (not the pose vector $\eta$); this follows Fossen's standard overloading.

### File overview

1. **dp_adapt_backs_controller.cpp/hpp**
   - Core controller implementation: computes `L`, `L_inv`, `L_dot`, the error state $z_1$, $z_2$, $\alpha$, $\dot\alpha$, and the full control wrench $\tau$.
   - Integrates the adaptive parameters online.

2. **dp_adapt_backs_controller_utils.cpp/hpp**
   - Utility functions: `calculate_L_inv`, `calculate_R_dot`, `calculate_Q_dot`, `calculate_L_dot`, `calculate_coriolis`, `calculate_Y_v`.

3. **dp_adapt_backs_controller_ros.cpp/hpp**
   - ROS 2 node wrapper: subscribes to odometry, killswitch, and reference topics; publishes thrust commands.

4. **adapt_params_nautilus.yaml / adapt_params_nautilus_sim.yaml**
   - Tunable controller parameters (`K1`, `K2`, `adapt_gain`, `d_gain`, `r_b_bg`, `time_step`).

5. **CMakeLists.txt**
   - Build configuration, ROS 2 dependencies, executable generation, and installation setup.

### Tuning Parameters
- **K1**: Outer loop gain matrix (position and orientation errors $z_1$).
- **K2**: Inner loop gain matrix (velocity error $z_2$).
- **adapt\_gain**: Diagonal adaptation rate for the 12 damping parameters ($\Gamma_\theta$).
- **d\_gain**: Diagonal adaptation rate for the 6 disturbance estimates ($\Gamma_d$).
- **r\_b\_bg**: Vector from body origin to centre of gravity (used in the Coriolis matrix).

## Backstepping Controller

### Error state and backstepping variables

The orientation error quaternion is formed by left-multiplication with the desired quaternion conjugate:

```math
q_e = q_d^* \otimes q = \begin{bmatrix} \eta_e \\ \varepsilon_e \end{bmatrix}
```

The tracking error in $\mathbb{R}^6$ is then:

```math
z_1 = \begin{bmatrix} p - p_d \\ 2\varepsilon_e \end{bmatrix}
```

The factor of 2 on the vector part of the error quaternion ensures $\dot{z}_{1,\text{ori}} = T_e(q_e)\,\omega$ and makes the LFC derivative tractable. The velocity error is:

```math
z_2 = \nu - \alpha
```

where $\alpha$ is the virtual control law defined below.

### Adaptive parameters

```math
\tilde{\Theta} = \hat{\Theta} - \Theta^\star, \qquad \tilde{d} = \hat{d} - d^\star
```

where:
- $\Theta^\star$ and $d^\star$ are the (unknown) true parameters
- $\hat{\Theta}$ and $\hat{d}$ are online estimates
- $\tilde{\Theta}$ and $\tilde{d}$ are the estimation errors

### Proof of control law

#### Step 1 — Outer loop (position and attitude)

Define the LFC:

```math
V_1 = \frac{1}{2} z_1^\top z_1
```

which is positive definite, radially unbounded, and satisfies $V_1(0) = 0$. For a constant setpoint ($\dot{\eta}_d = 0$):

```math
\dot{V}_1 = z_1^\top \dot{z}_1 = z_1^\top J_e(\eta)\,\nu
```

Treating $\nu$ as a virtual input (Khalil §14.3) and splitting $\nu = \alpha + z_2$:

```math
\dot{V}_1 = z_1^\top J_e\,\alpha + z_1^\top J_e\,z_2
```

Choose the virtual control law:

```math
\boxed{
\alpha = -J_e(\eta)^{-1} K_1\, z_1, \quad K_1 = K_1^\top > 0
}
```

Then $z_1^\top J_e\,\alpha = -z_1^\top K_1 z_1 < 0$ and:

```math
\dot{V}_1 = -z_1^\top K_1 z_1 + z_1^\top J_e\, z_2
```

The cross term $z_1^\top J_e z_2$ will be cancelled in Step 2.

#### Step 2 — Inner loop (velocity)

Augment the LFC with the inertia-weighted velocity term (Fossen 2021, §12.1):

```math
V_2 = \frac{1}{2} z_2^\top M\, z_2, \quad M = M^\top > 0,\; \dot{M} = 0
```

Differentiating and substituting the dynamics:

```math
\dot{V}_2 = z_2^\top M\,(\dot{\nu} - \dot{\alpha}) = z_2^\top\bigl(\tau - C(\nu)\nu + Y(\nu)\Theta^\star + d^\star - M\dot{\alpha}\bigr)
```

**Cross-term cancellation.** The scalar $z_1^\top J_e z_2 = z_2^\top J_e^\top z_1$. If the control law contains $-J_e^\top z_1$, then in $\dot{V}_1 + \dot{V}_2$:

```math
z_1^\top J_e\, z_2 + z_2^\top(-J_e^\top z_1) = z_2^\top J_e^\top z_1 - z_2^\top J_e^\top z_1 = 0
```

The cross terms cancel exactly, independent of the structure of $J_e$.

#### Adaptive extension

Since $\Theta^\star$ and $d^\star$ are unknown, form the composite LFC:

```math
V = V_1 + V_2 + \frac{1}{2}\tilde{\Theta}^\top \Gamma^{-1}_{\theta}\,\tilde{\Theta} + \frac{1}{2}\tilde{d}^\top \Gamma^{-1}_{d}\,\tilde{d}
```

Assuming $\dot{\Theta}^\star = 0$ and $\dot{d}^\star = 0$ (static true parameters):

```math
\dot{V} = \dot{V}_1 + z_2^\top M(\dot{\nu} - \dot{\alpha}) + \tilde{\Theta}^\top \Gamma^{-1}_{\theta}\,\dot{\hat{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d}\,\dot{\hat{d}}
```

Substituting the control law:

```math
\tau = -J_e^\top z_1 - K_2\, z_2 + M\dot{\alpha} + C(\nu)\nu - Y(\nu)\hat{\Theta} - \hat{d}
```

and collecting terms (cross terms cancel, $C(\nu)\nu$ cancels, $M\dot\alpha$ cancels):

```math
\dot{V} = -z_1^\top K_1 z_1 - z_2^\top K_2 z_2
+ \tilde{\Theta}^\top\!\left(\Gamma_\theta^{-1}\dot{\hat{\Theta}} - Y(\nu)^\top z_2\right)
+ \tilde{d}^\top\!\left(\Gamma_d^{-1}\dot{\hat{d}} - z_2\right)
```

From this we can separate the adaptive terms:

```math
\tilde{\Theta}^\top\!\left(\Gamma_\theta^{-1}\dot{\hat{\Theta}} - Y(\nu)^\top z_2\right) = \tilde{\Theta}^\top \Gamma_\theta^{-1}\!\left(\dot{\hat{\Theta}} - \Gamma_\theta Y(\nu)^\top z_2\right)
```

```math
\tilde{d}^\top\!\left(\Gamma_d^{-1}\dot{\hat{d}} - z_2\right) = \tilde{d}^\top \Gamma_d^{-1}\!\left(\dot{\hat{d}} - \Gamma_d z_2\right)
```

Choosing the update laws to zero these brackets:

```math
\boxed{
\dot{\hat{\Theta}} = \Gamma_{\theta}\, Y(\nu)^\top z_2
}
```

```math
\boxed{
\dot{\hat{d}} = \Gamma_{d}\, z_2
}
```

This gives the final Lyapunov derivative:

```math
\dot{V} = -z_1^\top K_1 z_1 - z_2^\top K_2 z_2 < 0, \quad \forall\,(z_1,z_2) \neq 0
```

Global asymptotic stability of $z_1 = 0$, $z_2 = 0$ follows from LaSalle's invariance principle (Khalil §4.2), with parameter estimates remaining bounded by the adaptive law structure.

### Full control law

```math
\boxed{
\tau = -J_e^\top z_1 - K_2\, z_2 + M\dot{\alpha} + C(\nu)\nu - Y(\nu)\hat{\Theta} - \hat{d}
}
```

### Controller gains

**$K_1$** is the outer loop gain (position and orientation errors):

```math
K_1 =
\begin{bmatrix}
k_{1,1} & & & & & \\
& k_{1,2} & & & & \\
& & k_{1,3} & & & \\
& & & k_{1,4} & & \\
& & & & k_{1,5} & \\
& & & & & k_{1,6}
\end{bmatrix}
```

**$K_2$** is the inner loop gain (velocity errors):

```math
K_2 =
\begin{bmatrix}
k_{2,1} & & & & & \\
& k_{2,2} & & & & \\
& & k_{2,3} & & & \\
& & & k_{2,4} & & \\
& & & & k_{2,5} & \\
& & & & & k_{2,6}
\end{bmatrix}
```

### Adaptive parameters and functions

The damping regressor $Y(\nu) \in \mathbb{R}^{6 \times 12}$ captures one linear and one quadratic term per DOF:

```math
Y(\nu) =
\begin{bmatrix}
\nu_1 & \nu_1|\nu_1| & 0 & 0 & \cdots & 0 & 0 \\
0 & 0 & \nu_2 & \nu_2|\nu_2| & \cdots & 0 & 0 \\
\vdots & & & & \ddots & & \vdots \\
0 & 0 & 0 & 0 & \cdots & \nu_6 & \nu_6|\nu_6|
\end{bmatrix}
```

The parameter vector $\hat{\Theta} \in \mathbb{R}^{12}$ is:

```math
\hat{\Theta} =
\begin{bmatrix}
\alpha_1 & \beta_1 & \alpha_2 & \beta_2 & \alpha_3 & \beta_3 & \alpha_4 & \beta_4 & \alpha_5 & \beta_5 & \alpha_6 & \beta_6
\end{bmatrix}^\top
```

where $\alpha_i$ and $\beta_i$ are the estimated linear and quadratic damping coefficients for DOF $i$. The adaptation gain $\Gamma_\theta$ is a $12 \times 12$ positive-definite diagonal matrix.

The disturbance estimate $\hat{d} \in \mathbb{R}^6$ has one component per DOF, adapted with the $6 \times 6$ diagonal gain $\Gamma_d$.

### Important implementation detail: computing $\dot{\alpha}$

The control law requires $\dot{\alpha}$, the time derivative of the virtual control. Since $\alpha = -J_e(\eta)^{-1}K_1 z_1$, applying the matrix identity $\tfrac{d}{dt}(A^{-1}) = -A^{-1}\dot{A}A^{-1}$:

```math
\dot{\alpha} = J_e^{-1}\dot{J}_e J_e^{-1} K_1 z_1 - J_e^{-1} K_1 J_e\,\nu
```

The block structure of $J_e$ gives:

```math
\dot{J}_e =
\begin{bmatrix}
\dot{R} & 0_{3\times 3} \\
0_{3\times 3} & \dot{T}_e
\end{bmatrix}
```

**$\dot{R}$** uses the standard identity:

```math
\dot{R} = R\,S(\omega)
```

**$\dot{T}_e$** follows from differentiating $T_e = \eta_e I + S(\varepsilon_e)$ using the quaternion kinematic equations $\dot{\eta}_e = -\tfrac{1}{2}\varepsilon_e^\top\omega$ and $\dot{\varepsilon}_e = \tfrac{1}{2}T_e\omega$:

```math
\dot{T}_e = \tfrac{1}{2}\bigl(S(T_e\,\omega) - (\varepsilon_e^\top\omega)\,I_3\bigr)
```

This compact closed form — compared to the lengthy trigonometric expression required for $\dot{T}$ in the Euler-angle version — is a key practical advantage of the quaternion parameterisation.

## Launch

To run the controller, use the ROS 2 launch file:
```bash
ros2 launch dp_adapt_backs_controller_quat dp_adapt_backs_controller_quat.launch.py
```

Remember to `colcon build` and `source install/setup.bash` first.

Two configuration files are provided:
- `adapt_params_nautilus.yaml` — tuned for the physical Nautilus AUV
- `adapt_params_nautilus_sim.yaml` — tuned for simulation
