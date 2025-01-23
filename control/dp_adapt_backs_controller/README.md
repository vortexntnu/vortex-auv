## DP Adaptive Backstepping Controller

This package implements a dynamic positioning (DP) Adaptive backstepping controller for the orca AUV. It estimates the linear and nonlinear damping using adaptive parameters, and compensates for uncertainties and disturbances in real-time in the same manner as a state estimator. The proof for this is done using Lyapunov functions and stability requirements to ensure convergence and stability.

### Overview
- Uses the backstepping control method for position and orientation control
- Includes adaptive terms to account for unmodeled dynamics and uncertainties.

### Model for AUV

```math
\dot{\eta} = J(\eta)\nu, \newline
M \dot{\nu} + C(\nu)\,\nu - F(\nu, \Theta^*) = \tau + d^*
```

- $\nu$: Body-fixed velocity vector
- $\eta$: Inertial position and orientation vector
- M: Constant mass-inertia matrix
- C($\eta$): Coriolis and centripetal terms
- J($\eta$): Transformation from body to inertial coordinates
- F($\nu$, $\Theta^*$) =  Y($\nu$) $\Theta^*$: The damping assumed damping function (linear and nonlinear), where Y(*) describes the behaviour
- $d^*$: Disturbance and uncertainties

### File overview
1. **dp_adapt_backs_controller.cpp/hpp**
     - The controller implementation
     - Implements the main control input, sets gains, references, and mass parameters.

2. **dp_adapt_backs_controller_utils.cpp/hpp**
     - Provides utility functions: skew-symmetric matrix generation, quaternion-to-Euler, Jacobian and some functions needed for the adaptive functions.

3. **dp_adapt_backs_controller_ros.cpp/hpp**
     - ROS node wrapper for the controller, subscribing to pose, twist, killswitch, and reference topics.
     - Publishes thrust commands.

4. **dp_adapt_backs_controller_node.cpp**
     - Entry point for the ROS executable.

5. **adapt_params.yaml**
     - Tunable controller parameters (K1, K2, adapt_gain, d_gain)
     - Contains the mass matrix (6×6) and other physical parameters:
         M =
         [ 30.0      0.0    0.0   ... ]
         [   0.0   30.0    ...   ... ]
         [   0.0    ...   30.0   ... ]
         [   ...    ...    ...    ... ]
         (partially shown for brevity)
         This is also added into the orca.yaml file

6. **CMakeLists.txt**
     - Build configuration, ROS 2 dependencies, executable generation, and installation setup.

### Tuning Parameters
- **K1** and **K2**: Gains for the backstepping control laws.
- **adapt_gain**: Adaptive gain for the linear and nonlinear damping.
- **d_gain**: Adaptive gain for the disturbances and uncertainties.
- **M** and **I_b**: Mass inertia matrix and rotational inertia.
- **m**: Vehicle mass.

## Backstepping controller

Using the Lyapunov proof we get the following functions for the backstepping controller:

##### Backstepping variables

$z_1 = \eta - \eta_d$
$z_2 = \nu - \alpha$

where the $\alpha$ is defined as the function that stabilizes the $z_1$ variable in the Lyapunov function system.

##### Adaptive parameters

$\tilde{\Theta} = \hat{\Theta} - \Theta^*$
$\tilde{d} = \hat{d} - d^*$

where:
- \(\Theta^*\) and \(d^*\) are the actual parameters,
- \(\hat{\Theta}\) and \(\hat{d}\) are the estimated parameters,
- \(\tilde{\Theta}\) and \(\tilde{d}\) are the estimation errors.

#### Proof of control law

We define the Lyapunov function candidate (CLF) as:

\[
V_1 = \frac{1}{2} z_1^\top z_1
\]

By fulfilling the criteriums of RUB, positive definite and V_1(0) = 0, we need to do a derivation and ensure negative definite

Taking the derivative of the Lyapunov function candidate \( V_1 \):

\[
\dot{V}_1 = \frac{d}{dt} \left( \frac{1}{2} z_1^\top z_1 \right)
\]

Using the chain rule, we get:

\[
\dot{V}_1 = z_1^\top \dot{z}_1
\]

Substitute \(\dot{z}_1\) with the dynamics of \(z_1\):

\[
\dot{z}_1 = \dot{\eta} - \dot{\eta}_d
\]

and then inserting in the relation; \(\eta = z_1 + \eta_d\)

\[
\dot{z}_1 = J(\eta)\nu - \dot{\eta}_d
\]


Thus, the derivative of the Lyapunov function candidate is: $\dot{\eta} = J(\eta)\nu$ and $\nu = z_2 + \alpha$
We also assume that the $\dot{\eta_d} = 0$, since we only get a desired position and orientation ($\eta$)
\[
\dot{V}_1 = z_1^\top J(\eta)(z_2 + \alpha)
\]

We choose an $\alpha$ value to make the $z_1$ be negative semi definite.

\[
\boxed{
\alpha = -J(\eta)^{-1}(K_1 z_1)
}
\]

\[
\dot{V}_1 = z_1^\top J(\eta)z_2 - z_1^\top K_1 z_1 < 0
\]

We will now define the rest of the Lyapunov function candidates, including the adaptive parameter.

\[
V_2 = \frac{1}{2} z_2^\top M z_2
\]
For the second backstepping variable

\[
V_{\theta} = \frac{1}{2} \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \tilde{\Theta}
\]

\[
V_d = \frac{1}{2} \tilde{d}^\top \Gamma^{-1}_{d} \tilde{d}
\]

Where the $\Gamma$ matrix is a diagonal gain matrix for tuning the adaptive rate of the parameters, and by definition positive definite.
The reasoning behind inserting in the adaptive parameters is that we want to ensure that the error, marked by the ~, is what we want to converge towards zero. To achieve this we need to ensure that it actually does this by accounting for it in the Lyapunov function and controller.

Then we combine them like this:


\[
V = V_1 + V_2 + V_{\theta} + V_d
\]

and now we will analyse the derivative of this CLF, and ensure convergence for the whole function.

\[
\dot{V} = \dot{V}_1 + z_2^\top M \dot{z}_2 + \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\tilde{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\tilde{d}}
\]

Before we write this out we need to make some assumption to make this more readable and easier to understand.

1. For $\dot{\tilde{\Theta}} = \dot{\hat{\Theta}} - \dot{\Theta}^*$ we assume that the actual value has no changes, assuming its static, and therefore the derivative $\dot{\Theta}^* = 0$.
2. The same condition holds for the $\dot{\tilde{d}}$

\[
\dot{V} = \dot{V}_1 + z_2^\top M (\dot{\nu} - \dot{\alpha}) + \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}}
\]


\[
= \dot{V}_1 - z_2^\top M \dot{\alpha} + z_2^\top\tau - z_2^\top C(\nu)\,\nu + z_2^\top F(\nu, \Theta^*) + z_2^\top d + \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}}
\]


\[
= - z_1^\top K_1z_1 + z_2^\top J(\eta)z_1 - z_2^\top M \dot{\alpha} + z_2^\top \tau - z_2^\top C(\nu)\,\nu + z_2^\top Y(\nu) \Theta^* + z_2^\top d  + \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}}
\]

We can say this:
Since we only know the estimate of the adaptive parameters, we can write the controller in two parts:

\[
\tau = \tau_{controller} - F(\nu, \hat{\Theta}) - d = \tau_{controller} -Y(\nu) \hat{\Theta} - \hat{d}
\]
Important to notice is that we here introduce the estimate (^) for the variables, not the actual value (*).
We insert this into the system and get:

\[
= - z_1^\top K_1 z_1 + z_2^\top J(\eta)z_1 - z_2^\top M \dot{\alpha} + z_2^\top \tau_{controller} - z_2^\top C(\nu)\,\nu - z_2^\top Y(\nu) (\hat{\Theta} - \Theta^* ) - z_2^\top (\hat{d} - d^*)  + \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}}
\]

We look at the adaptive parameters a little more now and try to simplify them as much as possible

\[
\hat{\Theta} - \Theta^* = \hat{\Theta} - (\hat{\Theta} - \tilde{\Theta}) = \tilde{\Theta} \newline
\hat{d} - d^* = \hat{d} - (\hat{d} - \tilde{d}) = \tilde{d}
\]


Now we have:
\[
= - z_1^\top K_1 z_1 + z_2^\top J(\eta)z_1 - z_2^\top M \dot{\alpha} + z_2^\top \tau_{controller} - z_2^\top C(\nu)\,\nu + (-z_2^\top Y(\nu) \tilde{\Theta}+ \tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}}) + (-z_2^\top \tilde{d} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}})
\]

\[
-z_2^\top Y(\nu) \tilde{\Theta}+\tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}} = - \tilde{\Theta}^\top Y(\nu)^\top z_2 +\tilde{\Theta}^\top \Gamma^{-1}_{\theta} \dot{\hat{\Theta}}\newline
-z_2^\top \tilde{d} + \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}} = - \tilde{d}^\top z_2+ \tilde{d}^\top \Gamma^{-1}_{d} \dot{\hat{d}}
\]

Now we choose the $\dot{\hat{\Theta}}$ and $\dot{\hat{d}}$ to zero this out
\[
\boxed{
\dot{\hat{\Theta}} =  \Gamma_{\theta} Y(\nu)^\top z_2
}
\]

\[
\boxed{
\dot{\hat{d}} = \Gamma_{d} z_2
}
\]
Now that we have defined this we can insert and remove this from the equation, which should leave us with the normal system. An observation made during the construction of the controller was that the adaptive part and backstepping part is decoupled. Maybe this is dependent on the method used for the adaptive part, which is more of a MRAC type method. Sadly i dont have enough information about adaptive controllers to comment on this in detail.

\[
= - z_1^\top K_1 z_1 + z_2^\top J(\eta)z_1 - z_2^\top M \dot{\alpha} + z_2^\top \tau_{controller} - z_2^\top C(\nu)\,\nu
\]
\[
\tau_{controller} = -K_2 z_2 + M \dot{\alpha} + C(\nu)\nu - J(\eta)z_1
\]

Combined this gives us the control law:

\[
\boxed{
\tau = -K_2 z_2 + M \dot{\alpha} + C(\nu)\nu - J(\eta)z_1 - F(\nu, \hat{\Theta}) - \hat{d}
}
\]

### Controller gains


The gains \(K_1\) and \(K_2\) are crucial for ensuring the stability and performance of the controller.

**\(K_1\)**: This gain matrix is associated with the outer loop stability, which primarily deals with the position and orientation errors (\(z_1\)). Proper tuning of \(K_1\) ensures that the position and orientation errors converge to zero, thereby stabilizing the outer loop.


\[
\begin{bmatrix}
k_{1,1} & 0 & 0 & 0 & 0 & 0 \\
0 & k_{1,2} & 0 & 0 & 0 & 0 \\
0 & 0 & k_{1,3} & 0 & 0 & 0 \\
0 & 0 & 0 & k_{1,4} & 0 & 0 \\
0 & 0 & 0 & 0 & k_{1,5} & 0 \\
0 & 0 & 0 & 0 & 0 & k_{1,6}
\end{bmatrix}
\]

**\(K_2\)**: This gain matrix is related to the inner loop stability, which handles the velocity errors (\(z_2\)). Tuning \(K_2\) appropriately ensures that the velocity errors are minimized, stabilizing the inner loop.

\[
K_2 =
\begin{bmatrix}
k_{2,1} & 0 & 0 & 0 & 0 & 0 \\
0 & k_{2,2} & 0 & 0 & 0 & 0 \\
0 & 0 & k_{2,3} & 0 & 0 & 0 \\
0 & 0 & 0 & k_{2,4} & 0 & 0 \\
0 & 0 & 0 & 0 & k_{2,5} & 0 \\
0 & 0 & 0 & 0 & 0 & k_{2,6}
\end{bmatrix}
\]

Together, \(K_1\) and \(K_2\) work to ensure that both the position/orientation and velocity errors are driven to zero, guaranteeing the overall stability and performance of the adaptive backstepping controller.


### Adaptive parameters and functions

Now that we have the proof for the control law, we need to look at the adaptive functions and params.

Since we wanted a damping (linear and nonlinear) $Y(\nu)$ was chosen to be a 6x12 matrix with one linear and one nonlinear damping element:

\[
Y(\nu) =
\begin{bmatrix}
\nu[0] & \nu[0] |\nu[0]| & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \nu[1] & \nu[1] |\nu[1]| & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \nu[2] & \nu[2] |\nu[2]| & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \nu[3] & \nu[3] |\nu[3]| & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \nu[4] & \nu[4] |\nu[4]| & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & \nu[5] & \nu[5] |\nu[5]| \\
\end{bmatrix}
\]

while the $\hat\Theta$ will be a 12x1 vector:

\[
\hat{\Theta} =
\begin{bmatrix}
\alpha_1 & \beta_1 & \alpha_2 & \beta_2 & \alpha_3 & \beta_3 & \alpha_4 & \beta_4 & \alpha_5 & \beta_5 & \alpha_6 & \beta_6
\end{bmatrix}^\top
\]

The $\Gamma_{\Theta}$ will therefore be a 12x12 diagonal vector with the gains for the linear and nonliear damping.

The disturbance d will have a 6x1 vector for the distubance and uncertainty estiamtes
\[
\hat{d} =
\begin{bmatrix}
d_1 & d_2 & d_3 & d_4 & d_5 & d_6
\end{bmatrix}^\top
\]

#### Important information

The implementation of the controller requires $\dot\alpha$ which is not that simple to calculate, given that this require the derivative of the $J(\eta)$ matrix.

\[
\dot{\alpha}
\;=\;
-\;J(\eta)^{-1}\,\dot{J}(\eta)\,J(\eta)^{-1}\bigl[- k_1\,(\eta - \eta_d)\bigr]
\;+\;
-J(\eta)^{-1} k_1\,\dot{\eta}
\]

This was manually derived, and $J(\eta)$ was also derived maunally but here we can use a trick

\[
J(\eta) =
\begin{bmatrix}
R & 0_{3x3} \\
0_{3x3} & T
\end{bmatrix}
\]

where:
- \(R\) is the rotation matrix representing the orientation of the AUV.
- \(T\) is the transformation matrix for the translational components.
- \(0_{3x3}\) is a 3x3 zero matrix.


The derivative of \(J(\eta)\) can be expressed in terms of the derivatives of \(R\) and \(T\):

\[
\dot{J}(\eta) =
\begin{bmatrix}
\dot{R} & 0_{3x3} \\
0_{3x3} & \dot{T}
\end{bmatrix}
\]

The trick is that $\dot{R}$ can be written as:

\[
\dot{R} = R \cdot S \cdot r
\]

where:
- \( R \) is the rotation matrix calculated from \(\eta\).
- \( S \) is the skew-symmetric matrix of the vector \((0, 0, 1)\).
- \( r \) is the z-component of the angular speed from \(\nu\).

For the T the expression needs to be manually calculated and this is not nice to look at:

\[
\dot{T}(\eta, \nu) =
\begin{bmatrix}
0 & \cos(\phi) \tan(\theta) \nu_x + \sin(\phi) \sec^2(\theta) \nu_y & -\sin(\phi) \tan(\theta) \nu_x + \cos(\phi) \sec^2(\theta) \nu_y \\
0 & -\sin(\phi) \nu_x & -\cos(\phi) \nu_x \\
0 & \cos(\phi) / \cos(\theta) \nu_x + \sin(\phi) \sin(\theta) / \cos^2(\theta) \nu_y & -\sin(\phi) / \cos(\theta) \nu_x + \cos(\phi) \sin(\theta) / \cos^2(\theta) \nu_y
\end{bmatrix}
\]

This can be calculated by the chain rule and parcial differentiate on $\phi, \theta, \psi$:

\[
\dot{T}(\eta, \nu) = \frac{\partial T}{\partial \phi} \dot{\phi} + \frac{\partial T}{\partial \theta} \dot{\theta} + \frac{\partial T}{\partial \psi} \dot{\psi}
\]

Here we could potentially use methods for estimating the derivative, but one of the benefits of this calculation is that we get good and accurate values for the rotation and translation derivatives. Instead of introducing more uncertainties as we try to remove this in the system with the adaptive part.
## Launch

To run the controller, use the ROS 2 launch file:
    `ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py`

It's important to `colcon build` and `source install/setup.bash` before launching, or else it will not be launchable.

This will load the parameters from the provided YAML, start the node, and begin control operation.
