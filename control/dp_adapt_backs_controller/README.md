## DP Adaptive Backstepping Controller

This package implements a dynamic positioning (DP) adaptive backstepping controller for the Orca AUV. It estimates the linear and nonlinear damping using adaptive parameters and compensates for uncertainties and disturbances in real-time, similar to a state estimator. The control law is derived and proven using Lyapunov functions and stability criteria to ensure convergence and stability.

---

### Overview
- Uses the backstepping control method for position and orientation control.
- Includes adaptive terms to account for unmodeled dynamics and uncertainties.

---

### Model for AUV

\[
\dot{\eta} = J(\eta)\nu, \newline
M \dot{\nu} + C(\nu)\,\nu - F(\nu, \Theta^*) = \tau + d^*
\]

- \(\nu\): Body-fixed velocity vector.
- \(\eta\): Inertial position and orientation vector.
- \(M\): Constant mass-inertia matrix.
- \(C(\eta)\): Coriolis and centripetal terms.
- \(J(\eta)\): Transformation from body to inertial coordinates.
- \(F(\nu, \Theta^*) = Y(\nu) \Theta^*\): Assumed damping function (linear and nonlinear), where \(Y(\nu)\) describes the behavior.
- \(d^*\): Disturbance and uncertainties.

---

### File Overview
1. **dp_adapt_backs_controller.cpp/hpp**
    - Implements the main control logic.
    - Sets gains, references, and mass parameters.

2. **dp_adapt_backs_controller_utils.cpp/hpp**
    - Utility functions: skew-symmetric matrix generation, quaternion-to-Euler conversion, Jacobians, and adaptive function utilities.

3. **dp_adapt_backs_controller_ros.cpp/hpp**
    - ROS node wrapper for the controller, subscribing to pose, twist, killswitch, and reference topics.
    - Publishes thrust commands.

4. **dp_adapt_backs_controller_node.cpp**
    - Entry point for the ROS executable.

5. **adapt_params.yaml**
    - Tunable controller parameters (\(K_1\), \(K_2\), \(\text{adapt\_gain}\), \(\text{d\_gain}\)).
    - Contains the mass matrix (6\(\times\)6) and other physical parameters:

\[
M = \begin{bmatrix}
30.0 & 0.0 & 0.0 & ... & ... & ... \\
0.0 & 30.0 & ... & ... & ... & ... \\
0.0 & ... & 30.0 & ... & ... & ... \\
... & ... & ... & ... & ... & ... \\
... & ... & ... & ... & ... & ...
\end{bmatrix}
\]

6. **CMakeLists.txt**
    - Build configuration, ROS 2 dependencies, executable generation, and installation setup.

---

### Tuning Parameters
- **\(K_1\) and \(K_2\)**: Gains for the backstepping control laws.
- **\(\text{adapt\_gain}\)**: Adaptive gain for the linear and nonlinear damping.
- **\(\text{d\_gain}\)**: Adaptive gain for the disturbances and uncertainties.
- **\(M\)** and **\(I_b\)**: Mass inertia matrix and rotational inertia.
- **\(m\)**: Vehicle mass.

---

### Backstepping Controller

Using the Lyapunov proof, we get the following functions for the backstepping controller:

#### Backstepping Variables

\[
z_1 = \eta - \eta_d, \quad z_2 = \nu - \alpha
\]

where \(\alpha\) stabilizes the \(z_1\) variable in the Lyapunov function system.

#### Adaptive Parameters

\[
\tilde{\Theta} = \hat{\Theta} - \Theta^*, \quad \tilde{d} = \hat{d} - d^*
\]

where:
- \(\Theta^*\) and \(d^*\) are the actual parameters.
- \(\hat{\Theta}\) and \(\hat{d}\) are the estimated parameters.
- \(\tilde{\Theta}\) and \(\tilde{d}\) are the estimation errors.

#### Proof of Control Law

We define the Lyapunov function candidate (CLF) as:

\[
V_1 = \frac{1}{2} z_1^\top z_1
\]

By fulfilling the criteria of being positive definite, and \(V_1(0) = 0\), we derive and ensure it is negative definite.

\[
\dot{V}_1 = \frac{d}{dt} \left( \frac{1}{2} z_1^\top z_1 \right) = z_1^\top \dot{z}_1
\]

Substituting \(\dot{z}_1\) with its dynamics:

\[
\dot{z}_1 = J(\eta)\nu - \dot{\eta}_d
\]

Assuming \(\dot{\eta}_d = 0\), we get:

\[
\dot{V}_1 = z_1^\top J(\eta)(z_2 + \alpha)
\]

Choosing \(\alpha\) such that \(z_1\) becomes negative semi-definite:

\[
\alpha = -J(\eta)^{-1}(K_1 z_1)
\]

Substituting \(\alpha\):

\[
\dot{V}_1 = z_1^\top J(\eta)z_2 - z_1^\top K_1 z_1 < 0
\]

---

### Adaptive Control Law

The control input \(\tau\) is defined as:

\[
\tau = -K_2 z_2 + M \dot{\alpha} + C(\nu)\nu - J(\eta)z_1 - F(\nu, \hat{\Theta}) - \hat{d}
\]

where:
- \(K_1\) and \(K_2\) are diagonal gain matrices.
- \(M\) is the mass-inertia matrix.
- \(C(\nu)\) accounts for Coriolis and centripetal forces.
- \(F(\nu, \hat{\Theta})\) models damping using estimated parameters \(\hat{\Theta}\).
- \(\hat{d}\) is the disturbance estimate.

---

### Launch Instructions

To run the controller, use the ROS 2 launch file:

```bash
ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py
```

Make sure to build the package and source the environment:

```bash
colcon build
source install/setup.bash
```

This will load the parameters from the YAML file, start the node, and begin control operations.
