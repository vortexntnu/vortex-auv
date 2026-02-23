# Thrust allocator

The **Thrust Allocator** is responsible for distributing the generalized control forces $\tau \in \mathbb{R}^n$ to the actuators in terms of control inputs $u \in \mathbb{R}^r$. For linear systems this boils down to $\tau = Bu$, where B is the input matrix. The individual control inputs $u_i$ are later passed into thruster_interface_auv.

# Notation

The thrust allocation problem follows the notation of Fossen (2021), Ch. 11.
The variables used in all allocation formulations (unconstrained, pseudoinverse, and QP) are:

### Generalized forces and configuration matrix

- $\tau \in \mathbb{R}^n$, Desired generalized force

- $T_e$, Extended thruster configuration matrix

---

### Actuator forces and extended vectors

- $f_e$, Extended force vector

- $\bar{f}$ defined as, $\; -\bar{f} \le f_{e,i} \le \bar{f}$ is the scalar bound used for load balancing

---

### Weighting matrices and penalties

- $W_f \succeq 0$, Weighting matrix on the extended force vector

- $Q \succeq 0$, Weighting matrix on the slack vector $s$.

- $\beta > 0$, Penalty weight on $\bar{f}$ used for load balancing (QP formulation).

---

### Constraints

- $f_{\min}, f_{\max}$ Lower and upper bounds on the extended force vector $f_e$.

# Interfaces

- **[ThrusterInterface](https://github.com/vortexntnu/vortex-auv/tree/main/motion/thruster_interface_auv)**

# Solvers

### **Pseudoinverse Allocator**

The pseudoinverse allocator follows the unconstrained weighted least–squares formulation
given in Fossen (2021, Eq. 11.27):

$$
J = \min_{f_e} \; ( f_e^\top W_f f_e )
\qquad \text{s.t.} \qquad
\tau - T f = 0,
$$

---

#### **Generalized pseudoinverse (Fossen Eq. 11.35)**

Solving the weighted least–squares problem leads to the **generalized pseudoinverse**

$$
T_w^+
= W_f^{-1} T_e^\top \left( T_e W_f^{-1} T_e^\top \right)^{-1},
$$

where $T_e$ is the extended configuration matrix used in the allocation.

---

#### **Right Moore–Penrose pseudoinverse (Fossen Eq. 11.36)**

If the allocator uses **identity actuator weights**,
i.e. $W_f = I$, then the generalized pseudoinverse simplifies to the **right Moore–Penrose pseudoinverse**

$$
T^+ = T_e^\top (T_e T_e^\top)^{-1}.
$$

For orca there was no big reason to weigh the different actuators since the drone will be using 8 of the same thruster. Therefore the pseudoinverse_allocator solution degenerates to the simpler Right Moore-Penrose pseudoinverse.

---

### **Constrained QP Allocator**

The constrained thrust allocation problem is formulated as a quadratic program (QP) following
Fossen (2021, Eq. 11.38). The optimization variables include the **extended force vector** $f_e$,
a slack vector $s$, and the scalar load-balancing parameter $\bar{f}$. For our intents and purposes it
the load balancing parameter will do more harm than good as different manouvers require some thrusters
to work hard whilst other thrusters to be at rest.

### **Original Fossen Formulation (QP standard form)**

$$
J = \min_{f_e,\, s,\, \bar{f}}
\; ( f_e^\top W_f f_e + s^\top Q s + \beta \bar{f} )
$$

$$
\text{s.t.} \quad
T_e f_e = \tau + s
$$

$$
f_{\min} \le f_e \le f_{\max}
$$

$$
-\bar{f} \le f_{e,i} \le \bar{f}.
$$

### **Implemented QP Formulation (QP standard form)**

$$
J = \min_{f_e,\, s}
\; ( f_e^\top W_f f_e + s^\top Q s)
$$

$$
\text{s.t.} \quad
T_e f_e = \tau + s
$$

$$
f_{\min} \le f_e \le f_{\max}
$$

This QP formulation allows thrust limits, load balancing, soft constraint handling.

# Testing

If you wish to run the tests inside of the tests folder, run the following commands:

#### 1. Build the package together with the tests.

```bash
colcon build --packages-select thrust_allocator_auv --cmake-args -DBUILD_TESTING=ON
```

#### 2. Run colcon test.

```bash
colcon test --packages-select thrust_allocator_auv   --event-handlers console_direct+
```

#### 3. print out the results with the --verbose flag.

```bash
colcon test-result --verbose
```

# Debugging (CasADi)

If the solver behaves unexpectedly it is possible to turn on CasADi's inbuilt spdlogs which show iterations, objective function value and report if the convergence was sucessfull or not.

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```
