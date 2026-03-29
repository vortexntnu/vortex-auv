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

## Action Server
The action server is responsible for handling goal requests and publishing guidance commands. The server will always prioritize new goal request, and will abort ongoing request when getting a new request. The action definition can be found [here](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action).

- Action name: /reference_filter
- Goal type: PoseStamped
- Result type: bool
- Guidance topic: /dp/reference
