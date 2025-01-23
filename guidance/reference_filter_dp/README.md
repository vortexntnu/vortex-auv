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

## Action Server
