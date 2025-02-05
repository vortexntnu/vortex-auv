## PID controller
The PID controller is defined
```math
\tau = -J_{\Theta}^{-1}(K_p \tilde{\eta} + K_d \dot{\tilde{\eta}} + K_i \int^t_0 \tilde{\eta}(\tau)d\tau)
```

where $\tau$ is the control input, $J_{\Theta}$ is the Jacobian based on Euler angles, $\tilde{\eta} = \eta - \eta_d$ is the pose error and $K_p$, $K_d$ and $K_i$ are tuning matrices.