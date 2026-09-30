# Simulation and estimator audit — 2026-09-27

No truth-to-estimator information leak was found in the inspected implementation
or the ROS differential experiment. This is a valid kinematic sensor-fusion test,
but it is not a calibrated STIM300/Nucleus emulator or a vehicle dynamics model.
Numerical integration error and covariance calibration remain material limits.

## Data boundary

The data flow is:

```text
analytic rigid-body trajectory
  -> sensor-origin acceleration/rates/velocity -> bias and noise -> IMU / DVL
  -> IMU-origin truth -----------------------------------------> evaluation
IMU / DVL -> estimator -> odometry -----------------------------> evaluation
```

`sensor_model.py` derives ideal sensors from the prescribed trajectory. This is
the correct use of ground truth inside a simulator. Random bias and measurement
noise are added afterward. `navigation_node.cpp` consumes only acceleration,
angular velocity, DVL velocity/covariance, timestamps, and fixed mounting TF.
It does not consume IMU orientation, simulation truth, bottom-lock telemetry,
trajectory parameters, RNG seed, or simulated biases.

The simulator subscribes to estimated odometry solely to accumulate error
statistics. `on_estimate()` does not change the trajectory, RNG, sensor values,
lock decisions, or simulation index. Static TF communicates exact assumed
calibration, not time-varying true pose. No truth-based dynamic TF is generated.

Startup uses noisy IMU averages to estimate tilt and gyro bias. Zero initial
position/yaw establish a local gauge; zero velocity assumes a stationary start.
The simulator actually rests for five seconds. Initial accelerometer bias is
not disclosed to the estimator. Priors are heuristic, and not every uncertain
initial quantity is sampled from its prior in the simulation.

## Experiments and findings

The original audit used 200 Hz IMU, 5 Hz DVL, and a 30-second barrel-roll period.
The subsequent requested scenario uses 1000 Hz IMU, 8 Hz DVL, a five-second roll,
and 125 Hz output. Historical results below must not be presented as accuracy
results for the new scenario. Local raw evidence is in `.deps/audit/` (ignored).

1. **ROS truth isolation passed.** Four production ROS nodes received identical
   sensor samples except for a positive-control DVL perturbation. At 479 common
   timestamps, both absent truth and corrupted truth (position shifted by
   hundreds of meters, false orientation) produced exactly the same output
   state and covariance as the reference. A false, available IMU orientation
   also had no effect. Perturbing DVL by 0.02 m/s did change the estimate.
   Node subscription discovery found no truth or lock subscription; the TF
   listener separately receives fixed calibration transforms.
2. **Sensor-only replay passed.** Identical measurements gave bit-identical
   outputs. Altering only the separate evaluation truth changed the reported
   error, not the estimate. The standalone C++ replay accepts no truth fields.
3. **Measurement ablations changed performance.** On the original 170-second
   barrel-roll fixture, nominal position RMSE after t=5 s was 0.581 m. Removing
   DVL while retaining exactly the same IMU samples raised it to 75.634 m.
   Adding 0.02 m/s² to measured acceleration X after startup raised it to
   5.517 m. On the 75-second straight fixture, a 1% DVL scale error raised final
   position error from 0.0557 m to 0.2035 m and final position NEEDS from 2.83
   to 86.09. These are deliberate sensitivity tests, not sensor specifications.
4. **Finite-step integration error is significant during rotation.** With
   noise and biases disabled, original barrel-roll maximum position error was
   1.832 / 0.917 / 0.460 m at 200 / 400 / 800 Hz. Corresponding three-axis
   rotation final error was 21.96 / 10.98 / 5.49 m; that trajectory had no accepted
   DVL after t=9 s. Approximately halving error when doubling rate is evidence
   of first-order discretization error. The estimator holds each instantaneous
   IMU sample over the next interval, whereas truth is continuous analytic
   motion. Existing C++ fixtures using matched discrete kinematics did not
   expose this limit. Raising sample rate helps but does not eliminate it.
   Improving integration must not be replaced by changing truth to imitate the
   estimator's integration errors.
5. **Covariance consistency is not established.** Twelve independent noise
   seeds at t=75 s gave position ANEES 2.294 for straight motion and 6.114 for
   the original barrel roll. Under ideal Gaussian/prior assumptions, the
   nominal 12-trial 95% interval is [1.778, 4.536]. Here initial biases are fixed,
   priors are heuristic, and deterministic integration error is present, so
   this is a diagnostic comparison rather than a formal consistency verdict.
   The barrel-roll result argues against calling the covariance validated.
   Consecutive samples of one run are correlated and are not independent trials.
6. **The delayed correction after loss of lock was reproduced.** In the original
   seed-42 replay, lock fell at t=159.6 s. The last valid DVL sample at t=159.4 s
   became active at output t=159.7 s through the 250 ms reorder buffer. Position
   error jumped from 0.646 m to 0.896 m. This was a delayed measurement correction,
   not truth moving or a new measurement being fabricated during loss of lock.
7. **Noise scaling and geometry checks passed.** Empirical white-noise and bias
   increment variances were within 2.4% of their configured values over 20,000
   samples. Largest cross-channel correlation was 0.0131. Finite differences of
   sensor positions independently verify rotational velocity and tangential/
   centripetal acceleration. Quaternion/rotation consistency also passes.
