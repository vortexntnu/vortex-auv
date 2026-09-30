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
8. **Checked-in Foxglove metric arithmetic passed runtime checks.** Float64Array
   covariance, full position cross-covariance, exact timestamp pairing, both
   arrival orders, duplicate rejection, frame mismatch, invalid covariance, and
   clock-reset handling were exercised. An independent analytic example gives
   NEEDS=5 for error [1,2,3] and covariance [[2,1,0],[1,2,0],[0,0,3]]. This does not
   inspect the user's saved Foxglove layout or compile its TypeScript types.

The package build and all 29 existing tests passed after the scenario change.
An isolated 18-second ROS run measured median message-timestamp rates of
1000 / 8 / 125 Hz for IMU / available DVL / odometry. Observed wall rates over
t=3--7 s were approximately 998 / 8 / 125 Hz. Full turns at t=12 and t=17 s,
lock loss, recovery, and continued odometry were checked. Measured rates are
machine-load dependent, not hard real-time guarantees.

A separate 75-second sensor-only replay of the new 1000/8 Hz, five-second-roll
scenario gave these results (position RMSE evaluated after t=5 s):

| Sensor noise | Position RMSE | Final position error | Final position NEEDS |
| --- | --- | --- | --- |
| Disabled, zero biases | 0.1283 m | 0.2096 m | 7.45 |
| Enabled, seed 42 | 0.1769 m | 0.2838 m | 14.48 |

Thus the higher rate reduces numerical error but does not remove it; the new
scenario's covariance cannot yet be called validated either. These are single
runs, not an ensemble consistency test. The graph was not returned to force NEEDS
inside a desired interval.

## Meaning and limitations of the DVL model

This is a **Cartesian measurement-level model**, representing the output of a
bottom-track velocity solution, not the internal acoustics or the Nucleus INS.
It computes velocity at the DVL origin, rotates it into instrument XYZ axes,
and adds independent Gaussian noise with standard deviation 0.005 m/s per axis.
The estimator uses the fixed IMU-to-DVL transform and measured gyro minus its
estimated bias to account for rotational lever-arm velocity.

The supplied product sheet specifies 0.5 cm/s single-ping standard deviation.
Using that value independently and isotropically on XYZ is an approximation;
the specification does not establish a constant full Cartesian covariance.
The manual's BottomTrackData includes per-axis uncertainty/FOM, validity bits,
and timing offsets. A hardware adapter must use valid fresh bottom-track XYZ,
appropriate timestamps, and a defensible covariance, rejecting invalid fields.
It must not substitute Nucleus INS velocity or repeatedly count a held ping as
new independent data.

The manual specifies 1--8 Hz internal acoustic triggering, with maximum rate
limited by configured range; 8 Hz applies at range <=9 m in its table. Interleaved
altimeter/current-profile operations can reduce bottom-track update rate.
The chosen 8 Hz simulation is an explicit simplification for the pool scenario.
Pool depth alone does not determine DVL-to-bottom range.

Not modeled: sequential three-beam acquisition, beam intersections, pool depth/
altitude, seabed slope, range-dependent rate/uncertainty, correlation between
axes, acoustic multipath/noise, sound-speed/scale errors, mounting uncertainty,
invalid sentinels, acquisition delay, or hysteresis. The 30-degree tilt lock
threshold is an assumed test rule, not a Nortek limit. It switches immediately;
no velocity is published while false. The lock Bool has no timestamp and remains
latched after the simulator stops, so it alone is not a freshness indicator.

## Other interpretation limits

- STIM300 noise magnitudes come from the supplied datasheet, but sensor bandwidth,
  group delay, temperature, quantization, scale errors, misalignment, saturation,
  vibration, and Earth rotation are omitted. Random-walk bias parameters and
  initial biases are assumptions, not a fit to the FFI rate-table data.
- Truth and estimator assume identical mounts, gravity, and synchronized clocks.
  These are disclosed perfect-calibration assumptions, not hidden truth updates.
- The trajectory is kinematically consistent. No thruster, hydrodynamic, buoyancy,
  controller, or pool-boundary dynamics establish that Nautilus can execute it.
- Position covariance is in world axes and matches the metric's position error.
  The graph and propagated covariance remain first-order approximations, including
  simplified within-interval bias noise and reused gyro measurement correlations.
- DVL NIS is computed before adding its factor. It is currently an internal gate,
  not a published metric; post-fit factor residual is not a replacement for NIS.
- RMSE summarizes received, timestamp-matched outputs. Invalid covariance causes
  the Foxglove script to omit that sample, including its RMSE contribution. Missing
  outputs therefore need separate monitoring; RMSE alone is not an availability
  test. Plotting header time avoids confusing arrival delay with physical error.
- Position and heading drift remain unbounded without absolute aiding. An
  individual correction can increase true error, and low NEEDS is not proof of
  good accuracy (the no-DVL ablation has large error and very large uncertainty).

## Repeating the audit

Build with `BUILD_TESTING=ON`. `audit_replay` is a local test executable, not an
installed ROS interface. From the workspace, after sourcing `install/setup.bash`:

```bash
PYTHONNOUSERSITE=1 python3 src/vortex-auv/navigation/gtsam/test/audit_simulation.py \
  --binary /home/vortex/ros2_ws/build/gtsam_navigation/audit_replay \
