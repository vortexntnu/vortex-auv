# Navigation ESKF

This package estimates navigation-frame position/velocity, body-to-navigation
orientation, and body-frame gyro/accelerometer biases. Its nominal state has 16
stored scalars and its right-local error covariance has 15 coordinates ordered
`position, velocity, rotation, gyro_bias, accel_bias`.

## Mathematical contract

- IMU inputs are specific force in m/s² and angular rate in rad/s. Do not remove
  gravity upstream. Navigation z is down; initial orientation is identity and
  establishes a local heading, not geographic North.
- `diag_Q_std` contains continuous-time noise-density standard deviations ordered
  `accel, gyro, gyro-bias random walk, accel-bias random walk`. Bias means have no
  deterministic decay. `diag_p_init` contains variances, not standard deviations.
- Propagation linearizes covariance at the beginning of the nominal interval.
  Van Loan discretization is retained; its matrix exponential is a host baseline,
  not an established deterministic MCU implementation.
- DVL measures body-frame velocity; its right-local attitude Jacobian is
  `skew(R.transpose() * v)`. The ROS adapter rotates sensor covariance and removes
  rotational lever-arm velocity using the latest bias-corrected gyro.
- Pressure measures sensor depth `p_z + (R * lever_arm)_z`. Its Jacobian includes
  attitude. IMU preprocessing removes centripetal and finite-difference angular
  acceleration lever-arm terms. Differentiation may amplify gyro noise; calibrate
  and validate extrinsics and noise under vehicle motion.
- Corrections use positive-definite measurement covariance, a Cholesky solve,
  configurable NIS rejection and Joseph covariance update. Injection transports
  the entire covariance using the exact SO(3) right Jacobian of the reset chart.
- Pose covariance maps local attitude error to parent-frame small rotation error.
  Body twist covariance includes velocity/attitude/bias cross terms and gyro noise.
  Gyro sample noise is treated as independent of the state; lever-arm compensation
  noise/correlations are approximations and need validation for large lever arms.

## Timing, validity and reset

The node requires successful IMU propagation and at least one accepted DVL and
pressure correction before publishing odometry. `eskf/valid` is false during
startup, IMU staleness, and faults. Consumers must monitor it and aiding/NIS
telemetry: true means the initialized inertial solution is current, not that DVL
or pressure remain available or that navigation meets a vehicle accuracy bound.

Odometry/TF retain the last propagated IMU timestamp. The twist header and odometry
child frame identify `base_link`; the pose header identifies `odom`. Initialized
or stale states are not republished with new timestamps.

Nonpositive IMU intervals and invalid samples are rejected. A gap exceeding
`max_imu_dt` or a numerical failure latches a fault. Recovery is explicit:

```sh
ros2 service call /nautilus/eskf/reset std_srvs/srv/Trigger '{}'
```

Reset establishes a new zero-position, identity-attitude prior and awaits all
three sensors again. Coordinate a reset with control; this is not continuous
navigation through a gap. Initial DVL/depth must be consistent with the configured
prior and innovation gates.

Aiding must be monotonic per sensor, recent, and within `max_aiding_skew` of the
latest IMU state. The ESKF applies it at the current state without rewind. This is
an explicit bounded-skew approximation; delayed estimation belongs in a buffered
estimator. DVL quality/lock must be checked by the upstream driver because the
Twist message does not carry that status. Negative/nonfinite covariance is rejected;
zero pressure variance uses the configured positive fallback variance.

## Build and test

The numerical library depends only on Eigen. Tests run with checks enabled even
in release builds:

```sh
cmake -S navigation/eskf -B /tmp/eskf-build -DESKF_BUILD_ROS=OFF -DESKF_WITH_GTSAM=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/eskf-build -j2
ctest --test-dir /tmp/eskf-build --output-on-failure
```

For ROS 2 Humble, build with `colcon build --packages-up-to eskf` and source the
workspace. The following test starts the real node, supplies synthetic messages,
and checks startup validity, frames, outliers, stale output, a gap and reset:

```sh
ros2 run eskf ros_contract_test.py
```

The standalone core tests additionally exercise seeded finite-difference sensor
Jacobians, bias/noise indexing, reset chart transport and covariance export.

## Launch and simulation

```sh
ros2 launch eskf eskf.launch.py drone:=nautilus use_sim:=true
```

The standalone launch loads robot/environment/noise configuration and requires
sensor TFs from `drone_description.launch.py`. `use_sim` selects simulated time,
IMU `/<namespace>/imu/data_raw`, and pressure `/<namespace>/pressure_sensor`.
DVL uses the robot configuration's adapter output. Optional `imu_topic`,
`dvl_topic`, and `pressure_topic` arguments override these routes. The composed
`auv_setup/state_estimation.launch.py` now delegates estimator configuration to
this same launch and starts physical drivers only outside simulation.

Debug outputs default to `/<namespace>/eskf/{odom,pose,twist}` with no TF authority.
Compare them against separate Stonefish truth. Enabling normal outputs for control
requires explicitly moving the simulator truth publishers/adapters away from
`odom/pose/twist` and selecting one odom-to-base TF publisher in the simulation
workspace; this repository cannot change external scenario publishers. The optional
comparison transformer and RPY utility are disabled unless requested.

These changes repair the report's mathematical defects and make the timing,
validity and launch contracts explicit. Native and synthetic ROS tests do not
establish physical sensor calibration, Stonefish control cutover or vehicle safety.

## GTSAM manifold preintegration backend

This branch adds an optional **host** backend using GTSAM's Forster
`ManifoldPreintegration`, `ImuFactor`, bias-random-walk factors, and
`IncrementalFixedLagSmoother`. It shares the ROS sensor processing, gating policy,
validity, reset and output contracts with the repaired ESKF.

The graph owns pose, navigation velocity and both biases at each keyframe.
Every IMU sample enters one preintegrated interval. A keyframe is created when
`keyframe_interval` elapses or an accepted aiding update closes the current
interval. Same-epoch DVL/depth factors reuse one state. Old states are marginalized
after `smoother_lag`. The current full joint pose/velocity/bias covariance is
converted to the ESKF coordinate convention, preserving cross terms. GTSAM bias
order is accelerometer then gyro; ESKF order is gyro then accelerometer.

Between graph updates, a fresh ESKF instance initialized from the optimized state
and covariance supplies high-rate output propagation and innovation gating. Its
predictions are **not factors or priors fed back into the graph**; raw IMU
information is represented there only by the preintegrated factor. This retains
the repaired propagation covariance code rather than introducing a second custom
propagator. Joint marginal extraction and the Van Loan predictor are deliberate
host-prototype costs, not demonstrated MCU or hard-real-time performance.

This first integration retains the adapter's bounded-skew aiding policy. It does
not insert measurements at historical acquisition times, rewind an interval,
accept aiding beyond the skew threshold, or revise marginalized states. Smoothing
still revises retained historical states when new current-time aiding arrives.
A future delayed-aiding adapter must preserve raw samples, split intervals or
associate measurements with retained timestamps and account for resulting timing
approximations explicitly.

### Reproducible dependency build

Use GTSAM 4.2 at `4f66a491ffc83cf092d0d818b11dc35135521612`. The backend rejects a
build with `GTSAM_TANGENT_PREINTEGRATION` enabled at compile time. A default binary
package may use tangent preintegration and is not a substitute for this build.
GTSAM 4.2's smoother requires `gtsam_unstable` as well as `gtsam`.

On Ubuntu 22.04/ROS Humble, install build prerequisites (`build-essential`, `cmake`,
`git`, `libeigen3-dev`, `libboost-all-dev`) and build into an isolated prefix:

```sh
bash navigation/eskf/tools/build_gtsam.sh /tmp/gtsam-forster-build /tmp/gtsam-forster
export CMAKE_PREFIX_PATH=/tmp/gtsam-forster:${CMAKE_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/tmp/gtsam-forster/lib:${LD_LIBRARY_PATH:-}
cmake -S navigation/eskf -B /tmp/gtsam-navigation-build \
  -DESKF_BUILD_ROS=OFF -DESKF_WITH_GTSAM=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gtsam-navigation-build -j2
ctest --test-dir /tmp/gtsam-navigation-build --output-on-failure
```

`ESKF_WITH_GTSAM` defaults to ON on this branch. Use `-DESKF_WITH_GTSAM=OFF` to
build only the repaired ESKF without GTSAM. For ROS, keep the same dependency
prefix/runtime path and build with `colcon build --packages-up-to eskf`.
The runtime default remains `estimator_backend:=eskf`; select GTSAM explicitly:

```sh
ros2 launch eskf eskf.launch.py drone:=nautilus use_sim:=true estimator_backend:=gtsam
# Equivalent backend-selecting entry point:
ros2 launch eskf gtsam.launch.py drone:=nautilus use_sim:=true
ESKF_TEST_BACKEND=gtsam ros2 run eskf ros_contract_test.py
```

`keyframe_interval` defaults to 0.2 s and `smoother_lag` to 2.0 s; set these ROS
parameters in estimator YAML. The lag must exceed the keyframe interval plus the
maximum accepted IMU step. Initial covariance and all four independent process
noise blocks must be positive definite. GTSAM's additional integration covariance
is 1e-8 by default in `GtsamNavigationParams`. Tune sensor densities for the actual
filtered STIM profile instead of treating these defaults as calibrated values.

The new tests compare motion against independent direct integration, bias updates
against reintegration, both sensor-factor Jacobians against finite differences,
and a known aided trajectory across repeated window marginalization. They also
verify that later aiding changes an earlier retained state and that the full
navigation covariance includes cross terms. The same synthetic ROS contract test
runs against either backend. No physical sensors, vehicle control or comparative
accuracy claims are implied by these tests.
