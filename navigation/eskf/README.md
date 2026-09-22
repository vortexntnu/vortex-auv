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
cmake -S navigation/eskf -B /tmp/eskf-build -DESKF_BUILD_ROS=OFF -DCMAKE_BUILD_TYPE=Release
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
