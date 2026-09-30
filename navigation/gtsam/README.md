# GTSAM navigation

IMU-origin navigation using one STIM300 and Nucleus bottom-track velocity.
The estimation library is independent of ROS; the component handles messages,
static TF, timestamps and publication. Biases are internal states, with no bias
topics. Maintainer: Patrick Sheehan <patricsh@stud.ntnu.no>.

For Nautilus dynamics with **GTSAM feeding DP**, joystick control and the existing
Foxglove dashboard, see [Stonefish integration](STONEFISH_INTEGRATION.md).
The standalone synthetic sensor simulation remains available separately.

## Frames and state

The estimated pose is `odom_T_imu`, velocity is internally in `odom`, and angular
velocity and IMU biases are in IMU axes. The state has 15 independent dimensions
(16 numbers when orientation is serialized as a quaternion). Gravity is fixed at
`[0, 0, +9.81]` in a local down-positive frame. Yaw zero is the initial heading;
it is not geographic north. Position zero is the initial IMU location.

Per the IMU-frame design, `body_P_sensor` is identity: GTSAM's integration
reference is the IMU itself. No acceleration lever-arm correction or rotation
is applied to raw IMU messages. The existing Nautilus static TF provides:

| Pose relative to base_link | XYZ (m) | RPY (rad) |
| --- | --- | --- |
| imu_link | -0.100, -0.001, 0.085 | 0, 0, 0 |
| dvl_link | -0.130, 0.003, 0.2414 | 0, 0, 3.14159 |

The node reads `imu_T_dvl` from TF, which composes these fixed transforms. It
does not estimate mounting geometry. The URDF explicitly marks the IMU position
as needing verification after mounting; these values are simulation assumptions.
The simulator includes the IMU's tangential and centripetal accelerations.

The DVL factor predicts `R_DI * (R_WI.transpose() * v_WI + omega_I.cross(t_ID))`.
This includes gyro bias and the IMU-to-DVL velocity lever arm. DVL covariance is
in DVL axes, and only bottom-track velocity is accepted by contract. Do not feed
Nucleus INS velocity or water-track velocity into this input.

## Dependencies and build

Use ROS 2 Humble, C++20, system Eigen, and **GTSAM 4.3.0**, pinned to commit
`71a25ca36c084cbad1f872e812d6d97fbadfdb05`. GTSAM is linked as an external library;
its source and build are kept in this package's ignored `.deps` directory.
No upstream source files are modified or vendored into the branch.

From `src/vortex-auv/navigation/gtsam`:

```bash
./scripts/build_gtsam.sh
source /opt/ros/humble/setup.bash
# Avoid unrelated user-installed Python plugins when configuring/running ROS tests.
export PYTHONNOUSERSITE=1
cd /home/vortex/ros2_ws
colcon build --packages-select gtsam_navigation --cmake-args \
  -DGTSAM_DIR=/home/vortex/ros2_ws/src/vortex-auv/navigation/gtsam/.deps/install/lib/cmake/GTSAM \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
colcon test --packages-select gtsam_navigation --event-handlers console_direct+
colcon test-result --verbose
```

The compiler checks that the default backend is `TangentPreintegration`.
GTSAM build options must have `GTSAM_TANGENT_PREINTEGRATION=ON` and
`GTSAM_LIEGROUP_PREINTEGRATION=OFF`. This is manifold-aware navigation using
GTSAM's efficient default tangent-space formulation. In 4.3.0 the
`IncrementalFixedLagSmoother` is part of the stable library; `gtsam_unstable`
is not needed. The graph uses IMU factors and separate bias random-walk factors.

## Run

```bash
ros2 launch gtsam_navigation simulation.launch.py
ros2 launch gtsam_navigation simulation.launch.py trajectory:=straight noise:=false
ros2 launch gtsam_navigation simulation.launch.py trajectory:=rotate duration:=300.0
ros2 launch gtsam_navigation simulation.launch.py trajectory:=barrel_roll duration:=300.0
ros2 launch gtsam_navigation simulation.launch.py trajectory:=square_barrel_roll
ros2 launch gtsam_navigation simulation.launch.py imu_profile:=stim300_30g seed:=7
ros2 launch gtsam_navigation navigation.launch.py
```

Run simulation and hardware launches separately. Simulation publishes `/clock`
and uses fixed 1000 Hz IMU and 8 Hz DVL samples by default, with 125 Hz odometry
publication. Override these using `imu_rate`, `dvl_rate` and `publish_rate` launch
arguments. Rates describe the configured simulation timeline; wall-time throughput
depends on available processing capacity. A five-second DVL dropout
starts at t=20 s, except in `square_barrel_roll`, which disables that extra outage.
Truth is published for comparison only; the generator prints
position, IMU-frame velocity and quaternion orientation-angle RMSE at completion. Change duration, seed,
trajectory, noise or stress_scale through launch arguments. The simulator's
static transforms reproduce the existing URDF and should not be launched with a
