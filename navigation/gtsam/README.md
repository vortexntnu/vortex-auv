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
second robot-description publisher.

`trajectory:=rotate` continuously increases the roll, pitch and yaw rotation
parameters at 12, 9 and 6 degrees/s respectively, giving full revolutions every
30, 40 and 60 seconds after ramp-up. The attitude is composed as
`Rz(yaw) Ry(pitch) Rx(roll)` and retains the world-X translation. All motion begins
after five seconds of stationary alignment and ramps up over four seconds. Body-axis
angular rates and accelerations include the coupling between the three rotations;
sensor measurements retain the IMU/DVL lever-arm effects. Truth includes the full
orientation quaternion and all three angular velocity components. The default
`turn` trajectory continues to rotate in yaw only.
The composed body rates are not three constant gyro readings. There is no
inverse Euler-rate calculation at pitch +/-90 degrees, so motion remains smooth
through those attitudes. Euler display angles will fold/wrap; orientation error
is therefore measured as the shortest quaternion rotation angle.

`trajectory:=barrel_roll` keeps the forward body X axis aligned with world X,
translates at 0.3 m/s, and continuously rolls about that axis at 72 degrees/s:
one revolution every five seconds after the same stationary alignment and smooth
ramp. It rests for five seconds and ramps for four seconds; the first complete
revolution occurs at simulation t=12 s, followed by t=17 s, 22 s, and so on.
Pitch and yaw stay zero.
The body origin follows a straight line; the offset IMU traces a small helix.

`trajectory:=square_barrel_roll` runs a finite 3 m x 3 m square followed by a
single forward barrel roll. Forward speed is limited to 0.3 m/s. Each straight
leg uses two-second cosine acceleration and braking ramps, a one-second stop,
a smooth four-second 90-degree yaw rotation while stopped, and a one-second
pause before the next leg. The fourth turn restores the initial heading at the
starting point. The yaw and roll profiles have zero angular velocity and
acceleration at their endpoints; there are no pose, velocity or acceleration
jumps. This is a stop-turn-go square, not a banked moving-corner maneuver.

| Simulation time | Body-origin motion |
| --- | --- |
| 0--5 s | Stationary alignment |
| 5--17 s | First 3 m leg, forward along initial X |
| 17--23 s | Stop, yaw 90 degrees, pause |
| 23--35 s | Second leg |
| 35--41 s | Stop, yaw 90 degrees, pause |
| 41--53 s | Third leg |
| 53--59 s | Stop, yaw 90 degrees, pause |
| 59--71 s | Fourth leg, returning to the starting point |
| 71--77 s | Stop, align with original heading, pause |
| 77--79 s | Accelerate forward |
| 79--84 s | One full 360-degree roll while moving at 0.3 m/s |
| 84--86 s | Brake to a stop, upright, 2.1 m forward of the start |
| 86 s onward | Remain stationary |

The single roll takes five seconds including smooth angular acceleration and
deceleration, so its peak angular rate is 135 degrees/s rather than a constant
72 degrees/s. This scenario defaults to a 100-second run. Its timed DVL dropout
is disabled, making the upright square fully aided under the simple lock model;
the roll still loses lock when tilted. `dropout_start` and `dropout_end` launch
arguments can enable an additional outage (equal values disable it).
The square refers to the body origin; the offset IMU moves around it during yaw
turns and the barrel roll. This remains a prescribed kinematic maneuver, not a
claim that Nautilus's thrusters can realize the specified accelerations.

For all trajectories, `/nautilus/dvl/bottom_lock` (`std_msgs/Bool`) reports
simulated lock at each DVL sampling instant. The retained flag is true only when
the DVL +Z boresight is within `dvl_max_tilt_deg` of world down and the configured
timed dropout is inactive. The default limit is 30 degrees, a configurable
simulation assumption, not a Nortek specification. No DVL twist is published
while lock is false. IMU and truth publication continue. Lock and velocity
updates resume when the sensor returns within the limit; GTSAM may gate those
measurements using its usual NIS check. Its status warns about dead reckoning
after the configured DVL timeout (1 second by default).

At the steady 72 degrees/s barrel-roll rate, each upright lock window lasts
about 0.833 s and yields roughly six or seven DVL measurements at 8 Hz. Lock is
unavailable for about 4.167 s between windows. The separate t=20--25 s dropout
also suppresses measurements even if the sensor is upright.

This remains a kinematic test with a simplified flat-seabed visibility model;
individual beam returns, altitude/range, acoustic effects, reacquisition time,
vehicle dynamics and thruster feasibility are not modeled. The bottom-lock flag
is simulator telemetry, not a new estimator input requirement.

When Stonefish or another vehicle stack is running, isolate this standalone
simulation and its Foxglove Bridge with `ROS_DOMAIN_ID=42` in both terminals.
Run only one simulation instance. After the sensor generator finishes, stop the
remaining launch with Ctrl+C before starting another run.

The hardware launch loads the existing Nautilus description. If already running,
use `start_description:=false`. The node waits for sensor TF; it does not assume
identity transforms when TF is missing. All parameters are startup-only.

| Direction | Relative topic | Message |
| --- | --- | --- |
| Input | imu/data_raw | sensor_msgs/Imu |
| Input | dvl/twist | geometry_msgs/TwistWithCovarianceStamped |
| Output | gtsam/odom | nav_msgs/Odometry |
| Output | gtsam/status | diagnostic_msgs/DiagnosticArray |

Default namespace/frame prefix is `nautilus`. IMU messages must have frame
`nautilus/imu_link`, acceleration as **specific force including gravity response**
in m/s², and rates in rad/s. Orientation is ignored. DVL messages must have frame
`nautilus/dvl_link`, velocity in m/s at that sensor's origin, and a positive
definite linear 3x3 covariance in the upper-left block of the ROS 6x6 covariance.
Invalid bottom lock must be suppressed upstream or marked by invalid covariance.
Zero/unknown covariance, negative validity markers, NaNs and bad frames are
rejected rather than given an invented certainty. Drivers are outside this package.

Odometry pose is in `nautilus/odom`, with child frame **`nautilus/imu_link`** and
twist in IMU axes. The existing controller expects `base_link`: transform this
odometry to the body/control origin before connecting it, including the
rotational velocity lever arm and covariance. A topic remap alone is insufficient.
The existing `odom_transformer` provides the repository's adapter pattern, but
currently resets the origin and copies covariance without transforming it; it is
not a covariance-correct drop-in adapter for this output. This package does not
change or automatically connect controller inputs.

`publish_tf` is off by default. When enabled, the node publishes the equivalent
`odom -> base_link` transform, preserving the URDF's `base_link -> imu_link` edge
and avoiding a second parent for the IMU. Only one odometry source may publish
that edge.

## Initialization, timing and limits

Keep the vehicle still for two seconds after valid IMU and TF become available.
The configurable window checks angular rate, acceleration magnitude and sample
variance. It estimates tilt and gyro offset, with an uncertain zero accelerometer
bias prior. Motion restarts alignment. This is coarse startup alignment, not a
factory calibration or thermal warm-up procedure. It cannot distinguish slow
constant motion from rest or determine absolute yaw/all accelerometer biases.
Earth rotation is neglected; its small projection may enter the initial gyro bias.

After initialization, the origin persists while carrying/launching the vehicle.
Without DVL it dead-reckons and uncertainty grows. IMU+DVL do not ensure bounded
position/heading drift, and controller stability is not established here.

The default five-second lag is an optimization-history tuning value, not output
latency. Old states are marginalized and factor slots reused. Keyframes are
created at DVL measurement times and at most 100 ms apart. DVL keyframes closer
than 5 ms are rejected to bound graph growth. IMU samples are zero-order held on
`[sample_time, next_sample_time)` and intervals are split exactly at keyframes.
Sorted input buffers allow 250 ms of reordering. Samples behind committed time
are rejected; there is no retroactive insertion outside that processing horizon.

High-rate output replays the uncommitted IMU tail from the latest corrected state.
Its covariance propagates the full pose/velocity/bias joint covariance and IMU
noise. This is a first-order approximation: process noise correlations within an
interval and reuse of gyro samples by DVL/IMU factors are not modeled exactly.
Pose covariance is mapped to world XYZ and fixed-axis rotation; twist covariance
is in IMU axes, with velocity/attitude/bias cross terms retained.

Publication is capped at 125 Hz with strictly advancing measurement timestamps.
Stale IMU or a clock mismatch stops output. A committed IMU gap over 100 ms,
buffer overflow, or optimizer failure latches a fault requiring node restart;
no fabricated measurements bridge a gap. DVL dropout retains IMU prediction and
reports WARN. Status reports initialization, data freshness, rejection counters,
bounded graph sizes and faults. Bias values are never a public ROS interface.

## Noise sources

STIM300 TS1524 rev.27 (August 2021), printed pages 6–9, Tables 6-3–6-5:

| Profile | Accel velocity random walk | Gyro angle random walk |
| --- | --- | --- |
| 10 g (provisional default) | 0.07 m/s/√h | 0.15 °/√h |
| 30 g | 0.21 m/s/√h | 0.15 °/√h |

Convert hourly square-root units by dividing by 60; convert gyro degrees to
radians. These are continuous white-noise densities: sampled rate noise has
standard deviation `density / sqrt(dt)`. Bias random walks use independent
increments `bias_density * sqrt(dt)`. The simulation defaults to accelerometer
offset `[0.003, -0.002, 0.001]` m/s² and gyro offset `[2e-5, -1e-5, 1e-5]` rad/s;
these and the drift densities (1e-5 m/s²/√s and 1e-7 rad/s/√s) are explicit
modeling assumptions, not datasheet claims. Both profiles can override densities.

The FFI report *Rate table tests of low-cost inertial measurement units*,
2014/01970, section 3.2 (printed pages 27 onward), reports tests of an older
STIM300 unit. It motivates stress testing bias/noise beyond nominal specifications.
`stress_scale` multiplies simulator white-noise and bias-drift densities only;
it is a sensitivity study, not a fitted FFI model. Allan bias-instability minima
are not used as bias random-walk coefficients. No claims are made about calibration
of the current physical sensor. Nucleus default single-ping DVL sigma is 0.005 m/s
from the supplied product datasheet; long-term scale errors are not white noise.

References: [IMU factor note](https://gtsam.org/notes/imu-factor/),
[GTSAM 4.3.0](https://github.com/borglab/gtsam/releases/tag/4.3.0), and the supplied
STIM300 datasheet, Nucleus documentation and FFI report. The PDFs are not copied
into this package.

## Local validation

See [the simulation audit](SIMULATION_AUDIT.md) for truth-isolation experiments,
numerical integration limits, covariance checks, and the DVL model's abstraction.

Built with ROS 2 Humble, GCC 13 and GTSAM 4.3.0. C++ tests cover numerical DVL
and output-covariance Jacobians, stationary alignment, invalid/reordered data,
turning motion, outlier rejection, DVL dropout/recovery, bounded history over
120 simulated seconds, and marginalization against a batch reference. Python
tests verify noise scaling and that simulation mounting geometry matches the
existing Nautilus URDF. The ROS launch test checks IMU-frame messages,
measurement timestamps, diagnostics, absence of bias topics and process exit.

The seeded 30-second C++ trajectory includes initial bias offsets, bias drift,
white sensor noise, turning and a two-second DVL dropout:

| Profile | Position RMSE | Velocity RMSE | Final attitude error |
| --- | --- | --- | --- |
| 10 g | 0.0171 m | 0.00529 m/s | 0.00139 rad |
| 30 g | 0.0176 m | 0.00725 m/s | 0.00136 rad |

These are reproducible synthetic regression results, not sensor calibration or
hardware accuracy guarantees. Absolute heading and bias observability remain
motion-dependent; the tests do not require every bias to converge to its truth.
