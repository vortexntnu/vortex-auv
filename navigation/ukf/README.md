# Navigation UKF

An unscented Kalman filter for the AUV, built to be compared against the
[ESKF](../eskf/README.md). Same sensors, same state, same tuning layout, so the
only thing that differs is how the uncertainty is propagated.

## State

The state is stored as a `State` struct, the same as `NominalState` in the ESKF.
The covariance lives in a 15 dimensional tangent space indexed by `StateIndex`.

| block      | `State`      | `StateIndex` |
| ---------- | ------------ | ------------ |
| position   | `pos`        | 0..2         |
| velocity   | `vel`        | 3..5         |
| attitude   | `quat`       | 6..8         |
| gyro bias  | `gyro_bias`  | 9..11        |
| accel bias | `accel_bias` | 12..14       |

Attitude error is right-local, `q ⊞ δθ = q * Exp(δθ)`, the same convention as the
ESKF, and navigation z is down.

## Layout

```
include/ukf/
  typedefs.hpp                    Eigen sizes, indices, State, ImuInput, filter structs
  lie/SO3.hpp                     Exp and Log on SO(3)
  models/manifold.hpp             Manifold and SensorModel concepts
  models/strapdown_ins.hpp        3D strapdown INS, f, Q and the state manifold
  models/measurement_models.hpp   DVL, depth and magnetometer
  transforms/unscented_transform  sigma points through any f, between two manifolds
  filters/ukf                     predict and update
  ros/ukf_ros.hpp                 ROS 2 node
src/                              mirrors include/
config/ukf_params.yaml
launch/ukf.launch.py
```

Templated code lives in a `.tpp` next to its header, like `eskf.tpp`.

A model is anything that satisfies the concepts in `models/manifold.hpp`: a
`dimension`, a `Point` type, and `composition_plus` / `composition_minus`. A
sensor adds `h` and `R`. Adding a sensor means writing one class and calling
`ukf.update(..., sensor)`, nothing else changes.

## Magnetometer

The IMU, DVL and depth sensor cannot observe absolute yaw, so the UKF also
uses the magnetometer in the Nortek Nucleus. It measures the earth's magnetic
field in the sensor frame, in tesla. The Nucleus sits next to the DVL, so the
field is rotated into `base_link` with the `dvl_link` transform.

The model predicts the field as `h(x) = R^T * reference_field`, where `R` is
the attitude of the state and `reference_field` is the earth field in the
navigation frame (NED). The measurement is a plain vector, so the innovation is
a plain difference. Updates are gated on NIS like the other sensors
(`magnetometer_nis_threshold`) and the NIS is published on `ukf/nis_magnetometer`.

Set `sensors.magnetometer` in the params file:

| parameter                    | meaning                                                  |
| ---------------------------- | -------------------------------------------------------- |
| `reference_field`            | earth field for the site in NED [T]                      |
| `measurement_noise_std_diag` | noise std per axis [T], keep it above the datasheet value |
| `use_tf_transform`           | take the sensor rotation from TF instead of parameters   |

A wrong `reference_field` (declination, inclination) or a constant offset from
the vehicle (hard iron) that the filter does not model shows up as a yaw error.

### In the simulator

Stonefish has no magnetometer, so `stonefish_sim_interface` makes one from the
true attitude. It rotates its own earth field into the sensor frame, adds a hard
iron offset and Gaussian noise, and publishes `sensor_msgs/MagneticField` on
`nucleus/mag` at the rate of the truth odometry (10 Hz). The values are in
`stonefish_sim_interface/config/magnetometer.yaml`. They differ from the
filter's on purpose, so the filter is tested against a mismatch like on the
real vehicle. The noise seed is fixed so runs are repeatable.

## Build and launch

The ukf also introduces the flag, `use_sim` which picks `ukf_params.yaml`
for running the drone in the Stonefish simulator or without flags
to utilize the `ukf_params_real_world.yaml` config for the vehicle:

### Launch and build UKF package standalone:

```sh
colcon build --packages-up-to ukf
ros2 launch ukf ukf.launch.py drone:=nautilus use_sim:=true
```

### UKF Consistency testing shell script:

1. Copy the .sh script into same level as your src/ directory
   for example:

```sh
cp -r ukf/scripts/launch_ukf_sim.sh ~/ros2_ws/
cd ~/ros2_ws/ && chmod +x launch_ukf_sim.sh
```

2. Run the shell script, make sure stonefish, foxglove and ukf packages work.
   Advised to run the build simulator script beforehand.

```sh
cd ~/ros2_ws/ && ./launch_ukf_sim.sh
```

The node reads the same IMU, DVL and pressure topics as the ESKF, plus a
magnetometer on `topics.magnetometer` (set with `magnetometer_topic:=...`). It
publishes under `ukf/`, so the two can run side by side on the same data.
