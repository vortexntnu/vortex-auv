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

## Build and launch

The ukf also introduces the flag, `use_sim` which picks `ukf_params.yaml`
for running the drone in the Stonefish simulator or without flags
to utilize the `ukf_params_real_world.yaml` config for the vehicle:

```sh
colcon build --packages-up-to ukf
ros2 launch ukf ukf.launch.py drone:=nautilus use_sim:=true
```

The node reads the same IMU, DVL and pressure topics as the ESKF, plus a
magnetometer on `topics.magnetometer` (set with `magnetometer_topic:=...`). It
publishes under `ukf/`, so the two can run side by side on the same data.
