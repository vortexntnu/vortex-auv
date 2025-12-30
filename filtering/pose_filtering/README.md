# pose_filtering

ROS 2 package providing pose-based multi-target tracking using an IPDA
filter for position together with independent orientation logic.

This package transforms incoming pose-like messages into a common
target frame, associates measurements to tracks using spatial and angular
gating, maintains an IPDA filter per track for position estimation and
existence management, and performs a separate orientation update by
averaging measurement quaternions and applying spherical linear
interpolation (slerp) to smoothly update the track orientation.

## Features

- Multi-target tracking using an IPDA (Integrated Probabilistic Data
  Association) filter for 3D position state estimation.
- Separate orientation handling: angular gating, quaternion averaging and
  slerp-based smoothing so orientation updates remain numerically stable
  and physically meaningful.
- TF2-aware subscriptions (message_filters + tf2 message filter) so
  incoming messages are automatically dropped until transform to the
  configured target frame is available.
- Supports several ROS message inputs (see below) and publishes a
  `geometry_msgs/PoseArray` containing the current tracked poses.

## Supported input messages

The node accepts the following message types as input measurements:

- `geometry_msgs/msg/PoseStamped`
- `geometry_msgs/msg/PoseArray`
- `geometry_msgs/msg/PoseWithCovarianceStamped`
- `vortex_msgs/msg/LandmarkArray`

The code uses a templated message type (PoseMsgT) and checks at
compile-time which types are supported. Incoming measurements are
transformed to the configured `target_frame` before being processed.

## Outputs

- `geometry_msgs/msg/PoseArray` — the current set of tracked poses in the
  configured target frame. Each Pose contains the track position and a
  quaternion orientation representing the track's current orientation.
- (DEBUG) `vortex_msgs/msg/PoseEulerStamped` topics with
  measurement-level and track-state-level pose messages when debug mode
  is enabled. Only publishes the first element of incoming measurements and stored tracks.


## How the filtering works

Position (IPDA):

- Each track runs a 3D IPDA filter instance (a Gaussian state: mean +
  covariance) where the motion model is a small constant-dynamics model
  and the sensor model maps state to direct position measurements.
- IPDA takes care of measurement association probabilities, gating and
  existence management (confirmation / deletion thresholds). This
  provides robust data association for cluttered measurements and
  handles tracks appearing/disappearing over time.

Orientation (independent logic):

- Orientation is handled separately from the IPDA position filter.
  Instead of embedding orientation inside the Gaussian state, the
  track stores a quaternion orientation and updates it with measurement
  quaternions when appropriate.
- Angular gating: for each track the manager performs an angular gate
  test that compares the current track quaternion to a measurement's
  quaternion. Measurements with an angular distance above the configured
  `max_angle_gate_threshold` are not associated with that track for
  orientation updates. This prevents spurious orientation jumps from
  outlier measurements.

Quaternion averaging and Slerp:

- When multiple measurements pass gating and are associated with a
  track, the manager extracts the corresponding quaternions and computes
  a (robust) mean orientation. The implementation collects the
  measurement quaternions and derives a representative quaternion —
  e.g. via a mean in quaternion space (the
  implementation collects and reduces the quaternions used for that
  update).
- To smoothly update the stored track orientation towards the measured
  mean, the node performs a spherical linear interpolation (slerp)
  between the current track quaternion q_current and the measurement
  mean quaternion q_meas_mean:

  q_new = slerp(q_current, q_meas_mean, alpha)

  where `slerp(q0, q1, t)` returns the interpolation at fraction `t`
  along the shortest rotation between `q0` and `q1`.

- The `alpha` parameter controls how aggressively the orientation is
  updated in a single tracking step:
  - `alpha = 0.0` : keep the current orientation (no change)
  - `alpha = 1.0` : instantly jump to the measurement mean
  - `0.0 < alpha < 1.0` : smooth interpolation; the closer alpha is to
    1.0 the faster the track follows measurements

### Adaptive alpha computation

The package computes the slerp `alpha` parameter adaptively using both the current and previous quaternion stored on the track and the mean quaternion built
from the current associated measurements.

Let
- q_k     be the current track orientation
- q_{k-1} be the previous track orientation
- q̄_z    be the mean quaternion of the associated measurements

Define angular discrepancies on SO(3):

- θ_k     = d(q_k, q̄_z)
- θ_{k-1} = d(q_{k-1}, q̄_z)

where d(·,·) is the quaternion geodesic distance (Eigen::Quaterniond::angularDistance).

Effective measurement disagreement:

- θ = max(θ_k, θ_{k-1})

Adaptive SLERP gain:

- α = clamp( exp( -θ / σ_ori ), α_min, 1 )
  with σ_ori = 0.1

Orientation update:

- q_{k+1} = slerp(q_k, q̄_z; α)

## Library export and usage

The core tracking logic (`PoseTrackManager`) is built and exported as a
shared library by this package so other packages can reuse the track
management functionality without pulling in the ROS node.

CMake / ament usage

In a consuming CMake-based package (ament_cmake) you can link against
the library like this:

```cmake
find_package(pose_filtering REQUIRED)

add_executable(my_node src/my_node.cpp)

# Link to the exported library target (the package exports the target
# with the same name as the package: `pose_filtering`)
target_link_libraries(my_node PRIVATE pose_filtering)

```

The package installs its public headers under `include/` so simply
including the header below in your code will work after linking:

```cpp
#include <pose_filtering/lib/pose_track_manager.hpp>

int main() {
  vortex::filtering::TrackManagerConfig cfg;
  // configure cfg.ipda, cfg.dyn_mod, cfg.sensor_mod, cfg.existence, etc.

  vortex::filtering::PoseTrackManager manager(cfg);

  std::vector<vortex::utils::types::Pose> measurements;
  double dt = 0.1; // seconds
  manager.step(measurements, dt);

  const auto& tracks = manager.get_tracks();
  // iterate tracks for id, pose, existence probability, etc.
}
```
