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

Orientation (SO(3) PDAF):

Orientation is handled by a dedicated PDAF-style filter operating on
the SO(3) manifold. Each track
maintains:

- a quaternion mean q (the track orientation), and
- a small Gaussian error state in the 3D tangent space (so(3)) around
  that mean.


The log and exp maps are mathematically defined for all unit quaternions
but the Kalman/PDAF linearization on the tangent space assumes small
errors. The implementation gates measurements and uses configurable
covariances to keep tangent residuals small; if large-angle residuals
occur they will either be gated out or yield small β (limited effect).

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
