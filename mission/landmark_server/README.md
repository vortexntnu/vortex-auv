# landmark_server

Builds the map of the course. It estimates the pose of every detected object
in the `map` frame, keeps them for the whole run, corrects odometry drift
through `map -> odom`, and publishes TF frames the mission can navigate to.
It never moves the vehicle.

## How it works

Keyframes and landmarks live in one iSAM2 graph. A new keyframe is added
every `keyframe_dist_m` or `keyframe_time_s`, with odometry, depth and
roll/pitch factors. Detections are matched to landmarks per class with a
Mahalanobis gate and the Hungarian algorithm, and each match becomes a factor
in the graph.

A detection that matches nothing becomes a candidate. When `confirm_hits`
detections land within `candidate_radius_m` of each other inside
`confirm_window_s`, the candidate becomes a landmark.

Landmarks are never deleted for being out of view. Seeing an old landmark
again after a loop adds factors to the same landmark, which is what corrects
the drift. Two landmarks of the same class that end up on top of each other
are merged.

Classes that can be mistaken for each other, like the red and white slalom
pipes, share a `group`. Detections of any class in the group go to the same
landmark and the landmark takes the class it was seen as most often.

The map frame is the vehicle's pose at startup or at `mission/wipe`: x along
its heading, y right, z down.

### Prior map

The prior map says roughly where each task is. It is not part of the graph.
It does two things: a class with a `prior` only gets new landmarks within
`prior_radius_m` of that task, and each task gets a `prior_<task>` frame to
search from before the object is seen.

## Interfaces

All under the drone namespace.

| Name | Type | Description |
|---|---|---|
| `odom` | sub, `nav_msgs/Odometry` | Vehicle pose |
| `landmarks` | sub, `vortex_msgs/LandmarkArray` | Detections, stamped with the image time. A rotation variance of 1000 or more means position only |
| `mission/wipe` | sub, `std_msgs/Empty` | Start a new map at the current pose |
| `landmark_server/landmarks` | pub, `vortex_msgs/LandmarkTrackArray` | The map, after every keyframe |
| `landmark_server/markers` | pub, `MarkerArray` | 2 sigma sphere and label per landmark |
| `landmark_server/nis` | pub, `Float64` | Mean NIS of the last 50 matches, should be near 1 |
| `landmark_server/set_premap` | srv, `vortex_msgs/SetPremap` | Replace the prior map and save it to `premap_file` |
| `landmark_server/get_premap` | srv, `std_srvs/Trigger` | The prior map as YAML |

## TF frames

All are children of `map`.

| Frame | Description |
|---|---|
| `odom` | Drift correction |
| `start` | Where the run started |
| `prior_<task>` | Where the prior map puts the task |
| `<class>` | The most observed landmark of the class |
| `gate_middle`, `<panel>_entrance`, `<panel>_exit` | Gate, +X through it away from the start |
| `slalom_left_<n>`, `slalom_right_<n>` | Where to pass row n on each side of its red pipe, +X through the row |
| `torpedo_opening_<name>` | Openings on the torpedo board, +X through the board |

A frame moves when the map is corrected, so look it up again before the
final approach. Frames seen from far away can be 0.3 to 0.6 m off and settle
to about 0.1 m up close.

## Running

```bash
ros2 launch landmark_server landmark_server.launch.py
ros2 launch landmark_server landmark_server.launch.py env:=sim
```

| Argument | Default | Description |
|---|---|---|
| `env` | `pool` | `sim` loads `sim.yaml` and `premap_sim.yaml` |
| `config_file` | `config/landmark_server.yaml` | Parameters |
| `premap_file` | `config/premap.yaml` | Prior map, written by `set_premap` |
| `odom_topic`, `landmarks_topic` | from the robot file | Topic overrides |

Start a new map with the vehicle at the start, facing the course:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

## Prior map GUI

```bash
ros2 run landmark_server competition_map_gui.py --ros-args -r __ns:=/nautilus
```

The GUI shows the pool from above. Place the reference where the vehicle
starts and point it at the course, then place the tasks. Send to Vehicle
applies the map at once and saves it. The old file is kept with a timestamp.

The dashed circle around a task is its `prior_radius_m`. The real object has
to be inside it or its detections are rejected. Crosses show the landmarks
the vehicle has found, so after a practice run you can drag the tasks onto
the crosses and send again.

File format:

```yaml
reference_frame: start
created_at: '2026-10-09T12:00:00'
objects:
  torpedo: {position: [17.0, -5.2, 2.5], orientation: [0.0, 0.0, 1.0, 0.0]}
```

## Adding a new object

For a new object the detector publishes:

1. Add the constants to `LandmarkType.msg` and `LandmarkSubtype.msg` in
   vortex-msgs if they are not there.
2. Add the same names to the tables in `src/config.cpp`.
3. Add an entry under `classes` in `config/landmark_server.yaml`:

   ```yaml
   buoy: {type: BUOY, subtype: BUOY_RED, symmetry_deg: 360.0, has_orientation: false, prior: "buoy", prior_radius_m: 3.0, max_instances: 1}
   ```

4. If it has a `prior`, add that label to `SERVICE_LABEL_MAP` in
   `scripts/competition_map_gui.py` so it can be placed in the prior map.

The server then publishes it as the TF frame `buoy`. If the mission can
navigate straight to the object, this is all that is needed.

## Adding a target frame

A target frame is needed when the place to go is not the object itself, like
the gap between two slalom pipes or an opening on the torpedo board.

1. Add a params struct to `config.hpp` and a member in `Params`.
2. Read the parameters in `load_config()` in `landmark_server_node.cpp`.
3. Write a function in `targets.cpp` that takes the landmarks and returns a
   list of `NamedPose`. Use `best_of()` to get a landmark by class name and
   return an empty list while the landmarks it needs are missing.
4. Call it in `publish_map()` next to the existing ones.
5. Add the parameters to `config/landmark_server.yaml`.

`torpedo_frames()` is the shortest example. Use +X as the direction the
vehicle should face or drive, so the mission can give offsets as "0.6 m in
front" without knowing the geometry.

Fixed offsets are easier to tune if they can be changed while running. The
torpedo openings do this in `on_parameters()`:

```bash
ros2 param set /nautilus/landmark_server_node torpedo.openings.large_left "[-0.21, -0.064]"
```

## Tuning

Tune the detection noise first until NIS is near 1, then the odometry noise
on a loop, then the new landmark values. Change one thing at a time.

| Symptom | Change |
|---|---|
| NIS well above 1 | Raise `detection.*` sigmas |
| NIS well below 1 | Lower `detection.*` sigmas |
| Duplicates after a loop | Raise `odom.sigma_*_per_m` |
| Map wobbles between landmarks | Lower `odom.sigma_*_per_m` |
| False detections become landmarks | Raise `confirm_hits`, lower `max_range_m` or `prior_radius_m` |
| Real objects appear late | Lower `confirm_hits` |
| Real object rejected at the edge of its task | Raise `prior_radius_m` or fix the prior map |
| A drifted copy stays next to an object | Raise `upkeep.merge_radius_m` |

## Testing in the simulator

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --headless --detach
ros2 launch landmark_server landmark_server.launch.py env:=sim
ros2 launch robosub_dummy_publisher robosub_dummy_publisher.launch.py profile:=realistic seed:=7
```

`robosub_dummy_publisher` is in vortex-cv. Watch `landmark_server/markers`,
the TF frames and `landmark_server/nis` in Foxglove.
