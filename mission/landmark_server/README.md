# Landmark Server

The **Landmark Server** is the map. It receives detections (`LandmarkArray`), tracks them, remembers them with stable ids and derives structure from the parts (gate yaw from the panels, torpedo openings from the icons, bin roles, the octagon over the table). It does **not** move the vehicle: the behavior tree computes targets from the map with the `landmark_targets` library and sends them to `waypoint_manager`.

```
perception (position without orientation) ─ LandmarkArray ─▶ intake
   ─▶ PoseTrackManager (live tracks, PDAF, N/M)
   ─▶ RetainedLandmarks (stable ids, memory, class rules)
   ─▶ LandmarkGraph (iSAM2: keyframes + landmarks, corrects odometry drift)
   ─▶ map rules (yaw, openings, roles, octagon) ─▶ object_map
course frame: set_course_frame ─▶ TF nautilus/course + course_frame_state
```

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `landmarks` | topic in (`LandmarkArray`) | Detections. `header.stamp` = image time, `frame_id` = camera frame (TF to `target_frame` at that time) |
| `odom` | topic in (`Odometry`) | Vehicle pose: pipe distance limit, which side of the gate is the front, and the keyframes of the graph. Its frame must be `target_frame` or have a static TF to it |
| `landmark_server/object_map` | topic out (`LandmarkTrackArray`) | The map: stable ids, `retained`, `has_orientation`, `derived`, `first_seen`, `last_measurement`, `observations` |
| `landmark_server/markers` | topic out (`visualization_msgs/MarkerArray`) | The map for Foxglove/RViz: a cube per landmark (faded if only remembered), its name with id, age and `derived` when a rule made it, an arrow along +X when the yaw is known, and the course direction and lane bounds in force. Classes in `markers.boxes` get their real size: PVC pipes (gate posts, slalom pipes) as solid pipes instead of the cube, large structures (gate, torpedo board, bin rig, table, octagon) as a see-through box around it (the whole gate gets no cube: only its two poster plates do). Colour per type, except the torpedo board parts: openings (`TORPEDO_TARGET_*`) as yellow spheres (large or small), icons as flat magenta squares on the board face |
| `landmark_server/live_tracks` | topic out (`LandmarkTrackArray`) | The live tracks of the tracker (always on) |
| `landmark_server/course_frame_state` | topic out (`CourseFrameState`, latched) | `UNSET` / `COARSE` / `GATE_LOCKED` |
| TF `nautilus/course` | TF (child of `target_frame`) | Course frame: x through the gate, y to the right, z down. Not published while `UNSET` |
| `landmark_server/set_course_frame` | service (`SetCourseFrame`) | Start value from the start pose and the coin flip (0, ±π/2 or π). Rejects NaN and illegal angles |
| `landmark_server/clear` | service (`std_srvs/Empty`) | Empty the map and the live tracks |
| `mission/wipe` | topic in (`Empty`) | Clear everything, including the course frame |
| `LandmarkPolling` | action server | Wait for a confirmed live track of a type/subtype (unchanged) |

## Conventions

- **Landmark frame**: origin in the object, +X out of the front, +Z down (NED). For the gate the front is the side the vehicle first saw it from (toward the start); the yaw is then locked after `yaw_lock.consistent_estimates` consistent estimates and never flips.
- **No orientation**: a rotation variance >= `intake.no_orientation_rot_variance` (1000) means position only. The tracker then leaves the orientation alone and `has_orientation` stays false.
- **Course frame**: odom X is the heading the ESKF started with, not the course direction. Nothing here uses fixed odom coordinates; the lane limits are boxes in the course frame (generous before the gate is locked, tight after). When the gate yaw is consistent for 10 estimates the frame moves to the gate; more than `warn_start_vs_gate_deg` off the start value gives a warning and the gate wins.

## Map rules (config `rules`, `classes`)

| Rule | What it does |
|---|---|
| Stable ids | A new track within `instance_gate_m` of a remembered landmark of the same class takes over its id; classes with `max_instances: 1` accept `plausibility_radius_m`. A false gate 8 m away does not take over. Only a clear nearest takes over (`rules.adoption`): the next remembered landmark of the class must be `ambiguity_ratio` (2) times farther away; otherwise the track waits up to `wait_sec` (2 s) for a closer look and then counts as a new object. A wrong take-over would join two objects in the graph for the rest of the run |
| Memory | `retain: forever` (gate, board, table, octagon) or `retain_sec`; pipes with `keep_after_observations` observations are kept for the rest of the run |
| Limits | `max_instances` per (type, subtype); no pipes within `min_distance_to_large_structures_m` of a gate/table/board/bin structure; pipes farther than `max_pipe_distance_m` are discarded at intake |
| Gate | Yaw from the panel line, gate pulled to the panel midpoint, synthetic `GATE_WHOLE` if only the panels were seen, panels inherit the yaw |
| Torpedo board | Yaw and centre from the icon pairs (the normals of both pairs are added), version from the icon heights (fire above blood = 1), `TORPEDO_TARGET_*` from icon + `torpedo_targets_from_icons` offsets (board frame; placeholder values, to be measured on our board) |
| Bins | The role icon seen by the down camera gives the role of the nearest bin; the roleless duplicate is hidden |
| Octagon | `OCTAGON_WHOLE` over the table; with `z_lock` on it floats at `surface_z` |
| Depth lock | `rules.z_lock`: floor classes get `floor_z`, surface classes `surface_z` (odom z, down positive). Entries can be a type (`OCTAGON`) or one subtype (`OCTAGON_WHOLE`). The table is not locked: its top is ~0.7 m above the floor. Off by default in code, on in the config with the simulator's pool depth: measure the real one |
| Detector covariance | `intake.measurement_covariance.use`: the position covariance of the detection (rotated into `target_frame`) replaces the class noise and the distance noise, in the tracker and the graph. `scale` multiplies it (testing), `min_std_m` is a floor. Off by default |
| Distance noise | `intake.distance_noise`: the tracker adds `base + per_meter * distance` to the position variance along the line of sight (depth) and `lateral_ratio` times that across it, so far detections weigh less and the depth, which a camera knows worst, weighs least. The covariance from perception is not used for the position |
| Association | One tracker update per camera frame (same stamp), in time order; hits and misses are counted once per tick. Per class, global nearest neighbour: squared Mahalanobis distance as the cost, the gate (`gate.max_pos_error`, `mahalanobis_gate_threshold`) as the limit, the Hungarian algorithm for the one-to-one assignment. Each track is then updated by PDAF with its own measurement |

## Smoothing backend (config `graph`)

Odometry drifts in x, y and yaw; a landmark remembered from 10 m ago is then
off by the drift since. `LandmarkGraph` (GTSAM iSAM2) keeps a factor graph of
vehicle keyframes (every `keyframe.distance_m` / `angle_deg` /
`interval_sec`) and landmark positions:

| Factor | From | Noise |
|---|---|---|
| Between keyframes | Odometry | `odom_noise`: std of one step, `pos_std_per_m` and `yaw_std_deg_per_m` times the distance (+ `yaw_std_deg_per_sec`). A drift that is a bias needs a larger value than the drift per metre |
| Roll, pitch, depth per keyframe | Odometry (IMU, pressure: no drift) | `absolute` |
| Keyframe → landmark position | Each measurement the tracker associated, relative to the nearest keyframe | The tracker's (class sensor noise + line-of-sight noise), Huber `measurements.huber_k` |

- Measurements go to the graph under the **map id**, not the track id. A track that takes over a remembered landmark (adoption) adds to the same graph landmark: that closes the loop and moves the keyframes and every landmark they saw.
- Measurements of a track that is not in the map yet wait (`max_pending_per_track`) and are added when it is; at most `max_per_keyframe` per landmark and keyframe.
- Output: once a landmark has `min_observations` in the graph, its map position is the graph's, **in the current odom frame**: where it is relative to the vehicle according to the graph, expressed with the vehicle's raw odometry pose. The controller keeps steering on odometry. Orientation, derived landmarks and the depth lock work as before, on top.
- Not affected: `live_tracks` and `LandmarkPolling` (tracker), the course frame TF (follows the gate as the map gives it).
- A log line every 10 s gives keyframes, landmarks and the correction (odom ← graph).
- The noise values are guesses until the drift has been measured in the pool. The association itself does not get better: a remembered landmark must still be within the adoption radius when it is seen again.

### Trying it in the simulator

One command starts everything (tmux session `sim_autonomy`):

```bash
utility_scripts/launch_sim_autonomy.sh --headless --fov        # baseline, no drift
utility_scripts/launch_sim_autonomy.sh --headless --drift 0.5  # drift + a server without graph
ros2 run landmark_server drift_route.py                        # then drive a loop
```

Foxglove layout: `foxglove/landmark_graph.json` (Layout, Import from file). It shows the map, the truth from the course layout (green spheres), a line from each map landmark to its truth (blue with graph, red without), the raw odometry path (`landmark_server/graph/odom_path`, orange) and the graph's corrected path (`landmark_server/graph/path`, green), and plots of the map error, drift against correction and `landmark_server/graph/stats` (`[keyframes, landmarks, correction x, y, yaw deg, slowest update ms]`).

The tools below are what the script starts:

The simulator's odometry does not drift. `scripts/` has tools that add drift
and compare the map with and without the graph (installed as
`ros2 run landmark_server <script>`):

- `drift_injector.py`: true odometry -> `/nautilus/odom_drift` (extra yaw per metre, `drift_yaw_deg_per_m`), true detections (`/nautilus/landmarks_true`) -> `/nautilus/landmarks_drift` in the drifted frame, the drift on `/nautilus/drift`.
- `graph_eval.py`: error of two object_maps (`/nautilus/...` with the graph, `/nautilus_raw/...` without) against the truth in the drifted frame, per second and to csv.
- `drift_route.py`: a loop with waypoint_manager: beside the gate and slalom along y = -3 to the torpedo board, back, and in front of the gate again.

Headless sim (`simulation.launch.py rendering:=false scenario:=nautilus_no_gpu` + `drone_sim.launch.py`), `dp_quat.launch.py`, `waypoint_manager`, the dummy publisher with `-p seed:=7 -p topic:=landmarks_true -p use_field_of_view:=true`, the injector, and two landmark servers with `-p topics.landmarks:=/nautilus/landmarks_drift -p topics.odom:=/nautilus/odom_drift`, one in namespace `/nautilus_raw` with `-p graph.enable:=false`.

Result 2026-09-25 (0.5 deg/m, 32 m, 15.8 deg drift at the end): remembered landmarks 0.12 m mean / 0.19 m max error with the graph, 1.37 / 1.97 m without (torpedo board 0.10 vs 1.62 m, table 0.12 vs 1.97 m).

`drift_injector.py -p noise:=true` adds camera noise (depth std 0.05 + 0.03 d along the line of sight, lateral 0.02 + 0.005 d) and writes its covariance into the detections; `bias_frac_std` adds a constant range bias per landmark that the covariance does not contain. Mean error of the remembered landmarks after the loop, 90 s after the route (six servers on the same data):

| Server | 0.5 deg/m, noise | + 5 % range bias | 1 deg/m, noise |
|---|---|---|---|
| no graph | 1.36 m | 1.38 m | 2.74 m |
| no graph, detector covariance | 1.41 m | 1.38 m | 2.75 m |
| graph, own noise model | 0.16 m | 0.90 m | 0.14 m |
| graph, detector covariance | **0.08 m** | **0.38 m** | 0.17 m |
| graph, covariance x0.1 (overconfident) | 0.50 m | 0.60 m | 0.21 m |
| graph, covariance x10 (underconfident) | 0.13 m | 0.38 m | 0.25 m |

Two laps (`drift_route.py -p route:=long`, 63 m, 31.5 deg drift at the end) with camera noise and the unstable dummy profile (misses, occlusions, outliers, false detections), four servers, truth from the course layout (`graph_eval.py -p truth_seed:=7`). After the second lap:

| Server | Remembered, mean/max | Landmarks (27 real) | Id swaps |
|---|---|---|---|
| no graph | 2.21 / 4.67 m | 34: 2 extra white pipes, 1 red, 1 gate post; the stale torpedo board (3.8 m off) blocks the real one (class full) | 0 |
| graph | 0.06 / 0.16 m | 31: every pipe once | 0 |

The take-over check (`rules.adoption`) changed nothing here, with or without the graph: with the graph the right landmark is clearly the nearest. It stays on as a safety net for dense objects before the first loop closure.

A correct detector covariance helps, most of all its shape (range much less certain than bearing), which keeps a range bias from pulling the map. An overconfident one is worse than the own model and gave a duplicate slalom pipe (the tracker gate became too tight). Covariance without the graph does nothing against drift.

## Configuration per environment

`config/landmark_server_config.yaml` holds what is the same everywhere. What differs between the simulator and a real pool is in a second file, loaded after it (its values win):

```bash
ros2 launch landmark_server landmark_server.launch.py env:=sim    # default
ros2 launch landmark_server landmark_server.launch.py env:=pool
```

| File | Contents |
|---|---|
| `config/sim.yaml` | The simulator's pool floor (`z_lock` on, 3.432 m), torpedo board offsets from its textures, lane limits. Noise values: the common ones (do not tune them in the simulator) |
| `config/pool.yaml` | The tuning sheet for a real pool: every value that must be measured, marked `MEASURE`, with the test it comes from. `z_lock` is off until the floor depth is measured |

Tests that start the node themselves load `landmark_server_config.yaml` and then `sim.yaml`.

## Files

- ROS-free (gtest): `class_config`, `retained_landmarks`, `course_frame`, `map_rules`, `landmark_graph` (own library, the only one that includes GTSAM)
- ROS: `landmark_server_ros.cpp` (intake, tick, polling, reset), `landmark_server_publish.cpp` (map, live tracks, course frame, services), `landmark_server_graph.cpp` (odometry, measurements to the graph, smoothed positions into the map)
- Launch test `test_graph_drift.py`: drifting odometry, loop closure on a table, the far landmark corrected

```bash
ros2 topic echo /nautilus/landmark_server/object_map
ros2 service call /nautilus/landmark_server/set_course_frame vortex_msgs/srv/SetCourseFrame \
  "{start_pose: {orientation: {w: 1.0}}, heading_offset_rad: 0.0}"
ros2 service call /nautilus/landmark_server/clear std_srvs/srv/Empty
```

## Polling

`LandmarkPolling` waits until a confirmed track matching `type` and `subtype` (0 = any subtype) exists and returns all of them.

```bash
ros2 action send_goal /orca/landmark_polling vortex_msgs/action/LandmarkPolling "{
  type: {value: 6},
  subtype: {value: 0}
}"
```
