# Landmark Server

The **Landmark Server** is the map. It receives detections (`LandmarkArray`), tracks them, remembers them with stable ids and derives structure from the parts (gate yaw from the panels, torpedo openings from the icons, bin roles, the octagon over the table). It does **not** move the vehicle: the behavior tree computes targets from the map with the `landmark_targets` library and sends them to `waypoint_manager`.

```
perception (position without orientation) ─ LandmarkArray ─▶ intake
   ─▶ PoseTrackManager (live tracks, PDAF, N/M)
   ─▶ RetainedLandmarks (stable ids, memory, class rules)
   ─▶ map rules (yaw, openings, roles, octagon) ─▶ object_map
course frame: set_course_frame ─▶ TF nautilus/course + course_frame_state
```

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `landmarks` | topic in (`LandmarkArray`) | Detections. `header.stamp` = image time, `frame_id` = camera frame (TF to `target_frame` at that time) |
| `odom` | topic in (`Odometry`) | Vehicle position (pipe distance limit, which side of the gate is the front) |
| `landmark_server/object_map` | topic out (`LandmarkTrackArray`) | The map: stable ids, `retained`, `has_orientation`, `derived`, `first_seen`, `last_measurement`, `observations` |
| `landmark_server/markers` | topic out (`visualization_msgs/MarkerArray`) | The map for Foxglove/RViz: a cube per landmark (faded if only remembered), its name with id, age and `derived` when a rule made it, an arrow along +X when the yaw is known, and the course direction and lane bounds in force. Classes in `markers.boxes` get their real size: PVC pipes (gate posts, slalom pipes) as solid pipes instead of the cube, large structures (gate, torpedo board, bin rig, table, octagon) as a see-through box around it. Colour per type |
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
| Stable ids | A new track within `instance_gate_m` of a remembered landmark of the same class takes over its id; classes with `max_instances: 1` accept `plausibility_radius_m`. A false gate 8 m away does not take over |
| Memory | `retain: forever` (gate, board, table, octagon) or `retain_sec`; pipes with `keep_after_observations` observations are kept for the rest of the run |
| Limits | `max_instances` per (type, subtype); no pipes within `min_distance_to_large_structures_m` of a gate/table/board/bin structure; pipes farther than `max_pipe_distance_m` are discarded at intake |
| Gate | Yaw from the panel line, gate pulled to the panel midpoint, synthetic `GATE_WHOLE` if only the panels were seen, panels inherit the yaw |
| Torpedo board | Yaw and centre from the icon pairs (the normals of both pairs are added), version from the icon heights (fire above blood = 1), `TORPEDO_TARGET_*` from icon + `torpedo_targets_from_icons` offsets (board frame; placeholder values, to be measured on our board) |
| Bins | The role icon seen by the down camera gives the role of the nearest bin; the roleless duplicate is hidden |
| Octagon | `OCTAGON_WHOLE` over the table; with `z_lock` on it floats at `surface_z` |
| Depth lock | `rules.z_lock`: floor classes get `floor_z`, surface classes `surface_z` (odom z, down positive). Entries can be a type (`OCTAGON`) or one subtype (`OCTAGON_WHOLE`). The table is not locked: its top is ~0.7 m above the floor. Off by default in code, on in the config with the simulator's pool depth: measure the real one |
| Distance noise | `intake.distance_noise`: the tracker adds `base + per_meter * distance` to the position variance along the line of sight (depth) and `lateral_ratio` times that across it, so far detections weigh less and the depth, which a camera knows worst, weighs least. The covariance from perception is not used for the position |
| Association | One tracker update per camera frame (same stamp), in time order; hits and misses are counted once per tick. Per class, global nearest neighbour: squared Mahalanobis distance as the cost, the gate (`gate.max_pos_error`, `mahalanobis_gate_threshold`) as the limit, the Hungarian algorithm for the one-to-one assignment. Each track is then updated by PDAF with its own measurement |

## Files

- ROS-free (gtest): `class_config`, `retained_landmarks`, `course_frame`, `map_rules`
- ROS: `landmark_server_ros.cpp` (intake, tick, polling, reset), `landmark_server_publish.cpp` (map, live tracks, course frame, services)

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
