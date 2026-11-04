# landmark_server

Turns detections into TF frames the mission can drive to, and corrects odometry
drift. It knows objects, not tasks: a new course or competition is config, not
code.

## The layers

Data only goes up. A layer knows nothing about the ones above it.

```
3  Mission     Behavior tree. Find -> Measure -> Act.
               Only sees the TF frames and the measure action below.
---------------------------------------------------------------------------
2  Estimation  Associator      which object is this detection?    <class> frames
               DriftGraph      the detections -> map -> odom      prior, start
               measure action  fresh detections while holding     <class>/measured
---------------------------------------------------------------------------
1  Sensors     Detections in the camera frame with the image stamp, odometry
```

| Code | Layer | Does |
|---|---|---|
| `association` | 2 | Hungarian per image on Mahalanobis distances from the graph's last solve. Objects stay in `odom`, as the median of their newest detections, and never die |
| `graph` | 2 | iSAM2 graph of keyframes and landmarks, fed the detections themselves. Gives `map -> odom`, and the distances the matching needs |
| `measure` | 2 | One object from the detections of a time window. Your task's own measurement goes here |
| `landmark` | 2 | `Hit`, `Landmark`, clustering |
| `premap` | 2 | Reads and writes the prior map |
| `config`, `ros_config` | | Settings and checking them |
| `markers` | | Debug markers |
| `landmark_server_node` | | Wires the above to ROS, no algorithms |

Navigation and drift correction are split. Landmarks and measured frames live in
`odom`, the frame the vehicle drives in, and the graph never moves them. What the
graph corrects is `map`: `start`, `prior_<task>` and `measured_in_map`, so going
home and searching stay right while `odom` drifts.

## Config

| What | Where | Changes |
|---|---|---|
| How the algorithms run | `config/landmark_server.yaml` | Rarely |
| What is on the course | `config/<competition>/classes.yaml` | Per competition |
| Where it is | `config/course/<course>/premap.yaml` | Per pool |
| What the mission does | Tree, in `vortex-cv` | Per competition |

```bash
ros2 launch landmark_server landmark_server.launch.py competition:=robosub course:=pool
ros2 launch landmark_server landmark_server.launch.py competition:=tac course:=tac_pool
ros2 launch landmark_server competition_map_gui.launch.py competition:=tac
```

### A new pool or course

1. `mkdir config/course/<name>` and copy an empty `premap.yaml` from `course/pool`.
2. Build once with `--symlink-install`, start with `course:=<name>`.
3. Make the prior map, see below.

### A new competition

1. Add `config/<competition>/classes.yaml`, see `config/tac/classes.yaml`.
2. New landmark type: see [Adding a landmark](#adding-a-landmark).
3. Write the tree. No change to this package unless it needs a
   new kind of measurement.

## Making the prior map

The prior map is one point per task: where to search, and detections far from it
are dropped. Make it from the vehicle on the test day:

1. Put the vehicle at the marked start, pointing along the course, and press
   **Vehicle faces the course**, then **New map at the vehicle**.
2. Drive to a task and hold still in front of the object.
3. Choose its class under **Measure an object** and press **Measure**. The task
   lands on the map where it was measured.
4. Repeat for each task. Measure again whenever you like, the last one counts.
5. Drag what looks wrong, then **Send to Vehicle**. It is saved with a backup.

If the detector can not see an object, place the task by hand on the map. Later
runs must start from the same marked spot, as the map is relative to the start.

## Using it from the mission

| Frame | Parent | |
|---|---|---|
| `start` | `map` | Where the map started |
| `prior_<task>` | `map` | Where the prior map puts the task |
| `<class>` | `odom` | Where the most observed object of the class was last seen. Until seen: its prior |
| `<class>/measured` | `odom` | The last successful measurement, fixed until measured again |
| `<class>/measured_in_map` | `map` | The same measurement in `map`, so the graph keeps it right. For coming back to it later |
| `<out_frame>/<name>` | `odom` | Frames a measurement of your own adds on the measured one |

Axes: +X forward, +Y right, +Z down. Objects with a front have +X out of the front.
An object without its own yaw takes its prior's heading in `<class>`, and +X from
the vehicle to the object in `<class>/measured`.

Every task is the same three steps, with offsets in the tree:

```xml
<GoToFrame frame="torpedo_board" offset="-3;0;0;0"/>          <!-- find -->
<Measure class_name="torpedo_board" fallback_from="torpedo_board"/>
<GoToFrame frame="torpedo_board/measured" offset="2;0;0;180"/>  <!-- act -->
```

To come back to something measured long ago, go near it on `measured_in_map` and
measure again, with `fallback_from` that frame if the measurement fails. `measured`
is fine right after measuring, but it drifts with `odom`.

Corrections move `prior_<task>`, `start` and `measured_in_map` in `odom`, so targets on those can
jump. `GoToFrame` follows with `resend_m`.

## Topics, services, actions

| Name | Kind | Type |
|---|---|---|
| `odom` | in | `nav_msgs/Odometry` |
| `landmarks` | in | `vortex_msgs/LandmarkArray` |
| `mission/wipe`, `mission/set_heading` | in | `std_msgs/Empty` |
| `landmark_server/landmarks` | out, in `odom` | `vortex_msgs/LandmarkTrackArray` |
| `landmark_server/map_landmarks` | out, in `map` | The graph's estimates, to check it. Not for navigation |
| `landmark_server/measure` | action | `vortex_msgs/MeasureLandmark` |
| `landmark_server/reset` | service | `vortex_msgs/ResetLandmarks` |
| `landmark_server/set_premap`, `get_premap` | service | Prior map, used by the GUI |

## How a detection is used

```
detection
  │  not a class, NaN, bad quaternion ───────────────▶ dropped
  │  farther than max_range_m from the vehicle ──────▶ dropped
  │  farther than the task radius from prior_<task> ─▶ dropped (measure still gets it)
  ▼
in odom, with TF at the image stamp
  ├─▶ matching: one round twice a second, per image and class group
  │     Hungarian on Mahalanobis distances from the graph's last solve
  │     matched ─────────────────────▶ that object
  │     inside a gate but taken, or ambiguous ▶ not used
  │     outside all gates ───────────▶ candidate
  │     confirm_hits in confirm_window_s ▶ new object, the <class> frame
  └─▶ graph: the detections of an object, never a filtered estimate
        averaged per object between keyframes, one bearing/range factor
```

- An object is the median of its last 100 detections, in `odom`. The graph does
  not move it.
- The gate comes from the graph's covariance, so it follows how unsure the map is
  and finds an object again after drift. That is what corrects the map: an old
  landmark seen from a new keyframe.
- A keyframe comes after `keyframe_dist_m`, or after `keyframe_time_s` with
  detections waiting. The time rule looks odd, but a vehicle that holds still to
  measure still gets its landmarks into the graph. Keyframes on motion alone gave
  more duplicates and a worse map home in the simulator, so test the whole course
  if you change this.
- The factor's sigma is not scaled by the number of detections averaged. Their
  errors are correlated, so many of them say little more than a few.
- Matching assumes reasonably good detections. Mostly wrong classes or many false
  positives are not handled here, only a few false ones and outliers.
- Classes of one `group` share objects, and the object takes the class most of its
  detections say. Red and white pipes are one group.
- One object held still can not tell a heading error from a position error. It
  takes a few objects, or moving while one is in view.
- `max_instances` limits what is shown, most observed first.

## Measure

```
goal: class_name, out_frame, duration_s, max_range_m, min_hits, max_spread_m, fallback_from
```

1. Hold the vehicle still near the object.
2. The action takes the detections of the class that arrive during `duration_s`,
   within `max_range_m`. The matching, graph and prior gate are not used.
3. The largest cluster is the object. It fails with fewer than `min_hits` or a
   spread (RMS distance from the median) over `max_spread_m`.
4. On success `out_frame` (default `<class_name>/measured`) is set in `odom`.
5. On failure the old frame stays. With `fallback_from`, that frame is copied to
   `out_frame` and `used_fallback` is true.

0 or "" in the goal uses the default from `measure` in the config.

## A measurement for your task

`Measure` on a class gives one object: the biggest cluster of that class. If your
task needs more than that, say a yaw from two objects, or frames worked out from the
shape of the task, write it yourself:

- `measure.cpp` is where it goes. It gets the detections of the time window and the
  vehicle's pose, and returns a `Measurement`. Keep it free of ROS so you can test it
  with made-up detections, see `test/test_landmark_server.cpp`.
- `Measurement::children` become frames on the measured one, `<out_frame>/<name>`.
- In `landmark_server_node.cpp` look at what happens to the goal's `class_name`:
  `accept_measure` turns down names that are no class, the detection callback decides
  which detections are kept for the measurement, and `tick_measure` calls
  `measure_object`.
- Numbers about the task go in the config, read them in `ros_config.cpp`.
- Detections are noisy and a short look from one spot says little about a heading.
  Look at the numbers in the log and at the frame before you drive on it.

## Reset

```bash
ros2 service call /nautilus/landmark_server/reset vortex_msgs/srv/ResetLandmarks "{class_name: torpedo_board}"
```

Forgets the landmarks of the class until seen again, `""` for all. Measured frames
and the graph stay. `mission/wipe` starts a new map, objects and graph, and drops
the measured frames. It keeps the heading from `mission/set_heading`.

## Debug

```bash
ros2 launch landmark_server landmark_server.launch.py debug:=true
```

`landmark_server/markers` and `landmark_server/raw_detections`. For `map -> odom`,
compare `landmark_server/map_landmarks` with where the objects are.

## Adding a landmark

| Step | File | What |
|---|---|---|
| 1 | `vortex-msgs/msg/LandmarkType.msg`, `LandmarkSubtype.msg` | The constants, if new |
| 2 | `vortex-utils/.../landmark_names.hpp` | The names, so the config can use them |
| 3 | `config/<competition>/classes.yaml` | The class |

The server refuses to start on two classes with one type and subtype, or an unknown
name. Detections of no class are ignored, with one warning.

| Field | What it does |
|---|---|
| `type`, `subtype` | What the detector sends |
| `symmetry_deg` | `0` full yaw, `90` / `180` yaw modulo that, `360` no yaw. Only `0` is tracked in `<class>` |
| `has_orientation` | Use the detector's orientation. It must set the orientation variances below 1000 |
| `prior` | Task in the prior map, `""` for no gate. Several classes can share one |
| `prior_radius_m` | Radius of the gate, unless the GUI set one. Prior error plus the size of the task |
| `max_instances` | Shown, most observed first. `0` for all |
| `group` | Classes the detector can mix up |

## Settings

All in `config/landmark_server.yaml`.

| Value | Raise it when | Lower it when |
|---|---|---|
| `association.confirm_hits` | False objects get confirmed | Real objects appear late |
| `association.gate_prob` | Objects are not found again after drift | Detections match the wrong object |
| `association.ambiguity_d2` | Detections between two objects match the wrong one | Too many detections are thrown away |
| `association.merge_radius_m` | One object becomes two | Two objects become one. Under half their spacing |
| `graph.sigma_*_per_m` | Objects seen again do not pull the map or fall outside the gate | The map jumps on noisy detections |
| `graph.bearing_sigma`, `range_sigma_*` | The map jumps on noisy detections | Objects seen again do not pull the map |
| `graph.dcs_phi` | Real corrections are ignored | Bad detections pull the map |
| `measure.*` | | |
