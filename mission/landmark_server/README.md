# Landmark Server

The map. It takes detections (`LandmarkArray`), tracks them, places the
known course tasks from them and keeps the map aligned with the drifting
odometry. Out comes one landmark per part of each task, with a stable id,
the task's yaw and derived targets such as the torpedo openings.

It does not move the vehicle. The behaviour tree turns map landmarks into
goals with the `landmark_targets` library and sends them to
`waypoint_manager`.

```
detections ─▶ intake gate ─▶ tracker ─▶ course model + retained landmarks ─▶ drift correction ─▶ map rules ─▶ object_map
             (course regions,  (live tracks,  (one landmark per task part,      (iSAM2: keyframes    (course frame,
              focus)            N/M, PDAF)     stable ids, memory)               + landmarks)         bins, octagon)
```

## Running

```bash
ros2 launch landmark_server landmark_server.launch.py                       # env:=sim
ros2 launch landmark_server landmark_server.launch.py env:=pool
ros2 launch landmark_server landmark_server.launch.py env:=pool debug:=true # with the debug topics
```

| Argument | Default | |
|---|---|---|
| `env` | `sim` | `sim` or `pool`: which measured values (`config/<env>.yaml`) and course layout (`config/course/<env>.yaml`) |
| `course` | the one of `env` | Another course layout: a name in `config/course` or an absolute path |
| `debug` | `false` | Debug topics and the map view (`config/debug.yaml`), see [Debugging](#debugging) |
| `calibration` | `false` | Drift calibration session with an ArUco board (`config/calibration.yaml`), see [Measuring a pool](#measuring-a-pool) |
| `drone`, `namespace` | `nautilus` | As for every node in `auv_setup` |

At start the log shows the configuration in use: graph on/off, the floor
depth, the detector noise and the course tasks. Check it says the
environment you meant.

## Interfaces

| Interface | Type | |
|---|---|---|
| `landmarks` | in, `LandmarkArray` | Detections. `header.stamp` is the image time and `frame_id` the camera frame; each message waits for the TF to `target_frame` at its stamp |
| `odom` | in, `Odometry` | Vehicle pose. Its frame must be `target_frame` or have a static TF to it |
| `mission/wipe` | in, `Empty` | Clear everything, the course frame included |
| `landmark_server/object_map` | out, `LandmarkTrackArray` | **The map** (below) |
| `landmark_server/course_state` | out, `CourseState`, latched | Every task: placed, locked, committed, variant, pose, its parts, and the detections dropped at intake by reason |
| `landmark_server/course_frame_state` | out, `CourseFrameState`, latched | `UNSET`, `COARSE` or `GATE_LOCKED` |
| TF `nautilus/course` | out | The course frame: origin at the gate, x through it, y right, z down. Not published while `UNSET` |
| `landmark_server/set_course_frame` | service, `SetCourseFrame` | Start value from the start pose and the coin flip (0, ±π/2 or π) |
| `landmark_server/set_focus` | service, `SetMapFocus` | The tasks the mission works on (empty = all); `lock_others` freezes the rest; `commit`/`uncommit` freeze a task's pose and variant. BT node `SetMapFocus` |
| `landmark_server/clear` | service, `std_srvs/Empty` | Empty the map and the tracker; the course frame stays |
| `landmark_polling` | action, `LandmarkPolling` | Waits until a confirmed track of `type`/`subtype` (0 = any) exists and returns them all |

### Reading the map

Each entry of `object_map` has a stable `landmark.id`, `type`, `subtype`,
`pose` and covariance, plus:

| Field | Meaning |
|---|---|
| `retained` | False while it is seen now; true when it is only remembered |
| `derived` | Computed rather than detected (a torpedo opening, the octagon over the table) |
| `has_orientation` | The yaw is known. Parts of a placed task carry the task's yaw: +X out of the front of the prop, +Z down |
| `last_measurement` | Age = now − this |
| `observations` | Detections behind it |

- **Navigating to a task:** take its parts from `object_map`. Remembered
  parts are kept for the whole run and move with the drift correction, so
  they stay where the object is relative to the vehicle.
- **Aiming (torpedo, dropper):** use the same entries. While `retained` is
  false they follow the live detections, with the class of the part. In the
  simulator, the torpedo icons within 3 m were 4.5 cm off with the class
  100 % right. Use the opening subtype (`TORPEDO_TARGET_*`) for the shot,
  and check `retained == false` and a small age before firing.
- **Not `live_tracks`:** the raw tracker tracks (a debug topic). A
  misclassified detection starts its own track on the right object, so the
  class there is wrong 10–20 % of the time.
- **`landmark_targets`** turns an entry into a waypoint goal: an offset in
  the landmark frame, the tool arm (launcher, dropper), resending as the
  landmark moves, dead reckoning up close, and `LOST` when it is not seen.
  See its README.

## Debugging

Debug output is off by default, so the vehicle only does the work it needs.
Two switches, set by `config/debug.yaml` (`debug:=true`) and changeable while
the server runs:

| Switch | Publishes |
|---|---|
| `debug.enable` | `landmark_server/live_tracks` every tick; the drift correction once a second: `graph/start_frame_path` (smoothed path from the start), `graph/odom_path` (raw odometry), `graph/path` (smoothed, in the current odom frame), `graph/landmarks`, `graph/pose` and `graph/stats` (`[keyframes, landmarks, correction x, y, yaw deg, slowest update ms, detector range error %]`) |
| `debug.markers` | `landmark_server/markers`: the map view (landmarks, the course tasks with their search circles and parts, the course frame and lane). How it is drawn: `config/markers.yaml` |

```bash
ros2 param set /nautilus/landmark_server_node debug.markers true
ros2 param set /nautilus/landmark_server_node debug.enable false
```

Or in Foxglove: `foxglove/landmark_graph.json` (Layout, Import from file)
has an *Innstillinger* tab with the parameters, next to the map, the error
plots, the graph state and the messages.

What else to look at:

- **The log.** Every 10 s the drift correction:
  `graph: 727 keyframes, 31 landmarks, correction (0.95, -1.38) m, 4.4 deg, detector range +2.7 %`.
  A detector range steadily above ±5 % means the detector's distances are
  off: fix the camera calibration, not the server.
- **`course_state`.** Which tasks are placed, and how many detections were
  dropped at intake and why (`outside_tasks`: far from every task's region,
  usually a prior that is off; `too_far`: beyond the template's
  `max_range_m`; `task_locked`; `course_frame_unset`; `outside_lane`).
- **`object_map` against `live_tracks`.** An object in `live_tracks` but not
  in the map is a course or class-rule problem; one missing from both is a
  detector or tracker problem.

## Configuration

The launch file loads these, later files winning:

| File | Contents | Changed |
|---|---|---|
| `config/landmark_server_config.yaml` | Tracker, free classes, floor/surface classes | Rarely: the same in every pool |
| `config/markers.yaml` | How the map view is drawn | Display only |
| `config/<env>.yaml` | Measured in that pool: floor depth, table height, `detector_noise`, `graph.odom_noise`. `pool.yaml` marks each `MEASURE` with its test | Per pool |
| `config/course/templates.yaml` | What each RoboSub prop looks like, and its tolerances | Per competition |
| `config/course/<env>.yaml` | Where the tasks are: `start`, and `{template, prior}` per task | Per pool |
| `config/debug.yaml` | Debug switches | `debug:=true` |
| `config/calibration.yaml` | Drift calibration session | `calibration:=true` |

Everything not in a file has a default in the code ([All settings](#all-settings)).
Keys are checked: a misspelt key in the course stops the server at start
with its name, and a removed key is rejected with what replaced it.

**Changing settings while it runs.** The map rules (`intake`,
`course_frame`, `classes`, `rules`, `markers`) and the debug switches are
ROS parameters. A change is parsed by the same code as at start and
applies at the next tick without losing the map; a bad value is rejected
with the reason. `track_config`, `graph`, `detector_noise` and `course` are
read at start: a new value is rejected with "restart".

```bash
N=/nautilus/landmark_server_node
ros2 param set $N rules.z_lock.floor_z 3.6
ros2 param load $N src/vortex-auv/mission/landmark_server/config/pool.yaml
ros2 param dump $N > tuned.yaml
```

## Course model

The course layout is known before a run: which tasks there are, what each
looks like and roughly where it is. The map does not discover objects from
scratch; it places the known tasks and fills in their parts.

- **Templates** (`course/templates.yaml`): a prop as parts (classes at
  offsets in the task frame: +X out of the front, +Y right, +Z down), each
  with a `sigma` for how well the prop and detector follow the drawing. A
  part can allow several classes (bin roles, octagon images: the votes
  decide), and a template can have variants that differ only in classes
  (the torpedo decal versions). `points` are derived targets: `{class,
  offset, from, yaw_deg}` relative to a part or the task origin. A new
  approach point is a config entry, not code.
- **Tolerances**, per template, a task can override: `region_radius_m`
  [2.5] (how far off the prior may be), `part_radius_m` [1.0],
  `yaw_window_deg` [30], `symmetric` [false], `min_parts` [2] (1: one part
  places it at the prior yaw), `max_range_m` [any].
- **Tasks** (`course/<env>.yaml`): `{template, prior: [x, y, yaw_deg]}` in
  the course frame (origin at the gate, x through it), and `start`, where
  the vehicle starts in that frame.
- **Class groups**: classes a detector mixes up (white/red pipe, fire/blood)
  are tracked as one kind. A part's class comes from the template or the
  votes, never from one detection.
- **Intake**: a detection of a task class far from every task that has it
  (outside the region of a task not yet placed, beyond `part_radius_m` of a
  placed one's part, or only in locked tasks) is dropped and counted in
  `course_state`. Without a course frame no task class is taken. Classes in
  no template go on as free landmarks under `classes`.
- **Placing**: the template is fitted to the confirmed tracks in its region
  (yaw within the window, at least `min_parts`, classes agreeing). The
  layout is then turned and moved to fit the tasks found, so the next task
  is searched where it really is.
- **Parts**: one landmark per part, stable id, never forgotten, never more.
  A track that fits no part is not an object.
- **Focus** (`set_focus`): with `lock_others`, tasks outside the focus are
  frozen and their detections dropped. `commit` freezes a task's pose and
  variant.

`course.enable: false` (no layout yet, as in `pool.yaml` until measured):
every class is a free landmark, with no gate yaw and no openings. That is
a detector test, not a mission.

**A new or moved task:** edit `tasks` in `course/<env>.yaml`. **A new
arrangement of known classes:** a new template. **A new kind of object:** a
constant in `vortex_msgs` (`LandmarkType`/`LandmarkSubtype`) and a rebuild;
the class names are generated from the messages, so the config knows it by
name.

## Drift correction

Odometry drifts in x, y and yaw, so a landmark remembered from 10 m ago is
off by the drift since. `LandmarkGraph` (GTSAM iSAM2) keeps vehicle
keyframes and landmark positions in one factor graph:

| Factor | Noise |
|---|---|
| Odometry between keyframes | `graph.odom_noise`: per metre, per √s (hover) and per second (gyro bias) |
| Roll, pitch and depth per keyframe (they do not drift) | `graph.absolute` |
| Keyframe → landmark, for each detection the tracker associated | `detector_noise`, Huber-robust |
| One detector range scale k for the run: every detection is (1 + k) × the true vector | prior 0 ± `graph.measurements.range_scale_std` (5 %) |

The map gets the smoothed positions **in the current odom frame**: where
the landmark is relative to the vehicle according to the graph, expressed
with the vehicle's raw odometry pose. The controller keeps steering on
odometry; the map moves to stay right. Measurements go in under the map
id, so a remembered landmark seen again closes the loop.

**The odometry noise must match the real drift.** Too large and the graph
bends the path after detection noise; too small and it cannot follow the
drift. Map error in the simulator (two-lap course, two seeds each):

| `odom_noise` | No drift | Realistic drift | Worst drift |
|---|---|---|---|
| graph off | 0.055 m | 0.168 m | 0.350 m |
| ×1 (`sim.yaml`) | 0.094 m | **0.064 m** | **0.090 m** |
| ×½ | 0.075 m | 0.092 m | 0.133 m |
| ×¼ | **0.049 m** | 0.103 m | 0.173 m |

When in doubt, round up. Measure it: [Measuring a pool](#measuring-a-pool).

## Results in the simulator

Realistic detector (noise growing with range, 2 % range bias, misclassified
detections, clutter, phantoms) and realistic odometry drift:

| | |
|---|---|
| Task parts (two-lap course) | 0.07–0.11 m |
| Gate at the end of the run (the way home) | 0.06–0.10 m |
| Torpedo icons within 3 m, from `object_map` | 4.5 cm, class 100 % right, one id each |
| Large drift (≈4° yaw over the run) | path 0.06 m (0.16 m with the range scale k off) |
| Detector at 3 Hz, 0.3 s late | map 0.10 m at the end, icons 6.8 cm |
| Detector at 1.5 Hz, 0.5 s late | map 0.18 m at the end, icons 7.3 cm; a slalom set can be missed |

All from the simulator and the dummy detector. Real detections fail in
other ways: replay a real recording before trusting these numbers.

## Measuring a pool

Two things are measured once per pool and vehicle: how much the odometry
drifts (`graph.odom_noise`) and how far off the detections are
(`detector_noise`). Neither needs an absolute position: a board that does
not move is the reference. With the map on the raw odometry, every time the
board is seen again, its apparent move is the drift since the first time.

**Set-up**

- An ArUco board (TAC board: four 15 cm markers, ids 28, 7, 96, 19,
  0.43 × 0.83 m; another board: its sizes in `aruco_detector_params.yaml`),
  fixed upright at the vehicle's depth with 15–20 m of open water in front.
  It must not move or sway. Mark a spot 3 m in front of it.
- Camera intrinsics calibrated, `aruco.marker_size` measured on the print.

```bash
ros2 launch aruco_detector aruco_detector.launch.py          # pubs.landmarks: /nautilus/landmarks
ros2 launch landmark_server landmark_server.launch.py env:=pool calibration:=true
ros2 run landmark_server aruco_drift.py --ros-args -r __ns:=/nautilus -p csv:=$HOME/bags/aruco_visits.csv
src/vortex-auv/utility_scripts/record_landmark_bag.sh aruco-calibration
```

`calibration:=true` runs with no graph and no course, and keeps the board
for the whole session. `aruco_drift.py` splits the session into visits (2 s
out of view ends one) and prints, after each, the jump against the first
visit and the `pool.yaml` values so far.

**Check first:** hold still at the 3 m spot. The visit line's `range` must
read the taped distance within 3 cm, or the marker size or camera
calibration is wrong.

**The session (about 30 minutes).** Every return is to the 3 m spot, facing
the board: stop, hold 10 s with it in view, then turn away.

| Test | What to do | Gives |
|---|---|---|
| A, loops | Out and back 5, 10, 20 and 30 m with the turns of a real run; one loop each way | `pos_std_per_m`, `yaw_std_deg_per_m` |
| B, hover | Hold still at 3 m for 90 s at the start and 2–3 min at the end | `min_pos_std_m`, `yaw_std_deg_per_sec` |
| C, ranges | Hold still 60 s each at 2, 4 and 6 m | `detector_noise` |
| D, range check | The taped check at 2 and 6 m too | Range bias: fix the calibration if over ~3 % |

At the end (Ctrl-C) the script prints the `pool.yaml` lines (about twice
the measured drift: the drift is a bias, the graph's steps are
independent). **Check the result** by replaying the recording with the
graph on: the board should hardly jump on a return.

```bash
C=install/landmark_server/share/landmark_server/config
src/vortex-auv/utility_scripts/replay_landmark_bag.sh ~/bags/<date>_aruco-calibration --env pool \
  raw "--params-file $C/calibration.yaml" \
  graph "--params-file $C/calibration.yaml -p graph.enable:=true"
```

Pitfalls: a board that moves, returns at a steep angle (over ~45° off the
board's normal), the board in view while driving, an odometry reset during
the session.

Also measure the course (tape the props from the gate, or the competition
map: `course/pool.yaml` priors and `start`, then `enable: true`) and the
floor depth (`rules.z_lock.floor_z`, then `enable: true`).

## Tuning

Most numbers are measured, not tuned. In order, one change at a time:

1. **Measure** the course priors, the floor, the detector noise and the
   drift. A wrong measurement looks like a tuning problem and no knob
   fixes it.
2. **Record** a run (`record_landmark_bag.sh`), so every change is tried on
   the same data.
3. **Read** what happened: `course_state`, the map view, `object_map`
   against `live_tracks`, the log.
4. **Change** the setting for the symptom ([All settings](#all-settings)).
5. **Replay** with the old and new value side by side
   (`replay_landmark_bag.sh <bag> base "" trial "-p <key>:=<value>"`); keep
   what is better.

### All settings

| Setting | Default | When to touch it |
|---|---|---|
| `track_config.default.nm` | confirm 3 of 5, delete 5 of 7 | Clutter gets confirmed (stricter), real objects take too long (looser) |
| `track_config.default.gate.max_pos_error` | 1.5 m | One object splits into several tracks (larger) |
| `track_config.default.dyn_mod_std_dev`, `sens_mod_std_dev` | 0.2, 0.2 | Tracks too jumpy (smaller sens) or too slow (larger dyn) |
| `track_config.<CLASS>.*` | the default | A class needs its own (see `SLALOM_PIPE`) |
| `detector_noise` | 0.06 m + 0.03/m along, 0.003/m across | Measured (test C) |
| `intake.measurement_covariance` | off | The detector gives a trustworthy covariance |
| `graph.enable` | on | Comparing with and without drift correction |
| `graph.odom_noise.*` | `<env>.yaml` | Measured (tests A, B) |
| `graph.measurements.range_scale_std` | 0.05 | 0 turns k off; leave it |
| `graph.keyframe`, `absolute`, `measurements` | 0.5 m / 10° / 5 s; 1°, 0.05 m; Huber 2 | Hardly ever |
| Template and task tolerances | see [Course model](#course-model) | A task is not placed, or takes a neighbour's parts |
| `course.*` globals | `variant_votes` 40, `lane_margin_m` 2, `max_align_deg` 20, `min_part_detections` 8 | A variant decided too early or late; a pool larger than the layout |
| `classes.<CLASS>` | 20 instances, 15 s, `instance_gate_m` 0.5 | Free classes only |
| `rules.z_lock`, `rules.table_octagon` | off; `primary: table`, 0.7 m | Measured (floor, table height) |
| `course_frame.gate_lock` | 10 estimates within 3° | Hardly ever |
| `debug.*` | off | See [Debugging](#debugging) |
| `timer_rate_ms`, `topics.*` | 200 ms, the usual names | Never for tuning |

## Simulator

The simulator's odometry does not drift and its detector is a dummy
(`robosub_dummy_publisher`). Two scripts start everything, from the
workspace root:

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --detach
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --profile realistic --drift-profile realistic --detach
ros2 run landmark_server drift_route.py --ros-args -r __ns:=/nautilus -p route:=robosub
```

`--drift-profile` (`stim300`, `realistic`, `degraded`, `worst`;
`config/drift/`) makes the odometry drift like an IMU + DVL estimator and
adds a second server without the graph (`/nautilus_raw`) for comparison.
The servers there run with `debug.yaml`. Don't build or run other heavy
jobs while the simulator runs: a starved controller makes the vehicle
oscillate.

| Script | |
|---|---|
| `drift_route.py` | Drives a route with `waypoint_manager`: `robosub` (gate on the Search & Rescue side, slalom, torpedo, bins, octagon), `short`, `long` (two laps), `slalom` |
| `drift_injector.py` | True odometry and detections → drifted ones (`/nautilus/odom_drift`), the drift on `/nautilus/drift` |
| `graph_eval.py` | The maps against the truth: errors per second, csv, and for Foxglove the truth (green dots), a line from each landmark to its truth, the true path (white; the graph's path, green, should lie on it) |
| `aruco_drift.py` | Drift calibration with an ArUco board ([Measuring a pool](#measuring-a-pool)) |

**Offline:** `record_landmark_bag.sh <test>` records the inputs, the raw
navigation sensors and what the live stack did into `~/bags/<date>_<test>`.
`replay_landmark_bag.sh <bag> --env pool base "" trial "-p ..."` plays the
inputs into one server per label (`/tune_<label>/...`, with debug on). Keep
the rate at 1.0: the servers tick on wall time.

## Files

| | |
|---|---|
| `src/landmark_server_ros.cpp` | The node: inputs, the tick, reset |
| `src/landmark_server_config.cpp` | Config files to settings, live parameter changes |
| `src/landmark_server_publish.cpp` | `object_map`, course frame and TF, services |
| `src/landmark_server_course.cpp` | Intake gate, tracker limits, `set_focus`, `course_state` |
| `src/landmark_server_graph.cpp` | Odometry and detections into the graph, smoothed positions into the map |
| `src/landmark_server_polling.cpp` | `LandmarkPolling` |
| `src/landmark_server_debug.cpp`, `_markers.cpp` | Debug topics, the map view |
| `src/class_config.cpp`, `course_model.cpp`, `structures.cpp`, `retained_landmarks.cpp`, `course_frame.cpp`, `map_rules.cpp` | ROS-free map logic |
| `src/landmark_graph.cpp` | ROS-free iSAM2 backend; the only file with GTSAM |

Tests (gtest, ROS-free): `colcon test --packages-select landmark_server`.
`test_config_files` parses the shipped config files, so a broken file
fails the tests.
