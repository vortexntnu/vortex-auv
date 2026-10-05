# Landmark Server

The **Landmark Server** is the map. It receives detections (`LandmarkArray`), tracks them, and places the known course tasks from them: one landmark per part of each task, with a stable id, the task's yaw and derived targets (the torpedo openings). It does **not** move the vehicle: the behavior tree computes targets from the map with the `landmark_targets` library and sends them to `waypoint_manager`.

```
perception (position without orientation) ─ LandmarkArray ─▶ intake
   ─▶ intake gate (lane, course tasks, focus; confusable classes -> one kind)
   ─▶ PoseTrackManager (live tracks, PDAF, N/M, tracks per class limited)
   ─▶ CourseModel (one landmark per part of each task) + RetainedLandmarks
      (stable ids, memory, class rules for the other classes)
   ─▶ LandmarkGraph (iSAM2: keyframes + landmarks, corrects odometry drift)
   ─▶ map rules (course frame from the gate panels, bin roles, octagon) ─▶ object_map
course frame: set_course_frame ─▶ TF nautilus/course + course_frame_state
```

## Interfaces

| Interface | Type | Purpose |
|---|---|---|
| `landmarks` | topic in (`LandmarkArray`) | Detections. `header.stamp` = image time, `frame_id` = camera frame (TF to `target_frame` at that time) |
| `odom` | topic in (`Odometry`) | Vehicle pose: the detection range (noise, `max_range_m`), which side of the gate is the front, and the keyframes of the graph. Its frame must be `target_frame` or have a static TF to it |
| `landmark_server/object_map` | topic out (`LandmarkTrackArray`) | The map: stable ids, `retained`, `has_orientation`, `derived`, `first_seen`, `last_measurement`, `observations` |
| `landmark_server/markers` | topic out (`visualization_msgs/MarkerArray`) | The map for Foxglove/RViz: each course task (placed: lines to its parts, parts never seen as spheres; not placed: its search circle; name, parts seen, variant, locked/committed), a cube per landmark (faded if only remembered), its name with id, age and `derived` when a rule made it, an arrow along +X when the yaw is known, and the course direction and the lane. Classes in `markers.boxes` get their real size: PVC pipes (gate posts, slalom pipes) as solid pipes instead of the cube, large structures (gate, torpedo board, bin rig, table, octagon) as a see-through box around it (the whole gate gets no cube: only its two poster plates do). Colour per type, except the torpedo board parts: openings (`TORPEDO_TARGET_*`) as yellow spheres (large or small), icons as flat magenta squares on the board face |
| `landmark_server/live_tracks` | topic out (`LandmarkTrackArray`) | The live tracks of the tracker (always on) |
| `landmark_server/course_frame_state` | topic out (`CourseFrameState`, latched) | `UNSET` / `COARSE` / `GATE_LOCKED` |
| TF `nautilus/course` | TF (child of `target_frame`) | Course frame: x through the gate, y to the right, z down. Not published while `UNSET` |
| `landmark_server/set_course_frame` | service (`SetCourseFrame`) | Start value from the start pose and the coin flip (0, ±π/2 or π). Rejects NaN and illegal angles |
| `landmark_server/clear` | service (`std_srvs/Empty`) | Empty the map and the live tracks |
| `landmark_server/set_focus` | service (`SetMapFocus`) | The tasks the mission works on (empty = all), `lock_others` freezes the rest, `commit`/`uncommit` freeze a task's pose and variant. BT node `SetMapFocus` (vortex_bt_nodes) |
| `landmark_server/course_state` | topic out (`CourseState`, latched) | Every task: placed or not, locked, committed, variant, pose, its parts (class now, landmark id, live, where the template puts it, class votes), and the detections dropped at intake by reason |
| `mission/wipe` | topic in (`Empty`) | Clear everything, including the course frame |
| `LandmarkPolling` | action server | Wait for a confirmed live track of a type/subtype (unchanged) |

## Conventions

- **Landmark frame**: origin in the object, +X out of the front, +Z down (NED). The parts of a course task have the task's yaw (its template's +X); the gate's front faces the start.
- **No orientation**: a rotation variance >= `intake.no_orientation_rot_variance` (1000) means position only. The tracker then leaves the orientation alone and `has_orientation` stays false.
- **Course frame**: odom X is the heading the ESKF started with, not the course direction. Nothing here uses fixed odom coordinates. With a course layout the lane is the area the layout covers (every task's search region and the start, plus `course.lane_margin_m`), turned and moved with the tasks found, so it does not jump when the gate locks the frame. Without a layout there is no lane. When the gate yaw is consistent for 10 estimates the frame moves to the gate; more than `warn_start_vs_gate_deg` off the start value gives a warning and the gate wins.

## Map rules (config `rules`, `classes`)

The gate and the torpedo board (pose, yaw, roles, openings) come from their
course templates (next section). What is left:

| Rule | What it does |
|---|---|
| Free landmarks | Classes in no course task (path markers): `classes.<CLASS>` gives `max_instances` and `retain_sec` (or `retain: forever`; `keep_after_observations`). Not listed: 20, 15 s. A new track within `instance_gate_m` of a remembered landmark of the same class takes over its id (`plausibility_radius_m` when `max_instances` is 1), but only when that landmark is clearly the nearest (`rules.adoption`) |
| Live only | `live_only: true` (the items on the table, which are moved during the run): followed by the tracker and published on `live_tracks`, never a landmark in `object_map` |
| Course frame | While both gate panels are seen, the line between them gives the course frame an estimate; panels closer than `min_panel_separation_m` (0.3) or farther than `max_panel_separation_m` (2.5) are not one gate |
| Bins | The role icon seen by the down camera hides the nearest bin without a role |
| Table and octagon | One xy for both, from `table_octagon.primary`: `table`, `octagon` or `midpoint`. The missing one is derived: the octagon over the table, or (with `z_lock` on) the table `table_height_m` above the floor under the octagon. With `z_lock` on the octagon floats at `surface_z` |
| Depth lock | `rules.z_lock`: floor classes get `floor_z`, surface classes `surface_z` (odom z, down positive). Entries can be a type (`OCTAGON`) or one subtype (`OCTAGON_WHOLE`). The table is not locked: its top is ~0.7 m above the floor. Per environment: measure the floor |
| Detector noise | `detector_noise` (per environment): std `base + along_std_per_m * d` along the line of sight, `base + across_std_per_m * d` across it, at range d. The tracker adds it to its class noise (`sens_mod_std_dev`), the graph uses it as is: far detections weigh less, and their depth, which a camera knows worst, least. One model, measured once (test C) |
| Detector covariance | `intake.measurement_covariance.use`: the position covariance of the detection (rotated into `target_frame`) replaces the class noise and the detector noise, in the tracker and the graph. `scale` multiplies it (testing), `min_std_m` is a floor. Off by default |
| Association | One tracker update per camera frame (same stamp), in time order; hits and misses are counted once per tick. Per class, global nearest neighbour: squared Mahalanobis distance as the cost, the gate (`gate.max_pos_error`, `mahalanobis_gate_threshold`) as the limit, the Hungarian algorithm for the one-to-one assignment. Each track is then updated with its own measurement |

## Course model (config `course`: `config/course/templates.yaml` + `config/course/<env>.yaml`)

The course layout is known before a run: which tasks there are, what each
looks like and roughly where it is. The map does not discover objects from
scratch; it places the known tasks and fills in their parts.

- **Templates** (`course.templates`, `course/templates.yaml`, the same in
  every pool): what a prop looks like. Its parts as classes at offsets in
  the task frame (+X out of the front, +Y right, +Z down), with `sigma` (how
  well the prop and the detector follow the drawing). A part can allow
  several classes (bin roles, baskets, octagon images: the votes decide), and
  a template can have variants that differ only in classes (the torpedo
  decal versions, decided by the votes, then fixed). The template also holds
  the tolerances of its tasks, the same for every task of that prop:
  `region_radius_m` [2.5] (how far the prior may be off), `part_radius_m`
  [1.0] (after placing), `yaw_window_deg` [30], `symmetric` [false] (looks
  the same turned around), `min_parts` [2] to place it (1: the part alone at
  the prior yaw), `max_range_m` [any] (farther detections of its parts are
  not taken).
- **Tasks** (`course.tasks`, `course/<env>.yaml`): where the props are in
  that pool, `{template, prior: [x, y, yaw_deg]}` in the course frame (origin
  at the gate, x through it). A task can set any of the tolerances above
  itself (a prior measured badly gets a larger `region_radius_m`).
  `course.start`: the start position in that frame, since the course frame
  starts at the start pose and moves to the gate when the gate locks it.
- **Checked keys**: a key the course does not know, in a task, a template,
  a part, a point or a variant (`region_radius:`, `form:`), stops the server
  at start with its name, instead of being ignored.
- **Class groups** (`course.class_groups`): classes a detector mixes up
  (white/red pipe, fire/blood, firetruck/ambulance, the role images). They
  are tracked as one kind; a part's class comes from the template (the red
  stands in the middle) or the votes, never from one detection.
- **Intake** (before the tracker): a detection outside the lane, or of a
  class that is a part of some task but far from every task that has it
  (outside the region of a task not placed yet, more than `part_radius_m`
  from the part of a placed one), or only in locked tasks, is dropped and
  counted (`course_state.drop_reasons`). Without a course frame no task class
  is taken. Classes in no template (table items, path markers) go on as free
  landmarks under `classes`.
- **Tracker limit**: per kind the parts in the course plus
  `extra_tracks_per_kind`; per other class `max_instances` plus that.
- **Placing a task**: its template is fitted to the confirmed tracks of its
  kinds in its region: yaw within the window, centre within the region, at
  least `min_parts` parts, reported classes not against the template.
- **Alignment**: the layout is turned and moved to fit the tasks found
  (least squares on their positions against their priors; one task gives
  the translation, two or more also the rotation, at most
  `course.max_align_deg`). A task not placed is searched there. A gate yaw a
  few degrees off (the panels are only 1.6 m apart) would otherwise put a
  task 14 m away 2 m beside its region.
- **Lane**: see the course frame above; detections outside are dropped at
  intake (`outside_lane`).
- **Parts**: one landmark per part, stable id, never forgotten, never more.
  A part follows a track while it stays within the part's gate (half the
  distance to the next part, at least `min_slot_gate_m`, horizontally); a
  free part takes the nearest unclaimed track of its kind in the gate (a
  remembered part keeps its id, so the graph sees it again). A track that
  fits no part is no object. The task pose is refitted to its parts (within
  the yaw window) and moves with the graph correction.
- **Derived points** (`points` in a template or a variant): points of a task
  that are not detected but follow from where it is, `{class, offset, from,
  yaw_deg}`: at the mapped position of the part `from` (else the task
  origin) plus `offset` turned with the task, facing the task's +X turned by
  `yaw_deg`. Published as derived landmarks with a stable id; for a
  template with variants only once the variant is decided. The torpedo
  openings are four of them per decal version. A new target of a task (an
  approach point, a passage) is a new entry, no code.
- **Orientation**: every part of a placed task has the task's yaw (+X out
  of the front of the prop), from the fit of all its parts. The course frame
  still locks to the gate from the two panels, only while both are being
  seen (the task pose is refitted from remembered parts every tick and
  looks consistent even when wrong); a lock a few degrees off is taken out
  by the alignment.
- **Focus** (`set_focus`): outside the focus with `lock_others`, a task is
  frozen (no new parts, no refit) and its detections are dropped. `commit`
  freezes a task's pose and variant.

`course.enable: false` (no layout yet, e.g. `pool.yaml` until it is
measured): every class is mapped as a free landmark, with no lane, no gate
yaw and no openings: a detector test, not a mission. The layout is read at
start (`course` is not live).

### A new or changed task

- Moved, more or fewer of a known task: edit `tasks` in `course/<env>.yaml`
  (and measure the priors). Only the tasks in the water: the map holds
  exactly their parts.
- A new arrangement of known classes: a new template in
  `course/templates.yaml` (parts, `sigma`, variants, `balanced_classes`,
  `points`, tolerances) and a task using it.
- A new kind of object: one constant in `vortex_msgs` (`LandmarkType` /
  `LandmarkSubtype`, subtypes named `<TYPE>_<NAME>`), rebuild; the class
  names are generated from the messages (`scripts/generate_class_names.py`),
  so the config knows it by name. The detector has to report it.

## Changing rules while it runs

The map rules (`intake`, `course_frame`, `classes`, `rules`, `markers`) are
ROS parameters: they show up in `ros2 param list`, Foxglove's parameter panel
and `ros2 param dump`, and a change applies at the next tick without losing
the map. Each change is parsed by the same code as at start; a bad value is
rejected with the reason and the old rules stay.

```bash
N=/nautilus/landmark_server_node
ros2 param set $N rules.z_lock.floor_z 3.6
ros2 param set $N classes.PATH_MARKER.retain_sec 30.0
ros2 param set $N rules.table_octagon.primary midpoint
ros2 param load $N src/vortex-auv/mission/landmark_server/config/pool.yaml  # a whole file
ros2 param dump $N > tuned.yaml                                               # keep what worked
```

- A key that is not in the config files can be set too (a setting left at
  its default, see "All settings"); the log warns, since a misspelt key is
  ignored. Keys that were removed are rejected with what replaced them.
- New class limits apply to new landmarks: a lower `max_instances` does not
  remove landmarks already in the map (`landmark_server/clear` does).
- `track_config`, `graph`, `detector_noise` and `course` are read at start: a
  new value is rejected with "restart the landmark server". Loading a whole file is fine as long as
  those values are unchanged.

## Smoothing backend (config `graph`)

Odometry drifts in x, y and yaw; a landmark remembered from 10 m ago is then
off by the drift since. `LandmarkGraph` (GTSAM iSAM2) keeps a factor graph of
vehicle keyframes (every `keyframe.distance_m` / `angle_deg` /
`interval_sec`) and landmark positions:

| Factor | From | Noise |
|---|---|---|
| Between keyframes | Odometry | `odom_noise`: std of one step, `pos_std_per_m` and `yaw_std_deg_per_m` times the distance (+ `yaw_std_deg_per_sec`). A drift that is a bias needs a larger value than the drift per metre |
| Roll, pitch, depth per keyframe | Odometry (IMU, pressure: no drift) | `absolute` |
| Keyframe → landmark position | Each measurement the tracker associated, relative to the nearest keyframe | `detector_noise` (the same model the tracker uses), Huber `measurements.huber_k` |
| Detector range scale | One unknown k for the whole run: every detection is (1 + k) times the true vector from the vehicle | Prior 0 ± `measurements.range_scale_std` (5 %) |

- **Range bias**: a camera's ranges are often a few percent off for every object (stereo calibration). Without k the graph can only explain that by stretching the path, and every task far from the start moves with it (in the simulator, 2 % too long ranges put the far tasks 0.15-0.2 m off with perfect odometry). k takes the bias instead; its estimate is in `graph/stats` and the 10 s log line (`detector range +2.1 %`), which also tells how far off the detector's ranges are.
- **The odometry noise must match the real drift** (`graph.odom_noise`, measured: "Measuring a pool"). Larger than the drift, the graph bends the path after the detection noise; smaller, it cannot follow the drift. Map error in the simulator (course route, two laps, two seeds each, k on):

  | `odom_noise` | No drift | Realistic drift | Worst drift |
  |---|---|---|---|
  | graph off | 0.055 m | 0.168 m | 0.350 m |
  | x1 (`sim.yaml`: the realistic profile) | 0.094 m | **0.064 m** | **0.090 m** |
  | x1/2 | 0.075 m | 0.092 m | 0.133 m |
  | x1/4 | **0.049 m** | 0.103 m | 0.173 m |

  Matched, the graph is never worse than without it. Too large costs a few cm when there is no drift; too small costs more when there is: when in doubt, round up. A model with the odometry's calibration (DVL scale, misalignment, gyro bias) as unknowns was tried and dropped: the DVL scale and the detector's range scale are hard to tell apart, and a gyro bias that wanders over minutes does not fit one constant.
- Measurements go to the graph under the **map id**, not the track id. A track that takes over a remembered landmark (adoption) adds to the same graph landmark: that closes the loop and moves the keyframes and every landmark they saw.
- Measurements of a track that is not in the map yet wait (`max_pending_per_track`) and are added when it is; at most `max_per_keyframe` per landmark and keyframe.
- Output: once a landmark has `min_observations` in the graph, its map position is the graph's, **in the current odom frame**: where it is relative to the vehicle according to the graph, expressed with the vehicle's raw odometry pose. The controller keeps steering on odometry. Orientation, derived landmarks and the depth lock work as before, on top.
- Not affected: `live_tracks` and `LandmarkPolling` (tracker), the course frame TF (follows the gate as the map gives it).
- A log line every 10 s gives keyframes, landmarks and the correction (odom ← graph).
- The noise values are guesses until the drift has been measured in the pool. The association itself does not get better: a remembered landmark must still be within the adoption radius when it is seen again.

### Trying it in the simulator

With `--drift`, `tmux_robosub_sim.sh` runs everything that sees the drifted
data in the frame `nautilus/odom_drift`, which `drift_injector.py` puts in TF
under the true `nautilus/odom`, and the graph-frame topics in `nautilus/odom`
(`graph.frame_id`, graph_eval `graph_frame_id`): Foxglove draws the maps and
the truth where they are in the world (a correct map stands still on the
green truth; white true path, green the graph's path on it, orange the raw
odometry drifting off). `--compare-install <dir>` runs the landmark_server of
another install space (an older branch) on the same data as `/nautilus_cmp`.

### Recording and replaying for offline tuning

```bash
utility_scripts/record_landmark_bag.sh A-loops [--images] [--extra '<regex>']
utility_scripts/replay_landmark_bag.sh ~/bags/<bag> --env pool \
    base "" nograph "-p graph.enable:=false" trial "-p graph.odom_noise.yaw_std_deg_per_m:=2.0"
```

`record_landmark_bag.sh` records the inputs (detections, odometry, TF), the raw
navigation sensors, what the live stack did, and a notes file, into
`~/bags/<date>_<test>` (mcap if the plugin is installed, else sqlite3).
`replay_landmark_bag.sh` plays only the inputs into one server per label
(`/tune_<label>/landmark_server/...`) with `use_sim_time`. Keep the rate at
1.0: the servers tick on wall time. The first Ctrl-C stops playback, the
second the servers. Checked on a simulator bag: the replayed map equals the
map the live server built (21 landmarks, 0.000 m apart).

Two scripts, run from the workspace root: the simulator and controller
(vortex-auv, tmux session `drone_launch`), then the perception and mission
chain (vortex-cv, tmux session `robosub_sim`):

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --headless --detach
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --fov          # baseline, no drift
src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --drift 0.5    # drift + a server without graph
ros2 run landmark_server drift_route.py                                    # then drive a loop
```

For the RoboSub course with rendering, start the simulator with
`launch_drone_sim.sh --scenario robosub --low-res --detach` instead.

Foxglove layout: `foxglove/landmark_graph.json` (Layout, Import from file). It shows the map, the truth from the course layout (green spheres), a line from each map landmark to its truth (blue with graph, red without), the true path (`/landmark_eval/true_path`, white, from graph_eval), the raw odometry path (`landmark_server/graph/odom_path`, orange) and the graph's corrected path in the graph frame (`landmark_server/graph/start_frame_path`, green: it should lie on the white one; `landmark_server/graph/path` is the same moved to the current odom pose, hidden), a plot of the path error of both against the truth, and plots of the map error, drift against correction and `landmark_server/graph/stats` (`[keyframes, landmarks, correction x, y, yaw deg, slowest update ms, detector range error %]`).

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

```bash
ros2 launch landmark_server landmark_server.launch.py env:=sim    # default
ros2 launch landmark_server landmark_server.launch.py env:=pool
ros2 launch landmark_server landmark_server.launch.py env:=pool course:=finals   # another task file
```

The launch file loads these, in this order (later files win):

| File | Contents | Who changes it |
|---|---|---|
| `config/landmark_server_config.yaml` | The tracker, the free classes, which classes are on the floor or at the surface | Rarely: behaviour, the same in every pool |
| `config/markers.yaml` | How the map is drawn in Foxglove | Never for tuning |
| `config/<env>.yaml` | Measured in that pool: floor depth, table height, `detector_noise`, odometry drift (`graph.odom_noise`). `pool.yaml` marks each value `MEASURE` with its test | Per pool, from measurements |
| `config/course/templates.yaml` | What each RoboSub prop looks like, and the tolerances of its tasks | Per competition (new props) |
| `config/course/<env>.yaml` | Where the tasks are: `start` and `{template, prior}` per task | Per pool |

### Measuring a pool

Two things are measured once per pool and vehicle: how much the odometry
drifts (`graph.odom_noise`) and how far off the detections are
(`detector_noise`). Neither needs an absolute position: a board that does
not move is the reference. With the map on the raw odometry, every time the
board is seen again, its apparent move is the drift since the first time.
An ArUco board gives its position to about 1 % of the range and its
orientation to a degree, so it also shows the heading drift directly.

**Set-up**

- The board (TAC board: four 15 cm markers, ids 28, 7, 96, 19, 0.43 x 0.83 m;
  another board: its sizes in `aruco_detector_params.yaml`), fixed upright
  to a wall or a weighted stand at the vehicle's depth, with 15-20 m of open
  water in front. It must not move or sway: anything it moves is measured as
  drift. Mark a spot 3 m in front of it (a weight on the floor, a lane line).
- Camera intrinsics calibrated, `aruco.marker_size` measured on the print.
- Start everything, the detector publishing to the landmark server's input:

```bash
ros2 launch aruco_detector aruco_detector.launch.py      # subs: front camera, pubs.landmarks: /nautilus/landmarks
ros2 launch landmark_server landmark_server.launch.py env:=pool calibration:=true
ros2 run landmark_server aruco_drift.py --ros-args -r __ns:=/nautilus -p csv:=$HOME/bags/aruco_visits.csv
src/vortex-auv/utility_scripts/record_landmark_bag.sh aruco-calibration
```

`calibration:=true` (`config/calibration.yaml`): no graph, no course, the
board kept for the whole session. `aruco_drift.py` splits the session into
visits (the board in view; 2 s out of view ends one) and prints after each
visit the jump against the first, and the values for `pool.yaml` so far.

**Check first:** hold still at the 3 m spot, tape the distance. The visit
line's `range` must read the taped distance within 3 cm; otherwise the
marker size or the camera calibration is wrong (a range error shows up as
drift and as the graph's `detector range` estimate).

**The session (about 30 minutes)**

Every return is to the same spot, 3 m in front of the board, facing it:
stop, hold 10 s with the board in view, then turn away so it is out of view.

| Test | What to do | Gives |
|---|---|---|
| B, hover | Hold still at 3 m for 90 s (at the start), and 2-3 min at the end | Hover drift: `min_pos_std_m`, `yaw_std_deg_per_sec` |
| C, ranges | Hold still 60 s each at 2, 4 and 6 m (turn away between them) | `detector_noise`: the spread along and across the line of sight, fitted over range |
| A, loops | Drive out and back 5, 10, 20 and 30 m with the turns of a real run (a 180 deg turn, a sideways leg), return to the spot each time; one loop clockwise, one counter-clockwise | Drift per metre: `pos_std_per_m`, `yaw_std_deg_per_m` (the slope of jump against distance driven) |
| D, range check | The taped check above, at 2 and 6 m too | Whether the detector's range is biased (fix the calibration if it is more than ~3 %) |

At the end (Ctrl-C) the script prints the `pool.yaml` lines: about twice
the measured drift (the drift is a bias, the graph's steps are
independent), and the detector noise. Copy them into `config/pool.yaml`.

**Check the result** on the recording: the board should hardly jump on a
return with the graph on and the new values (less than the detector noise,
~0.05-0.1 m), while it jumps by the drift without it.

```bash
C=install/landmark_server/share/landmark_server/config
src/vortex-auv/utility_scripts/replay_landmark_bag.sh ~/bags/<date>_aruco-calibration --env pool \
  raw "--params-file $C/calibration.yaml" \
  graph "--params-file $C/calibration.yaml -p graph.enable:=true"
ros2 run landmark_server aruco_drift.py --ros-args -r __ns:=/tune_graph \
  -p detections:=/nautilus/landmarks -p odom:=/nautilus/odom     # and /tune_raw
```

Pitfalls: a board that moves, returns at a steep angle (more than ~45 deg
off the board's normal the yaw gets noisy), the board in view while driving
(no clear visits), an odometry reset during the session (start again).

Also measure: the course (tape the props from the gate or take the
competition map: `course/pool.yaml` priors and `start`, then `enable: true`)
and the floor depth (`rules.z_lock.floor_z`, then `enable: true`).

### All settings

Everything not in the files has a default in the code. To change one, add it
to the right file (or `ros2 param set`, for the live ones).

| Setting | Default | When to touch it |
|---|---|---|
| `track_config.default.nm` | confirm 3 of 5, delete 5 of 7 | Clutter gets confirmed (stricter) or real objects take too long |
| `track_config.default.gate.max_pos_error` | 1.5 m | One object splits into several tracks (larger) |
| `track_config.default.dyn_mod_std_dev`, `sens_mod_std_dev` | 0.2, 0.2 m | Tracks too jumpy (smaller sens) or too slow (larger dyn) |
| `track_config.default.mahalanobis_gate_threshold` | 3.4 (99 % of true 3D detections) | Hardly ever |
| `track_config.<CLASS>.*` | the default | A class needs its own (see `SLALOM_PIPE`); `new_track_min_distance_m` |
| `detector_noise` | 0.06 m + 0.03/m along, 0.003/m across | Measured (test C) |
| `intake.measurement_covariance` | off | The detector gives a trustworthy covariance |
| `graph.enable` | true in the config | Comparing with and without drift correction |
| `graph.odom_noise.*` | `pool.yaml` | Measured (tests A, B) |
| `graph.keyframe`, `absolute`, `measurements` | 0.5 m / 10 deg / 5 s; 1 deg, 0.05 m; Huber 2, 3 per keyframe, 3 observations | Hardly ever |
| `graph.measurements.range_scale_std` | 0.05 (5 %) | 0 turns the range scale off; larger only for a detector whose range is known to be far off |
| `course.*` (globals) | `extra_tracks_per_kind` 2, `variant_votes` 40, `variant_ratio` 3, `min_class_agreement` 0.5, `min_part_detections` 8, `min_slot_gate_m` 0.2, `lane_margin_m` 2, `max_align_deg` 20 | A variant or role is decided too early/late (`variant_votes`); a pool much larger than the layout (`lane_margin_m`) |
| Template / task tolerances | see "Course model" | A task is not placed, or takes a neighbour's parts |
| `classes.<CLASS>` | 20 instances, 15 s, `instance_gate_m` 0.5 | Free classes only |
| `rules.adoption`, `plausibility_radius_m` | ratio 2, 2 s; 3 m | Free classes only |
| `rules.min_panel_separation_m`, `max_panel_separation_m` | 0.3, 2.5 m | Another gate size |
| `rules.table_octagon` | `primary: table`, `table_height_m` 0.7 | Measured (table height) |
| `rules.z_lock` | off | Measured (floor) |
| `course_frame.gate_lock`, `warn_start_vs_gate_deg` | 10 estimates within 3 deg; 30 deg | Hardly ever |
| `timer_rate_ms`, `topics.*`, `debug.*` | 200 ms, the usual names, off | Never for tuning |

## Files

- ROS-free (gtest): `class_config`, `retained_landmarks`, `course_model` (tasks, parts, intake regions, focus), `structures` (template fitting), `course_frame`, `map_rules`, `landmark_graph` (own library, the only one that includes GTSAM). `test_config_files` parses the shipped config files, so a broken file fails the tests
- ROS: `landmark_server_ros.cpp` (intake, tick, polling, reset), `landmark_server_publish.cpp` (map, live tracks, course frame, services), `landmark_server_graph.cpp` (odometry, measurements to the graph, smoothed positions into the map), `landmark_server_course.cpp` (intake gate, tracker limits, set_focus, course_state)

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
