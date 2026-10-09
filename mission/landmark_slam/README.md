# landmark_slam

The RoboSub landmark map. Estimates the pose (x, y, z, yaw) of every course
object in the `map` frame and corrects odometry drift by publishing
`map → odom`. It is the only owner of landmark state; the behavior tree reads
it through the landmark nodes in `vortex_bt_nodes`.

```
odom ──────────────▶ keyframes (every keyframe_dist_m / keyframe_time_s)
landmarks ─▶ JCBB ──▶ matched: observation factors ──▶ iSAM2 ─▶ landmarks, map → odom
              └─────▶ explained by no landmark: votes ─▶ new landmark
prior_map.yaml ────▶ start pose; where votes of a class are taken (optional)
```

## Method

- **Graph (iSAM2).** Keyframes `X(k)` and landmarks `L(id)`, both `Pose3` in
  `map`. Factors: odometry between keyframes (noise grows with the distance
  travelled), absolute depth (pressure) and roll/pitch (gravity) per
  keyframe, the start pose, and detections: bearing-range, or the
  relative pose when the detector gives an orientation and the class has
  one (the yaw snapped to the symmetric hypothesis closest to the estimate).
  Detection factors use Dynamic Covariance Scaling (Agarwal et al.
  2013), so wrong matches and moved props are down-weighted without tuning.
- **Association (JCBB, Neira & Tardós 2001).** All detections of one message
  together, only against landmarks of the same class. Each pair is gated on
  its own, then the largest set of pairs that passes the joint χ² test is
  kept, with the keyframe–landmark cross-covariances. A match that another
  candidate explains nearly as well (likelihood ratio < 10) is ambiguous and
  dropped until the objects can be told apart.
- **New landmarks by voting.** A detection that no landmark could explain is
  a vote at its map position. A vote's weight halves every 10 s. Where the
  votes of a class within `vote_radius_m` add up to `min_votes`, they become
  a landmark at their weighted centroid, and every vote is added to the
  graph as an observation. Votes at an existing landmark of the class are a
  second look at it and are dropped; for a class in the prior map, votes far
  from every prior entry are rejected. A landmark seen fewer than 30 times
  and not for 30 s is hidden (a phantom or a bad view); the class frame
  `<class>` is the landmark of the class seen most often.
- **Duplicates.** Two landmarks of a class closer than `vote_radius_m`
  whose positions agree (χ² at `gate_prob`) are one object: the one seen
  less is dropped (no more observations, not shown). It is not tied to the
  other: its factors may come from wrong detections, and tying them in
  pulled the kept one off by up to 0.5 m in the replays.
- **Consistency (NIS).** `landmark_slam/nis` is the mean normalised
  innovation squared of the last 50 matches. ≈ 1: the noise values fit;
  ≫ 1: too optimistic; ≪ 1: too pessimistic. Outside [0.5, 2] the node
  warns: recalibrate, don't tune.

## Interfaces (under the drone namespace)

| Interface | Type | |
|---|---|---|
| `odom` | in, `nav_msgs/Odometry` | Vehicle pose in odom (frame and child frame from the message) |
| `landmarks` | in, `vortex_msgs/LandmarkArray` | Detections, 3D position in the camera frame (TF to odom at the image time) or in odom. Rotation variance ≥ 1000 = no orientation |
| `mission/wipe` | in, `std_msgs/Empty` | Reset: reload the YAML files, clear the graph, map = start pose again |
| `landmark_slam/landmarks` | out, `vortex_msgs/LandmarkTrackArray` (transient local) | The map, after every keyframe. Per landmark: `id`, `type`/`subtype`, pose in `map`, `observations`, `first_seen`, `last_measurement`, `has_orientation`. The position covariance is relative to the vehicle (what navigating to it depends on); the rotation covariance is the landmark's own |
| `landmark_slam/markers` | out, `MarkerArray` | Sphere at 2σ per landmark and a label (RViz/Foxglove) |
| `landmark_slam/nis` | out, `std_msgs/Float64` | See above |
| TF `<ns>/map → <ns>/odom` | out | With every odometry message, moved toward the graph's value at ≤ 0.2 m/s and 0.2 rad/s so the controller never sees a jump |
| TF `<ns>/map → <ns>/<class>_<id>` | out | Every landmark as a frame, 10 Hz, same stamp as `map → odom`. Looked up from `odom`, it is where the drifted vehicle has to go: the correction comes through `map → odom` |
| TF `<ns>/<class>` | out | Per class the best landmark (lowest σ_xy, then most observations): a target before the id is known, e.g. `torpedo_board` |
| TF `<ns>/start` | out | Where the run started (keyframe 0): return home |
| TF `<ns>/prior_<class>` | out | Where the class should be (prior map: the mean of its entries, depth 0, along the map's x axis): the tree's search point before the object is seen |
| TF `<ns>/gate_middle`, `<ns>/<panel>_entrance`, `<ns>/<panel>_exit` | out | Gate frames (below) |

## Configuration

`ros2 launch landmark_slam landmark_slam.launch.py [env:=pool|sim] [params_file:=…] [classes_file:=…] [prior_map_file:=…]`

`env:=sim` loads `params_sim.yaml` on top (the simulator's measured noise)
and `prior_map_sim.yaml` (the simulator course).

| Parameter | Default | |
|---|---|---|
| `use_prior_map` | true | Load `prior_map.yaml`; else `map` = odom at startup |
| `prior_map_file`, `classes_file` | config/ | Paths (launch arguments) |
| `prior_map` | the file's text | The prior map as text: set it to send a new one (runtime, see below) |
| `keyframe_dist_m`, `keyframe_time_s` | 0.5, 1.0 | New keyframe after this distance or time |
| `odom_sigma_trans_per_m`, `odom_sigma_yaw_per_m` | 0.03, 0.01 | Odometry noise per metre (measured) |
| `default_prior_sigma_xy` | 1.0 | Prior entry `sigma_xy` when it gives none: votes are taken within 3 σ + `vote_radius_m` of an entry |
| `gate_prob` | 0.999 | Association gate (χ² per degree of freedom) |
| `min_votes`, `vote_radius_m` | 3, 0.5 | New landmark: vote weight within the radius |
| `gate.panel_classes`, `gate.min_separation_m`, `gate.max_separation_m`, `gate.approach_m`, `gate.depth_below_panel_m` | panels, 0.2, 2.5, 1.0, 0.5 | Gate frames |
| `bearing_sigma` | 0.03 | Detection bearing noise [rad] (measured) |
| `range_sigma_a`, `range_sigma_b` | 0.1, 0.05 | Range noise σ_r = a + b·r (measured) |

Everything else is a named constant in the code.

**Gate frames** (`gate`): from the two role panels, when both are mapped
and `min_separation_m`–`max_separation_m` apart. `gate_middle` between them,
and per panel `<panel>_entrance` / `<panel>_exit`, `approach_m` before and
after the gate line and `depth_below_panel_m` below the panel: through that
role's opening. All have +X through the gate, away from the start side, so
they do not flip once the vehicle is through. Drive them with `GoToFrame`
(vortex_bt_nodes).

**The start and the coin flip**: the map is anchored at `mission/wipe` (or
at the first odometry): the vehicle's pose then is `initial_pose`. So at the
start: put the vehicle at the start facing along `initial_pose.yaw` (the
course direction), anchor, then turn it for the coin flip. The odometry
follows the turn, so the map keeps the course's heading and no angle is
entered; the tree's first move turns the vehicle to the course. Anchor
before autonomous mode: `mission/wipe` also stops waypoint_manager's and the
reference filter's goals.

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty   # or the GUI's Anchor
```

**Adding an object type** is a YAML entry only: a class in
`landmark_classes.yaml` (the `type`/`subtype` values the detector publishes,
`symmetry_deg`, `has_orientation`), and optionally entries in
`prior_map.yaml` (the class, x, y, and how far the object can be from the
drawing). The prior map does not pull the map: a prior with an error per task
(a whole slalom set 0.5 m off) would bend it. It only decides where votes for
a class count, so a false detection far from where the object can be (a gate
post taken for a slalom pipe) never becomes a landmark.

**Changing the prior map without a restart** (from the topside, while the
vehicle is up): the parameter `prior_map` holds the prior map as text (at
startup, the file's). Setting it checks it (a bad map is rejected with the
reason), saves it on the vehicle (`$ROS_HOME/landmark_slam/prior_map.yaml`,
the old one kept with a time stamp) and uses it from the next anchoring
(`mission/wipe`), when the map is rebuilt: never halfway through a run. After a restart the launch file's map is used again; to keep
a sent one, launch with the path the log gives or copy it into `config/`.

```bash
ros2 param set /nautilus/landmark_slam_node prior_map "$(cat prior_map.yaml)"
```

Or draw it: `prior_map_gui.py` gets the prior map from landmark_slam, with
the live map and the vehicle on top. Place the start where the vehicle is
put in and the objects relative to it (click: new entry, drag: move, right
click: remove). After a practice run the live map shows where the objects
really are (crosses in the class colour with how often each was seen; a real
object is seen far more often than a phantom): drag the entries there.
*Send* sets `prior_map`; *Anchor here* starts a new run at the vehicle's
pose (`mission/wipe`); *Save as* keeps a copy on the topside.

```bash
ros2 run landmark_slam prior_map_gui.py --ros-args -r __ns:=/nautilus
```

## Noise calibration (once, not tuning)

Record a pool run near landmarks whose poses are known, with landmark_slam
running (for `map → odom`), then:

```bash
ros2 run landmark_slam calibrate_noise.py <bag> <truth.yaml> config/landmark_classes.yaml
```

`truth.yaml` is in the `prior_map.yaml` format with x, y and z per landmark. The script prints
`bearing_sigma`, `range_sigma_a`, `range_sigma_b` (robust spread of the
detection errors against the truth, range fitted per 1 m bin) and, from the
`map → odom` corrections per 0.5 m travelled, `odom_sigma_trans_per_m` and
`odom_sigma_yaw_per_m`. Paste them into `params.yaml`, replay the bag and
check that `landmark_slam/nis` stays near 1.

## Known limits

- Detections are used at the keyframe rate: per source (frame and types),
  the last 5 messages since the last keyframe, each associated on its own;
  the detections matched to one landmark become one factor (their mean,
  counted as at most 4 detections). Only the newest message votes for new
  landmarks.
