# landmark_slam

The RoboSub landmark map. Estimates the pose (x, y, z, yaw) of every course
object in the `map` frame and corrects odometry drift by publishing
`map → odom`. It is the only owner of landmark state; the behavior tree reads
it through the landmark nodes in `vortex_bt_nodes`.

```
odom ──────────────▶ keyframes (every keyframe_dist_m / keyframe_time_s)
landmarks ─▶ JCBB ──▶ matched: observation factors ──▶ iSAM2 ─▶ landmarks, map → odom
              └─────▶ explained by no landmark: votes ─▶ new landmark
prior_map.yaml ────▶ prior factors (optional)
```

## Method

- **Graph (iSAM2).** Keyframes `X(k)` and landmarks `L(id)`, both `Pose3` in
  `map`. Factors: odometry between keyframes (noise grows with the distance
  travelled), absolute depth (pressure) and roll/pitch (gravity) per
  keyframe, prior map entries, and detections: bearing-range, or the
  relative pose when the detector gives an orientation and the class has
  one (the yaw snapped to the symmetric hypothesis closest to the estimate).
  Detection and prior factors use Dynamic Covariance Scaling (Agarwal et al.
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
  from every prior entry are rejected.
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
| `landmark_slam/landmarks` | out, `vortex_msgs/LandmarkTrackArray` (transient local) | The map, after every keyframe. Per landmark: `id`, `type`/`subtype`, pose in `map`, `observations` (0 = prior only), `first_seen`, `last_measurement`, `has_orientation`. The position covariance is relative to the vehicle (what navigating to it depends on); the rotation covariance is the landmark's own |
| `landmark_slam/markers` | out, `MarkerArray` | Sphere at 2σ per landmark and a label (RViz/Foxglove) |
| `landmark_slam/nis` | out, `std_msgs/Float64` | See above |
| TF `<ns>/map → <ns>/odom` | out | With every odometry message, moved toward the graph's value at ≤ 0.2 m/s and 0.2 rad/s so the controller never sees a jump |

## Configuration

`ros2 launch landmark_slam landmark_slam.launch.py [params_file:=…] [classes_file:=…] [prior_map_file:=…]`

| Parameter | Default | |
|---|---|---|
| `use_prior_map` | true | Load `prior_map.yaml`; else `map` = odom at startup |
| `prior_map_file`, `classes_file` | config/ | Paths (launch arguments) |
| `keyframe_dist_m`, `keyframe_time_s` | 0.5, 1.0 | New keyframe after this distance or time |
| `odom_sigma_trans_per_m`, `odom_sigma_yaw_per_m` | 0.03, 0.01 | Odometry noise per metre (measured) |
| `default_prior_sigma_xy`, `_z`, `_yaw` | 1.0, 0.3, 0.5 | Prior entry trust when the entry gives none |
| `gate_prob` | 0.95 | Association gate (χ² per degree of freedom) |
| `min_votes`, `vote_radius_m` | 3, 0.5 | New landmark: vote weight within the radius |
| `bearing_sigma` | 0.03 | Detection bearing noise [rad] (measured) |
| `range_sigma_a`, `range_sigma_b` | 0.1, 0.05 | Range noise σ_r = a + b·r (measured) |

Everything else is a named constant in the code.

**Adding an object type** is a YAML entry only: a class in
`landmark_classes.yaml` (the `type`/`subtype` values the detector publishes,
`symmetry_deg`, `has_orientation`), and optionally entries in
`prior_map.yaml`. Same-class objects close together (slalom pipes) need a
prior `sigma_xy` below a quarter of their spacing; the node warns at startup
when two entries are too loose to tell apart.

## Noise calibration (once, not tuning)

Record a pool run near landmarks whose poses are known, with landmark_slam
running (for `map → odom`), then:

```bash
ros2 run landmark_slam calibrate_noise.py <bag> <truth.yaml> config/landmark_classes.yaml
```

`truth.yaml` is in the `prior_map.yaml` format. The script prints
`bearing_sigma`, `range_sigma_a`, `range_sigma_b` (robust spread of the
detection errors against the truth, range fitted per 1 m bin) and, from the
`map → odom` corrections per 0.5 m travelled, `odom_sigma_trans_per_m` and
`odom_sigma_yaw_per_m`. Paste them into `params.yaml`, replay the bag and
check that `landmark_slam/nis` stays near 1.

## Known limits

- A prior entry that gets a few outlier matches while the start pose is
  still badly off can stay off; the object's real detections then vote a new
  landmark at the right place, which wins `best of class` (lower σ, more
  observations).
- Detections are used at the keyframe rate: per source (frame and types),
  the latest message since the last keyframe.
