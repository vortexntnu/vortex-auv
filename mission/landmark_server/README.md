# landmark_server

The landmark map. It estimates the pose of every course object in the `map`
frame, keeps them for the whole run (the retained map), corrects odometry
drift with `map → odom`, and publishes TF frames to navigate by: search
points from the prior map, the objects, the gate's entrance and exit, and
the start (return home). It never moves the vehicle.

```
odom ─────────▶ keyframes (every keyframe_dist_m / keyframe_time_s)
detections ──▶ association (per class: Mahalanobis gate + Hungarian)
                 ├─ matched ─────────▶ factors ─▶ iSAM2 ─▶ landmarks, map → odom
                 └─ in no gate ─▶ candidate ─▶ confirmed (N hits in T s) ─▶ new landmark
upkeep: phantoms retired, duplicates retired, max_instances shown per class
premap.yaml / set_premap ─▶ where new landmarks may appear, prior_<task> frames (never in the graph)
```

## How it works

- **Graph (iSAM2).** Keyframes `X(k)` and landmarks `L(id)`, both `Pose3` in
  `map`. Factors: odometry between keyframes (noise grows with the distance
  travelled), absolute depth (pressure) and roll/pitch (gravity) per
  keyframe, and detections: bearing-range, or the relative pose when the
  detector gives an orientation and the class has one (the torpedo board's
  surface normal). Detection factors use Dynamic Covariance Scaling, so a
  wrong match is down-weighted instead of bending the map.
- **The map frame** is the vehicle at startup or at `mission/wipe`: x along
  its heading, y right, z down (depth, as odom). Anchor with the vehicle at
  the start facing the course, then turn it for the coin flip: the map keeps
  the course heading.
- **Association.** Per keyframe, each detection message on its own. Per
  class, every detection–landmark pair gets the squared Mahalanobis distance
  of its innovation (from the joint marginal of the keyframe and the
  landmark), the χ² gate (`gate_prob`) limits it and the Hungarian algorithm
  makes the one-to-one assignment. A match another landmark explains nearly
  as well (`ambiguity_d2`) is dropped until the view is clearer.
- **Class voting.** Classes one object can be taken for share a `group`
  (the white and the red slalom pipe). A detection of any class of the
  group is matched to the same landmark, each detection is a vote, and the
  landmark's class is the majority. A run of wrong-colour detections from
  far away then loses the vote instead of becoming a second landmark at the
  same place.
- **Loop closure.** Every landmark in the map takes part in the association,
  not only the ones seen lately. Seeing a remembered landmark again after a
  loop adds factors under its id: the graph moves the keyframes since, and
  `map → odom` takes the drift out. The gate grows with the uncertainty since
  the landmark was last seen, so a drifted landmark is still found.
- **New landmarks.** A detection in no landmark's gate is a candidate.
  Candidate detections of a class within `candidate_radius_m` add up;
  `confirm_hits` of them within `confirm_window_s` make a landmark, with all
  of them as its first factors. A candidate at an existing landmark of its
  class is a second look at it and dropped.
- **Retained map.** Landmarks are never deleted: not for being out of view,
  not for being seen rarely. Phantoms (things that are not there but are
  detected again and again at one place) are kept out by the prior gate
  (none far from their task), `confirm_hits` (none from one-off detections)
  and `max_instances`: per class only the most observed are published, and
  the rest keep absorbing their own detections so they never pull a real
  landmark. The one exception: two landmarks of a class closer than
  `merge_radius_m` whose positions agree (χ² at `gate_prob`) are one object,
  for example a copy made while the odometry had drifted. The one seen less
  is retired: no more matches, not published, its factors stay.
- **Prior map (outside the graph).** It only says where each task is
  roughly (one entry per task: `gate`, `slalom`, `torpedo`, `bin`,
  `octagon`). A class names its task (`prior`); a new landmark of the class
  is only made within `prior_radius_m` (xy) of it, so a false detection far
  from where the object can be never becomes a landmark. It also gives the
  search frames `prior_<task>`. It never pulls the map: a prior with an
  error per task would bend it.

## Interfaces (under the drone namespace)

| Interface | Type | |
|---|---|---|
| `odom` | in, `nav_msgs/Odometry` | Vehicle pose in odom (frame and child frame from the message) |
| `landmarks` | in, `vortex_msgs/LandmarkArray` | Detections. `header.stamp` = image time; frame = camera (TF to odom at the image time) or odom. Rotation variance ≥ 1000 = position only |
| `mission/wipe` | in, `std_msgs/Empty` | New map with the vehicle's pose as the start |
| `landmark_server/landmarks` | out, `LandmarkTrackArray` (transient local) | The map after every keyframe: `id`, `type`/`subtype`, pose in `map`, `observations`, `first_seen`, `last_measurement`, `retained` (not seen since the last keyframe), `has_orientation`. Position covariance: relative to the vehicle (what navigating to it depends on) |
| `landmark_server/markers` | out, `MarkerArray` | A sphere at 2σ per landmark (green seen now, grey remembered) and a label |
| `landmark_server/nis` | out, `Float64` | Mean normalised innovation squared of the last 50 matches: ≈ 1 when the detection noise values fit |
| `landmark_server/set_premap` | service, `vortex_msgs/SetPremap` | Replace the prior map (poses in `reference_frame`: `start` or `odom`). Saved to `premap_file`, the old file kept as `<name>_<YYYYmmdd_HHMM>.yaml`. Used at once |
| `landmark_server/get_premap` | service, `std_srvs/Trigger` | The prior map as YAML text in `message` |
| TF `map → odom` | out | With every odometry message |
| TF `map → <class>_<id>` | out | Every published landmark, 10 Hz, same stamp as `map → odom`: looked up from odom it is where the drifted vehicle has to go |
| TF `map → <class>` | out | Per class the landmark seen most often (a target before its id is known) |
| TF `map → prior_<task>` | out | Where the task should be (prior map): the search point before it is seen |
| TF `map → start` | out | Where the run started: return home |
| TF `gate_middle`, `<panel>_entrance`, `<panel>_exit` | out | From the two gate panels: +X through the gate, away from the start side |

## Navigating with it

Every target is a TF frame; look it up from odom and send it to
`waypoint_manager` (a frame moves when the map corrects drift, so re-send it
when it moves).

- **Search:** `prior_<task>` until the object is in the map, then `<class>`.
- **Aim:** `<class>_<id>` or `<class>` up close (the torpedo board's yaw
  comes from its measured normal: +X out of the front).
- **Through the gate:** `<panel>_entrance` → `<panel>_exit`.
- **Return home:** `<panel>_exit` → `<panel>_entrance` (the same frames,
  driven the other way, facing −X), then `start`. These frames come from the
  corrected map, so the way home uses the loop-closed gate.

## Running

```bash
ros2 launch landmark_server landmark_server.launch.py              # env:=pool
ros2 launch landmark_server landmark_server.launch.py env:=sim     # sim.yaml, premap_sim.yaml
```

| Argument | Default | |
|---|---|---|
| `env` | `pool` | `sim`: the simulator's detection noise (`config/sim.yaml`) and course (`config/premap_sim.yaml`) |
| `config_file` | `config/landmark_server.yaml` | All tunable values |
| `premap_file` | `config/premap.yaml` (`premap_sim.yaml` with `env:=sim`) | The prior map, written by `set_premap`. Build with `--symlink-install` to write the source file |
| `odom_topic`, `landmarks_topic` | the robot file's `odom`, `landmarks` | Other topics, e.g. the drift injector's in the simulator |

Start a new map (anchor) with the vehicle at the start facing the course:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

## The prior map and its GUI

`premap.yaml` (written by the GUI through `set_premap`):

```yaml
reference_frame: start            # the map frame
created_at: '2026-10-09T12:00:00'
objects:
  torpedo: {position: [17.0, -5.2, 2.5], orientation: [0.0, 0.0, 1.0, 0.0]}
gui_state: {...}                  # the GUI's drawing
```

```bash
ros2 run landmark_server competition_map_gui.py --ros-args -r __ns:=/nautilus \
    [-p pool_width_m:=50.0 -p pool_height_m:=25.0]
```

The GUI is the pool from above (50 x 25 m by default). It opens with the
prior map the vehicle has (the same as **Get from Vehicle**) and draws the
**live map** on top: every landmark the vehicle has found, as a cross in its
task's colour with the number of detections ("Show live map").

- **reference = the start of the run**: where the vehicle is when the map
  is anchored (`mission/wipe`), its arrow the vehicle's heading then (the
  map's x). Everything is measured from it: the side panel shows each task
  as x ahead / y right of the start, which is what is sent.
- Select an object, click to place it, turn lines (gate, slalom, torpedo:
  the arrow is the side the vehicle comes from) and the reference with the
  yaw slider. Nothing is selected at first, so a stray click moves nothing;
  the slider takes the selected object's yaw.
- The **dashed circle** around a task is where the vehicle accepts new
  landmarks of it (the classes' `prior_radius_m`, from the vehicle): the
  real object has to be inside.
- Sending a **moved reference** asks first: it shifts every task in the map,
  and detections outside the circles are then rejected.
- **Send to Vehicle** sets the prior map at once and saves it (the old file
  kept with a time stamp); `prior_<task>` moves in Foxglove. Depths are the
  ones loaded from the vehicle, else `DEFAULT_Z` in the script.
- After a practice run, the crosses show where the objects really are:
  move the tasks onto them and Send. A task only needs to be within its
  classes' `prior_radius_m` (3 m, slalom 4 m).
- Reference frame `start` (default): poses relative to the reference.
  `odom`: the poses are odom coordinates, converted with the current
  `map → odom`.

## Parameters

Everything is in `config/landmark_server.yaml`, with a comment per value.
Write floats with a decimal point (`3.0`): ROS parameters are typed.

| Group | Values | Raise it when | Lower it when |
|---|---|---|---|
| keyframes | `keyframe_dist_m`, `keyframe_time_s`, `max_messages_per_keyframe`, `max_range_m` | the node is too slow (larger keyframe steps) | far false detections become landmarks (`max_range_m`) |
| `odom.*` | odometry noise per metre and per step, absolute roll/pitch/depth | loops do not close (re-seen landmarks fall outside the gate, duplicates after a loop) | the map wobbles between landmarks |
| `detection.*` | bearing σ, range σ = a + b·r, orientation σ, `dcs_phi`, `max_merged_per_factor` | NIS ≫ 1 | NIS ≪ 1 |
| `association.*` | `gate_prob`, `ambiguity_d2` | real objects get duplicates (`gate_prob`) | neighbours steal each other's detections |
| `new_landmarks.*` | `candidate_radius_m`, `confirm_hits`, `confirm_window_s` | phantoms become landmarks (`confirm_hits`) | real objects take too long to appear, or a slow detector never confirms (hits per window ≤ its rate) |
| `upkeep.merge_radius_m` | | a drifted copy of an object stays next to it | two real same-class objects get joined |
| `gate.*` | panel classes, separation limits, `approach_m`, `depth_below_panel_m` | | |
| `classes.<name>` | `type`, `subtype`, `symmetry_deg`, `has_orientation`, `prior`, `prior_radius_m`, `max_instances`, `group` | real objects are rejected near the edge of their task (`prior_radius_m`) | false detections next to a task become landmarks |

Tune with the NIS first (detection noise), then the odometry noise on a
loop, then the new-landmark values and `max_instances` against phantoms. One change at a time.

## Testing with the dummy publisher

`robosub_dummy_publisher` (vortex-cv) publishes the course's detections.
With `profile:=realistic` it behaves like a camera on a moving vehicle: only
what is in view (`front_range_m`, `front_half_fov_deg`), and every error
effect is a `[near, far]` pair. Within `near_range_m` and
`centre_half_fov_deg` of the image centre, detections are accurate and
consistent. Toward the range limit and the edge of the view they are noisy,
missed, occluded, cluttered, sometimes far off or of the wrong class, and
phantoms (things that are not there) show up. The torpedo board comes with
its surface normal. Its noise in `sim.yaml` matches the realistic profile.

```bash
# The simulator (vehicle side), then the server and the dummy
src/vortex-auv/utility_scripts/launch_drone_sim.sh --headless --detach
ros2 launch landmark_server landmark_server.launch.py env:=sim
ros2 launch robosub_dummy_publisher robosub_dummy_publisher.launch.py profile:=realistic seed:=7
```

Watch `landmark_server/markers` and the TF frames in Foxglove, and
`landmark_server/nis`. Drive the vehicle up to the torpedo board: far away
the board wobbles and phantoms appear and then get retired; up close it
settles and its yaw (normal) is right. Drive back to the gate after a loop:
the gate is matched again under its id (no duplicate) and `map → odom` jumps
by the drift.
