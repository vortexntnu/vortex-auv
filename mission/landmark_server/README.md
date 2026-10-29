# landmark_server

Takes object detections from perception and keeps a map of the course. The
mission drives to the map's TF frames to find a task, then measures the
object close up with the `measure` action and drives to that.

## Core concepts

- **Hit**: one detection, stored in `odom`.
- **Landmark**: a cluster of hits of one class group. Its position is the
  median of the hits.
- **Class**: a kind of object, e.g. `torpedo_board`. Listed under `classes`
  in `config/landmark_server.yaml`. Detections of other types are ignored.
- **Measured frame**: an object measured from fresh, close detections while
  the vehicle holds still. Fixed in `odom` until measured again.
- **Target frame**: a place to go next to a frame, e.g. 2 m in front of the
  board. Offsets in the config.
- **Prior map**: roughly where each task is, relative to the start, drawn in
  the GUI. Gives `prior_<task>` frames and drops detections far from their
  task.

There is no drift correction. Landmarks live in `odom`, the frame the
vehicle drives in, so a landmark seen recently is where the vehicle will find
it. For the final approach the mission measures again, close up.

## Run

```bash
ros2 launch landmark_server landmark_server.launch.py            # pool
ros2 launch landmark_server landmark_server.launch.py env:=sim   # simulator, premap_sim.yaml
```

### Starting a map

The map starts where the vehicle is. Which way its X points is set apart
from that, so the vehicle can be turned before the run (the coin flip):

1. Put the vehicle at the start, pointing the way the reference points in
   the prior map, and set the course heading:

   ```bash
   ros2 topic pub --once /nautilus/mission/set_heading std_msgs/msg/Empty
   ```

2. Turn or move the vehicle as the run needs.
3. Start a new map where it is now. The heading from step 1 is kept:

   ```bash
   ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
   ```

The GUI has a button for each. After step 3 the vehicle's heading in the
map is how far it is turned from the course, so a mission that turns to map
heading 0 first faces the course again.

The heading is kept as an odometry yaw. It holds for as long as the
odometry is not restarted, and is forgotten when `landmark_server`
restarts. Without step 1 a new map points the way the vehicle points.

## Topics, services, actions

| Name | Kind | Type |
|---|---|---|
| `odom` | in | `nav_msgs/Odometry` |
| `landmarks` | in | `vortex_msgs/LandmarkArray` |
| `mission/wipe` | in | `std_msgs/Empty` |
| `mission/set_heading` | in | `std_msgs/Empty` |
| `landmark_server/landmarks` | out, in `odom` | `vortex_msgs/LandmarkTrackArray` |
| `landmark_server/measure` | action | `vortex_msgs/MeasureLandmark` |
| `landmark_server/reset` | service | `vortex_msgs/ResetLandmarks` |
| `landmark_server/set_premap`, `get_premap` | service | prior map, used by the GUI |

Detectors should publish in the camera frame with the image stamp. The server
puts each detection in `odom` with TF at that stamp. Detections already in
`odom` or `map` are used, with a warning.

## TF frames

| Frame | Parent | |
|---|---|---|
| `<class>` | `odom` | The most observed landmark of the class. Until the class is seen: its `prior_<task>`, under `map` |
| `<class>/live` | `odom` | The newest detection, for 1 s after it |
| `<class>/measured`, or the action's `out_frame` | `odom` | The last successful measurement |
| `<parent>/<target>` | the parent | Target frames from the config |
| `gate_middle`, `<panel>_entrance`, `<panel>_exit` | `odom` | Gate, from the two role panels |
| `start` | `map` | Where the map started |
| `prior_<task>` | `map` | Where the prior map puts the task |

`map -> odom` is fixed when the map starts (`wipe`, `set_heading`, or the
first odometry): `map` is the start pose turned to the course heading.

So `<class>` always exists for a class with a task in the prior map. Check
`landmark_server/landmarks` (or `LandmarkConfirmed`) to know whether it was
seen. Frames from best to worst for the final approach:
`<class>/measured` → `<class>/live` → `<class>` → `prior_<task>`.

Frame axes: +X forward, +Y right, +Z down. Objects with a front (the board,
bins, the table) have +X out of the front. Objects without a yaw get +X from
the vehicle to the object when measured, and no yaw in the map.

## How a detection becomes a landmark

```
detection
  │  type/subtype not a class, NaN, bad quaternion ─────▶ dropped
  │  farther than max_range_m from the vehicle ─────────▶ dropped
  │  farther than the task radius from prior_<task> ────▶ dropped
  ▼
stored in odom, the newest max_hits_per_class per class
  ▼  twice a second, per class group
densest hit, the hits within cluster_radius_m of it ─▶ a landmark if ≥ confirm_hits
  │  remove them, repeat
  ▼
class: majority of its hits. Position: median. Yaw: circular mean
  ▼
shown: the max_instances most observed of each class
```

- Landmarks are built again every time, so a bad hit cannot stick.
- Densest first: hits between two pipes do not join them into one.
- A landmark keeps its id while it moves less than twice `cluster_radius_m`.
- The covariance in `LandmarkTrackArray` is the spread of the hits squared
  over their number. `observations` is the number of hits.

## Measure

```
goal: class_name, out_frame, duration_s, max_range_m, min_hits, max_spread_m, fallback_from
```

1. Hold the vehicle still near the object.
2. The action collects the detections of the class that arrive during
   `duration_s` and are within `max_range_m` of the vehicle. The map and the
   prior gate are not used.
3. The largest cluster is the object. It fails with fewer than `min_hits`
   or a spread (RMS distance of the hits from the median) over
   `max_spread_m`.
4. On success `out_frame` (default `<class_name>/measured`) is set in `odom`.
   It stays there until the next measurement with that name.
5. On failure the old `out_frame` stays. With `fallback_from` set, that frame
   is copied to `out_frame` and `used_fallback` is true.

0 or "" in the goal uses the default from `measure` in the config.
Cancelling publishes nothing.

| Result | |
|---|---|
| `success` | The frame was measured |
| `used_fallback` | `fallback_from` was copied instead |
| `hits`, `spread_m` | Of the cluster used |
| `message` | Why it failed |

Feedback: `hits` collected so far.

### slalom_row

`class_name: slalom_row` measures the nearest row in front of the vehicle
from the red and white pipes:

| | |
|---|---|
| Origin | The red pipe |
| +X | Through the row, away from the vehicle |
| Direction | Across the line of the pipes. With only the red pipe seen: from the vehicle to it |
| Rows | Each red pipe is one row. A white pipe belongs to the nearest red one |

### Example

```bash
ros2 action send_goal /nautilus/landmark_server/measure vortex_msgs/action/MeasureLandmark \
  "{class_name: torpedo_board, fallback_from: torpedo_board}"
ros2 run tf2_ros tf2_echo nautilus/odom nautilus/torpedo_board/measured/standoff
```

## Target frames

```yaml
targets:
  standoff: {parents: ["torpedo_board", "torpedo_board/measured"], x: 2.0, yaw: 180.0}
```

This gives `torpedo_board/standoff` and `torpedo_board/measured/standoff`
whenever their parent exists. x, y, z in m and roll, pitch, yaw in degrees,
in the parent's axes. A parent can be any TF frame, also another target.
Change one while running:

```bash
ros2 param set /nautilus/landmark_server_node targets.standoff.x 1.5
```

| Target | Parents | |
|---|---|---|
| `entry`, `exit` | `gate_middle`, a measured gate panel | 1.5 m before and after the gate, 0.5 m below the panels |
| `gap_left_entry`, `gap_left_exit`, `gap_right_*` | `slalom_row/measured`, `.../next` | 1 m before and after the middle of each gap |
| `next` | `slalom_row/measured` | Where the next row should be |
| `standoff` | board, table | 2 m in front, facing it |
| `shoot` | each opening | 0.6 m in front, facing it |
| `above` | bins, table | 1 m above |
| `centre` | octagon | |
| `home` | `start` | |

The numbers are from the simulator course.

## Reset

```bash
ros2 service call /nautilus/landmark_server/reset vortex_msgs/srv/ResetLandmarks "{class_name: torpedo_board}"
```

Drops the stored hits and landmarks of the class, `""` for all. Measured
frames stay. `mission/wipe` also starts a new `map`.

## Debug mode

```bash
ros2 launch landmark_server landmark_server.launch.py debug:=true
```

| Topic | |
|---|---|
| `landmark_server/markers` | A cube or real-size box per landmark, its name, id, hits and spread, and an arrow along +X when the yaw is known. Grey when not seen for a second |
| `landmark_server/raw_detections` | Every detection as it comes in, a small sphere that lives 0.3 s |

## Prior map

```bash
ros2 launch landmark_server competition_map_gui.launch.py   # drone:=nautilus
```

1. Place the reference where the vehicle starts, pointing at the course.
2. Place each task. The slider sets the radius its detections must be within.
3. Send to Vehicle. It is saved to `config/premap.yaml`.

While the vehicle runs the GUI also shows, relative to the reference:

- crosses: what the vehicle has put in the map so far;
- dots: where each detection of the last two seconds landed, also the ones
  the map rejected;
- the arrow: the vehicle and its heading in the map.

All three are only in the right place on the drawing if the reference is
where the vehicle was when the map started. The arrow and the dots need
`map -> odom -> base_link` in TF, and the dots also the detection's frame.

## Adding a landmark

A landmark kind is a **class**: one entry under `classes` in
`config/landmark_server.yaml`. Perception keeps publishing
`vortex_msgs/LandmarkArray` with a type and subtype, and the server picks
the class with that type and subtype.

### Where to add it

| Step | File | What |
|---|---|---|
| 1 | `vortex-msgs/msg/LandmarkType.msg`, `LandmarkSubtype.msg` | The constants, if they do not exist yet |
| 2 | `vortex-utils/vortex_utils_ros/include/vortex/utils/ros/landmark_names.hpp` | The names in `kLandmarkTypes` / `kLandmarkSubtypes`, so the config can refer to them by name |
| 3 | `config/landmark_server.yaml` → `classes` | The class, see below |
| 4 | `config/landmark_server.yaml` → `markers.boxes` | Optional: a real-size box in the debug markers |
| 5 | `scripts/competition_map_gui.py` → `SERVICE_LABEL_MAP` | Only for a new task label, so it can be placed in the prior map |

Rebuild vortex-msgs, vortex-utils and landmark_server after steps 1 and 2.
A config change only needs a restart.

```yaml
classes:
  buoy: {type: BUOY, subtype: BUOY_RED, symmetry_deg: 360.0, has_orientation: false, prior: "buoy", prior_radius_m: 3.0, max_instances: 1}
```

The object is then published as the TF frame `buoy` and on
`landmark_server/landmarks`.

The server refuses to start if two classes have the same type and subtype,
or if a type or subtype name is not in `landmark_names.hpp`. Detections
whose type and subtype match no class are ignored, with one warning per
pair.

The server refuses to start if two classes have the same type and subtype,
or if a type or subtype name is not in `landmark_names.hpp`.

### Class fields

| Field | What it does | How to pick it |
|---|---|---|
| `type`, `subtype` | Names of the `LandmarkType` / `LandmarkSubtype` constants the detector sends | Must match what perception publishes |
| `symmetry_deg` | How much of the yaw is known. `0`: the full yaw. `90` / `180`: only modulo that, the yaws are averaged modulo it. `360`: no yaw, only position | The smallest turn that makes the object look the same. A pipe or buoy is `360`, a bin seen from above `180`, a board with a front `0` |
| `has_orientation` | Use the detector's orientation. Without it, or with `symmetry_deg: 360`, a detection only gives a position | `true` only if the detector gives a usable orientation. The detector must also set the orientation variances (covariance diagonal 21, 28, 35) below 1000. An all-zero covariance means no orientation |
| `prior` | Task label in the prior map, e.g. `"torpedo"`. Detections must lie near `prior_<task>`. `""` turns the gate off | Several classes can share a task. A class whose task is not in the prior map gets no gate, with a warning at startup |
| `prior_radius_m` | Horizontal radius of that gate in m. A radius set in the GUI for the task wins over this | Large enough for the error of the prior map plus the size of the task. Too small rejects the real object, too large lets false detections through |
| `max_instances` | How many landmarks of the class are published, most observed first. `0` = all | How many the course has. The frame `<class>` is always the most observed one |
| `group` | Classes that can be mistaken for each other share a group: they are clustered and voted on together. Default: the class name, alone | Give the same group when the detector mixes the classes up, e.g. red and white slalom pipes |

## Settings

All in `config/landmark_server.yaml`.

| Value | What it does | Raise it when | Lower it when |
|---|---|---|---|
| `max_range_m` | Detections farther from the vehicle are dropped | Far objects are never mapped | Far detections are noisy |
| `map.cluster_radius_m` | Hits this close to the densest one are one object | One object becomes two | Two objects of one group become one. Keep under half their spacing |
| `map.confirm_hits` | Hits a cluster needs to be a landmark | False detections become landmarks | Real objects appear late |
| `map.max_hits_per_class` | Hits kept per class, the newest | Landmarks out of view disappear | Clustering is slow |
| `map.max_hit_age_s` | Hits older than this are forgotten, 0 keeps all | | The map lags behind after a lot of drift. Landmarks out of view then disappear |
| `measure.*` | Defaults for the action | | |

### Debug box

```yaml
markers:
  boxes:
    buoy: {size: [0.2, 0.2, 0.2], solid: true, color: [1.0, 0.0, 0.0]}
```

`size` is [x out of the front, y right, z down], `offset` the box centre
from the landmark, `solid` draws the object itself instead of an outline.
Without a box the landmark is a small cube.
