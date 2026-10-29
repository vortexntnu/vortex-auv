# landmark_server

Takes object detections from perception and builds a map of the course. The
mission reads the map as TF frames and drives to them.

## Core concepts

- **Landmark**: one object in the map, built from many detections.
- **Class**: a kind of object, e.g. `torpedo_board`. Listed under `classes`
  in `config/landmark_server.yaml`. Detections of other types are ignored.
- **Map frame**: where the vehicle was at startup. `map -> odom` corrects
  odometry drift.
- **Prior map**: roughly where each task is in the pool, drawn in the GUI.
  Gives a `prior_<task>` frame to search from and rejects detections far
  from their task.
- **Target frame**: a frame that is not an object itself, e.g. the gap
  between two slalom pipes.

## Run

```bash
ros2 launch landmark_server landmark_server.launch.py            # pool
ros2 launch landmark_server landmark_server.launch.py env:=sim   # simulator
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

## Topics

| Topic | Direction | Type |
|---|---|---|
| `odom` | in | `nav_msgs/Odometry` |
| `landmarks` | in | `vortex_msgs/LandmarkArray` |
| `mission/wipe` | in | `std_msgs/Empty` |
| `mission/set_heading` | in | `std_msgs/Empty` |
| `landmark_server/landmarks` | out | `vortex_msgs/LandmarkTrackArray` |

Perception publishes detections on `landmarks`. To use the map, read
`landmark_server/landmarks` or look up the TF frames below.

## TF frames

| Frame | |
|---|---|
| `<class>` | The object, e.g. `torpedo_board` |
| `prior_<task>` | Where the prior map puts the task |
| `start` | Where the run started |
| `gate_middle`, `<panel>_entrance`, `<panel>_exit` | Gate, from the two role panels |

For target frames +X is the direction to drive or face.

With only one gate panel in the map there is no `gate_middle`, and that
panel's entrance and exit point along the line from `start` to the panel.
That is only right when the run starts in front of the gate.

## Debug mode

```bash
ros2 launch landmark_server landmark_server.launch.py debug:=true
```

Also publishes:

| Topic | |
|---|---|
| `landmark_server/markers` | The map for Foxglove: a cube or real-size box per landmark, its name, id and uncertainty, and an arrow along +X when the yaw is known. Grey when not seen lately |
| `landmark_server/nis` | Should be near 1. Higher means the `detection` noise in the config is too low |
| `landmark_server/raw_detections` | Every detection as it comes in, a small sphere in the colour of its type that lives 0.3 s. Missed detections show as flicker next to the steady map |

Off by default.

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
`config/landmark_server.yaml`. Perception does not change. It keeps
publishing `vortex_msgs/LandmarkArray` with a type and subtype, and the
server picks the class with that type and subtype.

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

### How a detection becomes a landmark

```
detection
  │  type/subtype not a class ─────────────────────────────▶ ignored
  │  range > max_range_m ──────────────────────────────────▶ dropped
  ▼
match against landmarks of the same group (chi² gate) ──▶ matched: refines that landmark
  │ unmatched
  ▼
prior gate: farther than the task radius from prior_<task> ──▶ rejected
  ▼
candidate: within candidate_radius_m of earlier unmatched hits
  │  confirm_hits within confirm_window_s
  ▼
new landmark, unless one of the group is within merge_radius_m
  ▼
shown: the max_instances most observed of the class
```

- The prior gate only stops **new** landmarks. Once a landmark exists,
  matched detections refine it wherever it ends up.
- The class of a landmark is a majority vote over its detections. Within a
  `group`, a landmark first seen as one class can become another.
- Same-group landmarks that end up within `merge_radius_m` are merged, e.g.
  a copy made while the odometry had drifted.
- `max_instances` only limits what is published. Extra landmarks stay in
  the map and can take over if they get more detections.

### Class fields

| Field | What it does | How to pick it |
|---|---|---|
| `type`, `subtype` | Names of the `LandmarkType` / `LandmarkSubtype` constants the detector sends | Must match what perception publishes |
| `symmetry_deg` | How much of the yaw is known. `0`: the full yaw. `90` / `180`: only modulo that, each detection is snapped to the symmetric yaw closest to the estimate. `360`: no yaw, only position | The smallest turn that makes the object look the same. A pipe or buoy is `360`, a bin seen from above `180`, a board with a front `0` |
| `has_orientation` | Use the detector's orientation. Without it, or with `symmetry_deg: 360`, a detection only gives bearing and range | `true` only if the detector gives a usable orientation. The detector must also set the orientation variances (covariance diagonal 21, 28, 35) below 1000, else the orientation is ignored anyway |
| `prior` | Task label in the prior map, e.g. `"torpedo"`. New landmarks must lie near `prior_<task>`. `""` turns the gate off | Several classes can share a task. A class whose task is not in the prior map gets no gate, with a warning at startup |
| `prior_radius_m` | Horizontal radius of that gate in m. A radius set in the GUI for the task wins over this | Large enough for the error of the prior map plus the size of the task. Too small rejects the real object, too large lets false detections through |
| `max_instances` | How many landmarks of the class are published, most observed first. `0` = all | How many the course has. The frame `<class>` is always the most observed one |
| `group` | Classes that can be mistaken for each other share a group: they are matched, merged and voted on together. Default: the class name, alone | Give the same group when the detector mixes the classes up, e.g. red and white slalom pipes |

### Global values that decide what gets in

All in `config/landmark_server.yaml`, shared by every class.

| Value | What it does | Raise it when | Lower it when |
|---|---|---|---|
| `max_range_m` | Detections farther away are dropped | Far objects are never mapped | Far detections are noisy |
| `new_landmarks.confirm_hits` | Detections a candidate needs to become a landmark | False detections become landmarks | Real objects appear late |
| `new_landmarks.confirm_window_s` | Time those detections must fall in | The detector sees the object only now and then | Brief false detections pile up |
| `new_landmarks.candidate_radius_m` | How close a detection must be to a candidate to count for it | Hits of one object start several candidates | Two objects of one group are closer than twice this |
| `upkeep.merge_radius_m` | Landmarks of one group this close become one | Duplicates stay | Real objects of one group are merged |
| `association.gate_prob` | How far, in sigmas, a detection may lie from a landmark and still match | Detections of a known object start new candidates | Detections match the wrong object |
| `association.ambiguity_d2` | A match is dropped when a second landmark is almost as close | Objects next to each other steal detections | Detections near two objects are wasted |
| `detection.*` | Detection noise. Larger trusts each detection less | NIS well above 1 | NIS well below 1, map slow to settle |
| `detection.orientation_yaw_sigma` | Noise of the detector's yaw | The landmark's yaw jumps | Yaw follows the detector too slowly |

`landmark_server/nis` (debug mode) is the check for the `detection` values.
Every rejected detection shows as a dot in the GUI and in
`landmark_server/raw_detections`. A class that keeps hitting the prior gate
gives a warning with how far its nearest detection was.

### Debug box

```yaml
markers:
  boxes:
    buoy: {size: [0.2, 0.2, 0.2], solid: true, color: [1.0, 0.0, 0.0]}
```

`size` is [x out of the front, y right, z down], `offset` the box centre
from the landmark, `solid` draws the object itself instead of an outline.
Without a box the landmark is a small cube.

## Target frames

A target frame is a place to go that is not an object itself: the gap
between two pipes, a point in front of a board. It is computed from the
landmarks every time the map is published.

```
landmarks ─▶ <task>_frames(landmarks, params) ─▶ NamedPose[] ─▶ TF map -> <name>
                      │
                      └─ returns nothing while the landmarks it needs are missing
```

### Conventions

| | |
|---|---|
| Frame | In `map`. +X is the way to drive or face, +Y right, +Z down |
| Level | Yaw only. Roll and pitch are zero even if the landmark is tilted |
| Offsets | The mission adds its own offset in the frame: x = -1 is 1 m before it, x = 1 is 1 m past it |
| Missing landmarks | Return an empty list. The frame then does not exist and the mission waits or falls back |
| Names | Fixed and predictable, e.g. `<panel>_entrance`. The mission config refers to them by name |
| Numbers | In `config/landmark_server.yaml`, not in the code |
| Landmark frame | Origin in the object. For objects with a front (the torpedo board) +X points out of the front |

### API

```cpp
#include "landmark_server/targets.hpp"

struct NamedPose {
    std::string name;
    gtsam::Pose3 pose;   // in map
};

// Most observed landmark of a class, or nullptr.
const LandmarkState* best_of(const std::vector<LandmarkState>& landmarks,
                             const std::string& cls);

// What a landmark gives you.
landmark->pose.translation();        // position in map
landmark->pose.rotation().yaw();     // only meaningful if landmark->yaw_known
landmark->n_obs;                     // number of detections
landmark->cls.name;                  // class name from the config
```

### Example

A frame 1 m in front of a buoy, facing it.

1. Params in `config.hpp`, plus a `BuoyParams buoy;` member in `Params`:

   ```cpp
   struct BuoyParams {
       std::string buoy_class;
       double standoff_m{1.0};
   };
   ```

2. Read them in `load_config()` in `landmark_server_node.cpp`:

   ```cpp
   read("buoy.buoy_class", p.buoy.buoy_class);
   read("buoy.standoff_m", p.buoy.standoff_m);
   ```

3. The function in `targets.cpp`, declared in `targets.hpp`:

   ```cpp
   std::vector<NamedPose> buoy_frames(const std::vector<LandmarkState>& landmarks,
                                      const BuoyParams& buoy,
                                      const gtsam::Point3& start) {
       const LandmarkState* b = best_of(landmarks, buoy.buoy_class);
       if (!b) {
           return {};
       }
       // A buoy has no front, so face it from the start side.
       const gtsam::Point3 to_buoy = b->pose.translation() - start;
       const double yaw = std::atan2(to_buoy.y(), to_buoy.x());
       const gtsam::Rot3 R = gtsam::Rot3::Yaw(yaw);
       const gtsam::Point3 p =
           b->pose.translation() - R * gtsam::Point3(buoy.standoff_m, 0.0, 0.0);
       return {{"buoy_front", gtsam::Pose3(R, p)}};
   }
   ```

4. Publish it in `publish_map()`, next to the gate frames:

   ```cpp
   for (const NamedPose& g : buoy_frames(shown, cfg_.params.buoy,
                                         graph_.keyframe_pose(0).translation())) {
       frames_.push_back(frame(g.name, g.pose));
   }
   ```

5. Add the values to `config/landmark_server.yaml`:

   ```yaml
   buoy:
     buoy_class: "buoy"
     standoff_m: 1.0
   ```

The mission can now use `GoToFrame frame="buoy_front"`.

`gate_frames()` in `targets.cpp` is a real one: it builds five frames from
the two gate panels, or two frames when only one panel is mapped.

### To write

| Target | Owner | Frames | Built from |
|---|---|---|---|
| Slalom gaps | André | `slalom_left_<n>`, `slalom_right_<n>`, +X through the row | Red and white pipes |
| Torpedo openings | Johannes | `torpedo_opening_<name>`, +X through the board | Board centre and normal |

Slalom hints:
- Each red pipe is one row. The white pipes belong to the nearest red one.
- The gap is between the red pipe and the white pipe on that side. Decide
  what to publish when the white pipe has not been seen yet.
- Number the rows so that a row found late does not rename the others.
- The row's direction comes from the line through its pipes.

Torpedo hints:
- The openings are fixed offsets from the board centre, in the plane of the
  board.
- The board's +X points out of the front, towards the vehicle. The opening
  frame should point the other way.
- Put the offsets in the config. Being able to change them while running
  with `ros2 param set` makes them much easier to line up against the
  camera image.
- Which opening is ours depends on the board version.

## Tuning

Run with `debug:=true` and change one value at a time.

| Symptom | Change |
|---|---|
| NIS above 1 | Raise `detection.*` sigmas |
| Duplicates after a loop | Raise `odom.sigma_*_per_m` |
| False detections become landmarks | Raise `confirm_hits` or lower the task radius |
| Real objects appear late | Lower `confirm_hits` |
| Object rejected near its task | Raise the task radius or fix the prior map |
