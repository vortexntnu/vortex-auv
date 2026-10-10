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

Reset the map with the vehicle at the start, facing the course:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

## Topics

| Topic | Direction | Type |
|---|---|---|
| `odom` | in | `nav_msgs/Odometry` |
| `landmarks` | in | `vortex_msgs/LandmarkArray` |
| `mission/wipe` | in | `std_msgs/Empty` |
| `landmark_server/landmarks` | out | `vortex_msgs/LandmarkTrackArray` |

Perception publishes detections on `landmarks`. To use the map, read
`landmark_server/landmarks` or look up the TF frames below.

## TF frames

| Frame | |
|---|---|
| `<class>` | The object, e.g. `torpedo_board` |
| `prior_<task>` | Where the prior map puts the task |
| `start` | Where the run started |
| `gate_middle`, `<panel>_entrance`, `<panel>_exit` | Gate |

For target frames +X is the direction to drive or face.

## Debug mode

```bash
ros2 launch landmark_server landmark_server.launch.py debug:=true
```

Also publishes:

| Topic | |
|---|---|
| `landmark_server/markers` | The map for Foxglove: a cube or real-size box per landmark, its name, id and uncertainty, and an arrow along +X when the yaw is known. Grey when not seen lately |
| `landmark_server/nis` | Should be near 1. Higher means the `detection` noise in the config is too low |

Off by default.

## Prior map

```bash
ros2 run landmark_server competition_map_gui.py --ros-args -r __ns:=/nautilus
```

1. Place the reference where the vehicle starts, pointing at the course.
2. Place each task. The slider sets the radius its detections must be within.
3. Send to Vehicle. It is saved to `config/premap.yaml`.

Crosses show what the vehicle has found so far.

## Adding an object

1. Add the constants to `LandmarkType.msg` / `LandmarkSubtype.msg` in
   vortex-msgs.
2. Add the names to the tables in `src/config.cpp`.
3. Add a class in `config/landmark_server.yaml`:

   ```yaml
   buoy: {type: BUOY, subtype: BUOY_RED, symmetry_deg: 360.0, has_orientation: false, prior: "buoy", prior_radius_m: 3.0, max_instances: 1}
   ```

4. Add `"buoy": ["buoy"]` to `SERVICE_LABEL_MAP` in
   `scripts/competition_map_gui.py` to place it in the prior map.

The object is now published as the frame `buoy`.

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
the two gate panels.

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
