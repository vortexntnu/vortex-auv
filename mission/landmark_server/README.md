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
| `landmark_server/markers` | Landmarks with uncertainty, for Foxglove |
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

## Adding a target frame

Example: a point 1 m in front of the buoy.

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
                                      const BuoyParams& buoy) {
       const LandmarkState* b = best_of(landmarks, buoy.buoy_class);
       if (!b) {
           return {};
       }
       const gtsam::Rot3 R = gtsam::Rot3::Yaw(b->pose.rotation().yaw());
       const gtsam::Point3 p =
           b->pose.translation() - R * gtsam::Point3(buoy.standoff_m, 0.0, 0.0);
       return {{"buoy_front", gtsam::Pose3(R, p)}};
   }
   ```

4. Publish it in `publish_map()`, next to the other target frames:

   ```cpp
   for (const NamedPose& g : buoy_frames(shown, cfg_.params.buoy)) {
       frames_.push_back(frame(g.name, g.pose));
   }
   ```

5. Add the values to `config/landmark_server.yaml`:

   ```yaml
   buoy:
     buoy_class: "buoy"
     standoff_m: 1.0
   ```

`gate_frames()` is a real example.

## Target frames to write

These are not written yet. Both have been tried and work.

**Slalom gaps**: one frame per row on each side of the red pipe, e.g.
`slalom_left_<n>` and `slalom_right_<n>`, with +X through the row.

- Each red pipe is one row. The white pipes belong to the nearest red one.
- The gap is between the red pipe and the white pipe on that side. Decide
  what to publish when the white pipe has not been seen yet.
- Number the rows so that a row found late does not rename the others.
- The red and white pipes share a `group` in the config, so a pipe seen
  with the wrong colour does not become a second landmark.

**Torpedo openings**: one frame per opening on the board, with +X through
the board.

- Perception gives the board centre and its normal. The openings are fixed
  offsets from the centre in the plane of the board.
- The board's X axis points out of the front, towards the vehicle.
- Keep the frames level, even if the board estimate is slightly tilted.
- Put the offsets in the config. It helps a lot if they can be changed
  while running with `ros2 param set`, so you can line them up against the
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
