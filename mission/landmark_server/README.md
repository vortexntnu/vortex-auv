# landmark_server

Maps the course objects from detections and odometry, publishes `map -> odom`
and the TF frames the mission navigates to.

Keyframes and landmarks are in one iSAM2 graph. Detections are matched to
landmarks per class (Mahalanobis gate, Hungarian). Unmatched detections
become a landmark after `confirm_hits` hits. Landmarks are kept for the whole
run. The map frame is the vehicle pose at startup or at `mission/wipe`.

## Run

```bash
ros2 launch landmark_server landmark_server.launch.py            # pool
ros2 launch landmark_server landmark_server.launch.py env:=sim   # sim.yaml + premap_sim.yaml
```

Reset the map with the vehicle at the start, facing the course:

```bash
ros2 topic pub --once /nautilus/mission/wipe std_msgs/msg/Empty
```

Launch arguments: `env`, `config_file`, `premap_file`, `odom_topic`,
`landmarks_topic`.

## Interfaces

| Name | Type | |
|---|---|---|
| `odom` | sub, `nav_msgs/Odometry` | |
| `landmarks` | sub, `vortex_msgs/LandmarkArray` | Rotation variance >= 1000 means position only |
| `mission/wipe` | sub, `std_msgs/Empty` | New map at the current pose |
| `landmark_server/landmarks` | pub, `vortex_msgs/LandmarkTrackArray` | |
| `landmark_server/markers` | pub, `MarkerArray` | |
| `landmark_server/nis` | pub, `Float64` | Should be near 1 |
| `landmark_server/set_premap` | srv, `vortex_msgs/SetPremap` | Saved to `premap_file` |
| `landmark_server/get_premap` | srv, `std_srvs/Trigger` | |

## TF frames

Children of `map`.

| Frame | |
|---|---|
| `odom` | |
| `start` | Where the run started |
| `prior_<task>` | From the prior map |
| `<class>` | Most observed landmark of the class |
| `gate_middle`, `<panel>_entrance`, `<panel>_exit` | +X through the gate |
| `slalom_left_<n>`, `slalom_right_<n>` | Pass point of row n, +X through the row |
| `torpedo_opening_<name>` | +X through the board |

## Prior map

Rough position of each task. A class with a `prior` only gets landmarks
within the task's radius.

```bash
ros2 run landmark_server competition_map_gui.py --ros-args -r __ns:=/nautilus
```

Place the reference at the start pose, place the tasks, Send to Vehicle. The
dashed circle is the task's radius, set with the slider for the selected
task. Without one the classes' `prior_radius_m` is used. Crosses are
landmarks the vehicle has found.

```yaml
reference_frame: start
objects:
  torpedo: {position: [17.0, -5.2, 2.5], orientation: [0.0, 0.0, 1.0, 0.0], radius: 3.0}
```

## Adding an object

1. Add the constants to `LandmarkType.msg` / `LandmarkSubtype.msg` (vortex-msgs).
2. Add the names to the tables in `src/config.cpp`.
3. Add a class in `config/landmark_server.yaml`:

   ```yaml
   buoy: {type: BUOY, subtype: BUOY_RED, symmetry_deg: 360.0, has_orientation: false, prior: "buoy", prior_radius_m: 3.0, max_instances: 1}
   ```

4. If it has a `prior`, add the label to `SERVICE_LABEL_MAP` in
   `scripts/competition_map_gui.py`.

It is then published as the frame `buoy`.

## Adding a target frame

For targets that are not an object, like a gap or an opening.

1. Params struct in `config.hpp`, member in `Params`.
2. Read it in `load_config()` in `landmark_server_node.cpp`.
3. Function in `targets.cpp` returning `NamedPose`s. Return nothing while the
   landmarks it needs are missing. +X is the driving direction.
4. Call it in `publish_map()`.
5. Add the values to `config/landmark_server.yaml`.

See `torpedo_frames()`. Its offsets can be changed while running:

```bash
ros2 param set /nautilus/landmark_server_node torpedo.openings.large_left "[-0.21, -0.064]"
```

## Tuning

| Symptom | Change |
|---|---|
| NIS above 1 | Raise `detection.*` sigmas |
| NIS below 1 | Lower `detection.*` sigmas |
| Duplicates after a loop | Raise `odom.sigma_*_per_m` |
| Map wobbles between landmarks | Lower `odom.sigma_*_per_m` |
| False detections become landmarks | Raise `confirm_hits`, lower `max_range_m` or `prior_radius_m` |
| Real objects appear late | Lower `confirm_hits` |
| Object rejected near its task | Raise the radius or fix the prior map |

## Simulator

```bash
src/vortex-auv/utility_scripts/launch_drone_sim.sh --headless --detach
ros2 launch landmark_server landmark_server.launch.py env:=sim
ros2 launch robosub_dummy_publisher robosub_dummy_publisher.launch.py profile:=realistic seed:=7
```
