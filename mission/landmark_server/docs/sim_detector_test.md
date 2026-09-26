# Testing a real detector in the simulator

How to test a perception detector (for example the slalom pipe detector)
against `landmark_server` in the Stonefish simulator: the detector provides
one kind of object, the dummy perception provides the rest, and the map is
checked against the true course.

The example throughout is the slalom detector. For another detector, swap the
task names and the class in the checks.

## Contents

1. [What the detector must publish](#1-what-the-detector-must-publish)
2. [Build and start](#2-build-and-start)
3. [Check the detector output](#3-check-the-detector-output)
4. [Open Foxglove](#4-open-foxglove)
5. [Drive past the objects](#5-drive-past-the-objects)
6. [What to look for](#6-what-to-look-for)
7. [If something is wrong](#7-if-something-is-wrong)
8. [Record a bag](#8-record-a-bag)
9. [Optional: with drift](#9-optional-with-drift)
10. [Quick reference](#10-quick-reference)

---

## 1. What the detector must publish

`landmark_server` does the tracking, the ids and the noise handling. The
detector only has to report what it sees in each image.

### Must

| # | Requirement | Why |
|---|---|---|
| 1 | Topic `/nautilus/landmarks`, type `vortex_msgs/LandmarkArray` | `landmark_server` reads it. The dummy publishes on the same topic at the same time; that is supported |
| 2 | **One message per image**, `header.stamp` copied from the image | The tracker and the graph attach the measurement to the vehicle pose at that time |
| 3 | `header.frame_id` = the camera frame of the image, and it exists in TF | Detections are transformed to `nautilus/odom` at the image time |
| 4 | `type.value = 7` (`SLALOM_PIPE`), `subtype.value` **1 = white, 2 = red** | Subtype 0 does not match the class rules |
| 5 | Position = a point on the pipe axis, preferably mid-height, always the same point | `landmark_targets` uses x and y; the tracker needs the same point every frame |
| 6 | Orientation `w = 1` and rotation variance (covariance indices 21, 28, 35) ≥ 1000 | Means "position only". A zero quaternion is discarded as invalid |
| 7 | Rate **≥ 5 Hz, preferably 10 Hz** | A track is confirmed after hits in 3 of 5 ticks of 200 ms. At 2 Hz it never is |
| 8 | No NaN; publish nothing rather than a guess | Invalid entries are dropped and counted as warnings |

### Should

- **Covariance or zeros.** Leave the position covariance at zero and
  `landmark_server` uses its own noise model (the safe default). If a
  covariance is given, it must be larger along the line of sight than across
  it and grow with distance. A covariance that is too small causes duplicate
  pipes.
- **Say how the range is found.** Depth camera (`/nautilus/depth_camera`), or
  known pipe length (white 1.27 m, red 0.94 m) and pixel height. The second
  usually has a consistent range bias, which changes how the depth noise
  should be tuned.
- **Publish with reliable QoS** if the raw detections should show up in
  Foxglove (the detection markers node subscribes reliably). The map works
  either way.

### Does not need to

- Track, smooth or keep ids between frames (`id` may be 0).
- Publish in odom or world coordinates.
- Filter by distance. Pipes farther than 7 m, and pipes within 1.5 m of the
  gate, table, torpedo board or bin rig, are dropped by `landmark_server`.

**Known trap:** the gate poles are PVC pipes too. Check that they are not
detected as slalom pipes.

---

## 2. Build and start

The detector needs camera images, so run the simulator **with rendering**
(not `--headless`).

```bash
cd ~/ros2_ws
git -C src/vortex-auv branch --show-current   # rework/isam2
git -C src/vortex-cv branch --show-current    # rework/isam2
MAKEFLAGS="-j2" nice -n 19 colcon build --packages-select vortex_msgs vortex_utils \
  pose_filtering landmark_server landmark_targets waypoint_manager robosub_dummy_publisher \
  --parallel-workers 2
source install/setup.bash

src/vortex-auv/utility_scripts/launch_sim_autonomy.sh --fov \
  --tasks gate,torpedo_board,bin,octagon,table
```

- `--tasks` leaves `slalom` out of the dummy, so every pipe in the map comes
  from the real detector. For another detector, leave its task out instead.
- `--fov` makes the dummy publish only what the cameras could see.
- The script uses course seed 7 for both the simulator and the dummy, so the
  roles agree.

tmux windows: **sim** (simulator, controller, landmark_server,
waypoint_manager), **perception** (dummy, frames), **check** (the map against
the true course), **tools** (Foxglove bridge, commands). Switch with
`Ctrl-b` and the window number. Stop everything with
`tmux kill-session -t sim_autonomy`.

Then start the detector in its own terminal (`source install/setup.bash`
first).

---

## 3. Check the detector output

Do this before looking at the map. If a row fails, nothing later works.

| Check | Command | Expect |
|---|---|---|
| It publishes | `ros2 topic info -v /nautilus/landmarks` | The detector node listed as a publisher next to the dummy |
| Rate | `ros2 topic hz /nautilus/landmarks` | Higher than with the dummy alone; the detector's part ≥ 5 Hz |
| Content | `ros2 topic echo /nautilus/landmarks --once` | `type.value: 7`, `subtype.value: 1` or `2`, rotation variance ≥ 1000, non-empty `frame_id` |
| Frame exists | `ros2 run tf2_ros tf2_echo nautilus/odom <frame_id>` | A transform, not an error |
| Stamps | Compare `header.stamp` with the image's stamp | The same time, not the publish time |
| Map config | landmark_server pane, line starting `LandmarkServer config:` | `z_lock on (floor 3.432 m)` (sim values) |

---

## 4. Open Foxglove

1. Open Foxglove and connect to `ws://localhost:8765`.
2. **Layout → Import from file**:
   `src/vortex-auv/mission/landmark_server/foxglove/landmark_graph.json`.

| In the 3D view | Meaning |
|---|---|
| Green transparent spheres | The truth: where the objects really are |
| Pipes, boxes, cubes | The map (what the vehicle navigates by) |
| Blue lines | Error of the map: from each map object to its true position |
| Small dots | Raw detections right now |
| Cyan arrow | The goal the controller steers to |

The slalom pipes are in three rows at x ≈ 8.1, 10.1 and 12.1 (the course in
the simulator is rotated 90°), spread across y.

---

## 5. Drive past the objects

In the **tools** window:

1. **Stand still** in front of the slalom at about 5 m (send a waypoint, or
   drive with the joystick). Pipes should appear in the map about 1 s after
   they come into view.
2. **Drive a loop** past the slalom and back (it passes beside the pipes
   along y = −3, without touching them):
   ```bash
   ros2 run landmark_server drift_route.py
   ```
3. **Run the slalom scenario**, starting in front of the first row (pipes
   within 7 m):
   ```bash
   ros2 run landmark_targets landmark_targets_scenario_node --ros-args \
     -r __ns:=/nautilus -p scenario:=slalom
   ```
   It ends with `SCENARIO SUCCESS` (exit 0) or `SCENARIO FAILURE` (exit 1).

---

## 6. What to look for

| Look at | Good | Problem |
|---|---|---|
| Pipe count (plot *Antall landemerker*, or `object_map`) | 6 white, 3 red, stable | More: duplicates. Fewer: pipes not confirmed |
| Blue lines on the pipes | Short, under about 0.2 m | Long: position error in the detector or its TF |
| Colours | Red map pipes on the red truth spheres | A colour mix-up puts a map pipe far from any sphere of its colour |
| Ids in the labels | The same number the whole run | Changing: pipes lost and recreated |
| Near the gate | No pipes at the gate poles | Gate poles detected as pipes |
| While the vehicle turns | Pipes stay where they are | Pipes swing with the turn: stamp or TF problem |
| Check window (`graph_eval`) | `all` error for the map under about 0.2 m | Larger: look at which objects in Foxglove |

---

## 7. If something is wrong

| Symptom | Likely cause | Try |
|---|---|---|
| No pipes in the map | Detector slower than 3 of 5 ticks, wrong subtype, frame missing in TF, pipes > 7 m away | Section 3. Tentative tracks show in `/nautilus/landmark_server/live_tracks` |
| Duplicate pipes at long range | The 0.5 m hard gate is smaller than the detector's depth noise | `-p track_config.default.gate.max_pos_error:=1.0` |
| Duplicate pipes everywhere | The detector's covariance is too small (if given) | Leave it at zero, or `-p intake.measurement_covariance.use:=false` |
| Pipes at the gate poles | Detector confusion | `-p classes.SLALOM_PIPE.min_distance_to_large_structures_m:=2.0` |
| Pipes forgotten about 15 s after leaving view | Retention (normal) | Kept for the whole run after 30 observations |
| Pipes swing when the vehicle turns | Stamp or camera TF | Fix in the detector or TF, not with parameters |

**Restart landmark_server with a parameter:** in the landmark_server pane of
the **sim** window, stop it with `Ctrl-c` and run:

```bash
C=install/landmark_server/share/landmark_server/config
ros2 run landmark_server landmark_server_node --ros-args -r __ns:=/nautilus \
  --params-file $C/landmark_server_config.yaml --params-file $C/sim.yaml \
  --params-file install/auv_setup/share/auv_setup/config/robots/nautilus.yaml \
  -p track_config.default.gate.max_pos_error:=1.0
```

The map starts empty after a restart. In the simulator, only change the
parameters that depend on how the detector behaves (`max_pos_error`,
`track_config.<CLASS>.nm`, `min_distance_to_large_structures_m`). The noise
values are measured in the pool (see the tuning guide).

---

## 8. Record a bag

```bash
ros2 bag record -s mcap -o sim_slalom_detector \
  /nautilus/landmarks /nautilus/odom /nautilus/pose /tf /tf_static /nautilus/front_camera
```

With it, the detector's real noise and bias per distance can be measured, and
settings can be tried by replaying the bag, without running the simulator
again. Write down which part of the run was which (standing still at 5 m,
the loop, the scenario).

---

## 9. Optional: with drift

**Do not combine `--drift` with a real detector yet.** With `--drift`, the
map runs on odometry that drifts on purpose, and the dummy's detections are
shifted to match. A real detector's detections are transformed through the
camera TF, which the simulator computes from the true pose, so they do not
drift. The map then gets two pictures that disagree, and the graph can be
pulled the wrong way. The result says nothing about the detector.

To see the drift correction work, run it with the dummy only (no `--tasks`):

```bash
src/vortex-auv/utility_scripts/launch_sim_autonomy.sh --headless --drift 0.5
ros2 run landmark_server drift_route.py
```

In Foxglove, red lines are the error without the graph, blue with it. After
the vehicle sees the gate again, the blue lines should be short and the red
ones long.

---

## 10. Quick reference

```bash
# start (rendering, dummy without slalom)
src/vortex-auv/utility_scripts/launch_sim_autonomy.sh --fov --tasks gate,torpedo_board,bin,octagon,table

# checks
ros2 topic info -v /nautilus/landmarks
ros2 topic hz /nautilus/landmarks
ros2 topic echo /nautilus/landmarks --once
ros2 run tf2_ros tf2_echo nautilus/odom <frame_id>

# drive
ros2 run landmark_server drift_route.py
ros2 run landmark_targets landmark_targets_scenario_node --ros-args -r __ns:=/nautilus -p scenario:=slalom

# record
ros2 bag record -s mcap -o sim_slalom_detector /nautilus/landmarks /nautilus/odom /nautilus/pose /tf /tf_static /nautilus/front_camera

# stop
tmux kill-session -t sim_autonomy
```

More background: `mission/landmark_server/README.md`, and the Landmark Server
Handbook (how the tracker, map and graph work, every parameter, tuning).
