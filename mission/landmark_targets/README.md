# landmark_targets

The library BT nodes use to turn map landmarks into `WaypointManager` goals. The core has no ROS dependency (gtest); a thin ROS part converts `LandmarkTrack` and reads TF.

```
landmark_server/object_map ─▶ MapLandmark ─▶ LandmarkTarget::step(landmark, odom, now)
                                              ├─ send_goal  ─▶ WaypointManager goal (odom pose for base_link)
                                              └─ phase: TRACKING | DEAD_RECKONING | LOST
```

## Conventions

| | |
|---|---|
| Landmark frame | Origin in the object, +X out of the front, +Z down (NED) |
| `LANDMARK` offset | In the landmark frame: x = 2 is 2 m in front, yaw π looks at the object, negative x is behind. Needs `has_orientation` |
| `LANDMARK_ODOM_AXES` | Offset along the odom axes (z < 0 above). For objects without orientation. Same as `apply_pose_offset` |
| Tool arm | The target position applies to the tool (launcher, dropper, camera): `p_base = p_target - R(q_target) * t_base→tool`; the orientation applies to `base_link`. Look it up with `lookup_tool_arm` |
| Freeze | The target is computed once (after a commit) |
| Dead reckoning | Within `dead_reckoning_distance` the node stops updating; reference filter and controller hold the last goal |
| Track loss | Landmark not seen for `track_loss_timeout_sec` before dead reckoning → `LOST`, the node fails and the tree picks a fallback |
| New goal | Only when the target moved more than `resend_distance` (5 cm) and at most every `min_resend_interval_sec` |

## API

```cpp
#include "landmark_targets/landmark_target.hpp"

TargetSpec spec;
spec.frame  = OffsetFrame::LANDMARK;
spec.offset = pose_with_yaw(2.0, 0.0, 0.0, M_PI);       // 2 m in front, looking at it
spec.tool_arm = *lookup_tool_arm(tf, "base_link", "launcher_link");
LandmarkTarget target(spec, landmark_id);                // lock the id from the map

auto s = target.step(map.find(landmark_id), odom, now);  // every tick
if (s.phase == Phase::LOST) { /* cancel goal, FAILURE */ }
if (s.send_goal)            { /* send WaypointManager goal(s) with hold and tolerances */ }
```

Geometry (`geometry.hpp`): `forward_distance`, `side_of`, `perpendicular_heading`, `CourseFrame` with `to_course` / `from_course`. `course_frame_from_tf` reads TF `nautilus/course` and returns `nullopt` while the frame is `UNSET`, so no node computes in a frame that does not exist.

Slalom (`slalom.hpp`): `match_pipes` (nearest red pipe in front and not passed, best white pair with the red one on the line, gap on the gate side, heading perpendicular to the whites turned inward, one-white fallback with mirroring, shortcut with the previous offset) and `avoid_slalom_waypoints` (three waypoints in the course frame that take the vehicle out of the field, past it and back in front of the gate).

## Example

`examples/scenario_node.cpp` runs a scenario the way BT nodes would, against `landmark_server` and `waypoint_manager`:

```bash
ros2 run landmark_targets landmark_targets_scenario_node \
  --ros-args -r __ns:=/nautilus -p scenario:=gate      # gate | torpedo | bin | slalom | return_home
```

It ends with `SCENARIO SUCCESS` / `SCENARIO FAILURE` and the matching exit code. `ApproachStep` corresponds to `ApproachLandmark`, `MoveRelativeStep` to `MoveRelative`, `SlalomStep` to the slalom layers and `return_home` to `AvoidSlalom` + approach from behind.
