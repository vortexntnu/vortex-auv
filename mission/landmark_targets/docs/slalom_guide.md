# Slalom: building the target yourself

A guide for whoever owns the slalom task. It explains what the slalom target
has to do, walks through how the existing solution (`match_pipes` in
`slalom.hpp` / `slalom.cpp`) works so you can write your own, and shows how to
plug it into the behavior tree in `robosub_mission`.

The existing code and its tests are your answer key. Try each step yourself
first, then compare.

## Contents

1. [The task](#1-the-task)
2. [What you get and what you return](#2-what-you-get-and-what-you-return)
3. [Conventions that will bite you](#3-conventions-that-will-bite-you)
4. [The algorithm, step by step](#4-the-algorithm-step-by-step)
5. [Build it yourself: suggested order](#5-build-it-yourself-suggested-order)
6. [Doing all three layers](#6-doing-all-three-layers)
7. [Into the behavior tree](#7-into-the-behavior-tree)
8. [Testing](#8-testing)
9. [Files](#9-files)

---

## 1. The task

Three layers of pipes across the course. Each layer is a red pipe with a
white pipe on each side. Pass every layer **on the same side of the red pipe
as the gate half you chose** (left or right), at the right depth. Hitting a
pipe or passing on the wrong side costs points.

So for each layer the job is: find the gap between the red pipe and the white
pipe on our side, drive through its middle, and move on to the next layer.

## 2. What you get and what you return

**In** (all from `landmark_server/object_map`, in the odom frame):

| Input | Type | Notes |
|---|---|---|
| Red pipes | `std::vector<Pipe>` (`id`, `position`) | Subtype `SLALOM_PIPE_RED` (2). Position only, no orientation |
| White pipes | `std::vector<Pipe>` | Subtype `SLALOM_PIPE_WHITE` (1) |
| Vehicle pose | `vortex::utils::types::Pose` | From odometry |
| Gate side | `Side::LEFT` / `Side::RIGHT` | Decided at the gate (which panel we passed) |
| Passed red pipes | `std::vector<int>` | Map ids of red pipes already behind us |
| Known offset | `std::optional<SlalomOffset>` | Gap relative to the red pipe from the previous layer |

**Out**: `std::optional<PipeGap>`

| Field | Meaning |
|---|---|
| `position` | Gap centre (odom x, y) |
| `heading` | Yaw to pass the gap with |
| `red_id` | Map id of this layer's red pipe (to mark it passed) |
| `offset_from_red` | `position - red`, reused for the next layer |

`nullopt` means "no layer found": the tree then searches or passes blind.

Keep the function **ROS-free**: plain structs in, plain structs out. That is
what makes it testable with gtest and reusable from the tree.

## 3. Conventions that will bite you

- **NED:** x forward, y **right**, z down. A vehicle at the origin facing +x
  has **left at −y**. The tests check this (`gap_left.y() == -1`).
- **Yaw:** positive yaw turns right (clockwise seen from above).
- **Map ids are stable, track ids are not.** Use the ids from `object_map`
  for "passed" pipes.
- **Pipes behind you still exist in the map.** They are remembered, so you
  have to filter them out yourself (in front of the vehicle, not passed).
- **The course can be rotated.** Never assume the layers run along odom x;
  work relative to the vehicle heading and the pipes themselves. There is a
  test for a course rotated 90°.

Helpers already in `geometry.hpp`: `forward_distance(vehicle, point)`,
`side_of(vehicle, point)`, `perpendicular_heading(a, b, current_yaw)`.

## 4. The algorithm, step by step

This is how `match_pipes` does it. Each step has the reason, because the
reasons matter more than the code.

1. **Candidate red pipes:** at least `min_forward_m` (1 m) in front of the
   vehicle and not in `passed_red_ids`.
   *Why:* the layer you just passed is still in the map, right behind you.
2. **Reference:** the nearest candidate. None → return `nullopt`.
   *Why:* the nearest red pipe ahead is the next layer.
3. **Shortcut:** if `known_offset` is set, the gap is `red + offset` with the
   previous heading. Done.
   *Why:* the three layers are built alike. Once one layer is solved, the next
   only needs its red pipe. This also works when the white pipes are hard to
   see.
4. **White candidates:** white pipes at least `min_white_from_red_m` (0.5 m)
   from the red one.
   *Why:* a white detection on top of the red pipe is noise or a
   misclassification.
5. **Best white pair:** for every pair (a, b), project the red pipe onto the
   line a→b: `t = (red − a)·(b − a) / |b − a|²`. Keep pairs where the red pipe
   lies between them (`0.1 ≤ t ≤ 0.9`) and close to the line (at most
   `max_line_distance_m`, 0.5 m). Take the pair with the red pipe closest to
   the line.
   *Why:* this picks the whites of **this** layer, not white pipes from the
   next layer that happen to be close.
6. **Left and right:** sort the pair into left and right with `side_of`
   relative to the vehicle heading. If both are on the same side (you are
   off to one side), order them by lateral position instead.
7. **The gap:** midpoint between the red pipe and the white pipe on the gate
   side. Heading: perpendicular to the white-white line (the one closest to
   the current heading), turned `inward_deg` (15°) towards the red pipe.
   *Why perpendicular:* you pass the layer straight through. *Why inward:* it
   keeps the vehicle away from the white pipe, which is the one you are most
   likely to clip.
8. **Only one white pipe seen:** if it is on the gate side, the gap is the
   midpoint. If it is on the wrong side, mirror it through the red pipe:
   `red − (white − red) / 2`.
   *Why:* the gap is on our side even if we only see the other white pipe.

## 5. Build it yourself: suggested order

Write your own function next to the existing one (for example
`match_pipes_v2`), and point the existing tests at it one by one.

| Step | Make this test pass (`test/test_slalom.cpp`) |
|---|---|
| Steps 1, 2, 5, 6, 7 | `GapBetweenRedAndTheWhiteOnTheGateSide` |
| Heading relative to the vehicle, not odom | `CourseRotated90Degrees` |
| Step 1 filtering | `PassedRedPipesAndPipesBehindAreIgnored` |
| Step 3 | `ShortcutUsesThePreviousOffset` |
| Step 4 | `WhitePipesCloseToTheRedOneAreIgnored` |
| Step 8 | `OneWhitePipeOnTheRightSideGivesTheMidpoint`, `OneWhitePipeOnTheWrongSideIsMirrored` |
| Step 5 (line test) | `RedPipeMustLieOnTheLineBetweenTheWhitePairs` |

Then write tests of your own for what the existing ones don't cover, for
example: two layers visible at once, a missing red pipe, noisy positions
(±0.2 m).

```bash
cd ~/ros2_ws
colcon build --packages-select landmark_targets
colcon test --packages-select landmark_targets && colcon test-result --verbose
```

Things to consider improving once yours matches:

- What if the nearest red pipe belongs to a neighbouring lane? (The lane
  limits in `landmark_server` already keep most of those out of the map.)
- Should the shortcut be checked against white pipes when they are visible?
- Is 15° inward right for our vehicle's width?

## 6. Doing all three layers

`match_pipes` solves one layer. The loop around it (in the tree, or in
`examples/scenario_node.cpp` `SlalomStep` as a reference) is:

```
for layer in 1..3:
    gap = match_pipes(red, white, pose, gate_side, passed, offset)
    if gap:
        go to (gap.position, gap.heading) at slalom depth, hold 1 s
        move 1 m straight ahead (body frame)      # through the layer
        passed += gap.red_id
        offset = (gap.offset_from_red, gap.heading)
    else:
        scan / wait a little; if still nothing: 2 m straight ahead (blind layer)
```

A blind layer does not update `offset`: it teaches nothing about where the
gap is.

## 7. Into the behavior tree

The tree lives in `vortex-cv/mission/robosub/robosub_mission`
(BehaviorTree.CPP v4). Your function becomes a node there; the tree does the
looping, searching and fallbacks.

### 7.1 The node

A synchronous action that reads the map and pose from the blackboard and
writes the gap. It calls the same ROS-free function.

```cpp
// robosub_mission/include/robosub_mission/nodes/match_pipes.hpp
#include <behaviortree_cpp/action_node.h>
#include <landmark_targets/slalom.hpp>
#include <vortex_msgs/msg/landmark_track_array.hpp>

class MatchPipes : public BT::SyncActionNode {
   public:
    using BT::SyncActionNode::SyncActionNode;

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<vortex_msgs::msg::LandmarkTrackArray>("map"),
            BT::InputPort<vortex::utils::types::Pose>("pose"),
            BT::InputPort<std::string>("gate_side"),          // "left" / "right"
            BT::InputPort<std::vector<int>>("exclude"),       // passed red ids
            BT::BidirectionalPort<std::optional<vortex::mission::SlalomOffset>>("offset"),
            BT::OutputPort<vortex::utils::types::Pose>("gap_pose"),
            BT::OutputPort<int>("red_id"),
        };
    }

    BT::NodeStatus tick() override {
        // 1. read the ports; split the map into red and white Pipe vectors
        //    (type SLALOM_PIPE, subtype 2 = red, 1 = white; map ids)
        // 2. auto gap = vortex::mission::match_pipes(red, white, pose,
        //                                            side, exclude, offset);
        // 3. no gap -> FAILURE (the tree searches or goes blind)
        // 4. write gap_pose (x, y, slalom depth, yaw = heading), red_id;
        //    SUCCESS
    }
};
```

Register it in `robosub_mission/src/register_nodes.cpp`:

```cpp
factory.registerNodeType<MatchPipes>("MatchPipes");
```

and add `landmark_targets` and `vortex_msgs` to `robosub_mission`'s
`package.xml` and `CMakeLists.txt`.

Recording the layer (append `red_id` to `exclude`, store the offset) is a
second small node, e.g. `RecordLayer`, run after the vehicle is through.

### 7.2 The subtree

`robosub_mission/behavior_trees/tasks/slalom.xml`, a starting point with the
fallbacks written in the tree rather than in the nodes:

```xml
<?xml version="1.0"?>
<root BTCPP_format="4">
  <BehaviorTree ID="Slalom">
    <Repeat num_cycles="3">
      <ForceSuccess>
        <Fallback name="Layer">
          <Sequence name="Matched">
            <RetryUntilSuccessful num_attempts="10">
              <Sequence>
                <Sleep msec="100"/>
                <MatchPipes map="{map}" pose="{pose}" gate_side="{gate_side}"
                            exclude="{passed_red_ids}" offset="{slalom_offset}"
                            gap_pose="{gap_pose}" red_id="{red_id}"/>
              </Sequence>
            </RetryUntilSuccessful>
            <Timeout msec="45000">
              <GoTo pose="{gap_pose}" mode="POSITION_AND_YAW"
                    position_tolerance="0.3" orientation_tolerance_deg="20" hold_s="1.0"/>
            </Timeout>
            <Timeout msec="15000">
              <MoveRelative frame="BODY_RELATIVE" x="1.0" position_tolerance="0.3"/>
            </Timeout>
            <RecordLayer red_id="{red_id}" passed="{passed_red_ids}"/>
          </Sequence>
          <Timeout msec="20000">
            <MoveRelative frame="BODY_RELATIVE" x="2.0" position_tolerance="0.4"/>  <!-- blind layer -->
          </Timeout>
        </Fallback>
      </ForceSuccess>
    </Repeat>
  </BehaviorTree>
</root>
```

Then in `main.xml`: `<include path="tasks/slalom.xml"/>` at the top and
`<SubTree ID="Slalom" _autoremap="true"/>` in `Mission`. `GoTo`,
`MoveRelative` and the map/pose feeders are shared nodes (see the RoboSub
course tree plan); if they don't exist yet, agree with whoever writes them on
the port names.

**Rule of thumb:** nodes do one thing and return SUCCESS or FAILURE; retries,
timeouts, searches and blind layers are written in the XML.

## 8. Testing

1. **Unit tests** (section 5): fast, no simulator.
2. **Simulator with the dummy perception** (perfect pipes):
   ```bash
   src/vortex-auv/utility_scripts/launch_drone_sim.sh --scenario robosub --low-res --detach
   src/vortex-cv/perception_setup/scripts/tmux_robosub_sim.sh --fov
   ros2 run landmark_targets landmark_targets_scenario_node --ros-args \
     -r __ns:=/nautilus -p scenario:=slalom
   ```
   Start in front of the first layer (pipes within 7 m). The scenario ends
   with `SCENARIO SUCCESS`. Watch it in Foxglove with
   `landmark_server/foxglove/landmark_graph.json`.
3. **Simulator with the real slalom detector:** see
   `landmark_server/docs/sim_detector_test.md` (the dummy without slalom,
   checks of the detector output, what to look for).
4. **Your tree:** `ros2 launch perception_setup robosub_mission.launch.py`
   once your nodes are registered.

## 9. Files

| File | What |
|---|---|
| `include/landmark_targets/slalom.hpp`, `src/slalom.cpp` | `match_pipes`, `avoid_slalom_waypoints` (the reference solution) |
| `include/landmark_targets/geometry.hpp` | `forward_distance`, `side_of`, `perpendicular_heading`, course frame |
| `test/test_slalom.cpp` | The tests to pass |
| `examples/scenario_node.cpp` (`SlalomStep`) | The three-layer loop without a tree |
| `vortex-cv/mission/robosub/robosub_mission` | The behavior tree package |
| `landmark_server/config/landmark_server_config.yaml` (`classes.SLALOM_PIPE`) | How pipes are kept in the map: max 10 white / 5 red, 0.7 m apart, dropped within 1.5 m of the gate, remembered for the run after 30 observations |
