# Mission Execution System

A ROS 2 (Humble) and BehaviorTree.CPP (v4.7+) based mission execution system with pause/resume support and state persistence.

## Overview

This package implements a mission orchestration system where BehaviorTree.CPP serves as the top-level mission orchestrator. The system is designed to support manual operator interruption and resume from the same logical point in the behavior tree without restarting the BT.

## Core Design Principles

1. **WaypointAction Lifecycle**: The WaypointManager action lifecycle defines when external waypoints are valid
2. **BehaviorTree Control**: The BT explicitly starts and stops the WaypointAction
3. **State Persistence**: Mission execution state survives BT tick stops and manual operator takeover
4. **Pause/Resume**: Mission execution can be paused and resumed without losing state

## Architecture

### Mission Execution State

The system tracks mission execution state independently of BT ticking through the `MissionExecutionGate` node:

- **Execution States**: `IDLE`, `RUNNING`, `PAUSED`
- **Mission Phases**: `SEARCH`, `CONVERGE`, `PIPELINE`, `DONE`

### Mission Phases

#### Phase 1: SEARCH + LANDMARK ENUM (Parallel)
- **WaypointAction**: ACTIVE (persistent mode)
- External perception nodes send waypoints continuously
- `LandmarkEnumPolling` sends action goal to Landmark ENUM Action Server
- When ENUM succeeds:
  - Search stops
  - WaypointAction is stopped
  - MissionPhase transitions to `CONVERGE`

#### Phase 2: LANDMARK CONVERGE (Exclusive)
- **WaypointAction**: INACTIVE
- `LandmarkConverge` sends action goal to Landmark Convergence Action Server
- Uses ReferenceFilter directly for motion control
- On success: MissionPhase transitions to `PIPELINE`

#### Phase 3: PIPELINE FOLLOWING
- **WaypointAction**: ACTIVE (persistent mode)
- External pipeline node sends waypoints continuously
- Mission ends when PipelineFollowing finishes
- MissionPhase transitions to `DONE`

## BehaviorTree Nodes

### MissionExecutionGate
**Type**: Stateful Condition Node  
**Purpose**: Gates mission execution at the root of the tree

**Responsibilities**:
- Track mission execution state (IDLE, RUNNING, PAUSED)
- Track current mission phase (SEARCH, CONVERGE, PIPELINE, DONE)
- Provide ROS 2 services for mission control
- Write `mission_phase` and `mission_id` to blackboard

**ROS Services**:
- `/mission/start` - Start mission execution
- `/mission/pause` - Pause mission execution
- `/mission/resume` - Resume mission execution
- `/mission/abort` - Abort mission execution

**Behavior**:
- Returns `FAILURE` if mission not started
- Returns `RUNNING` if mission is paused
- Returns `SUCCESS` if mission is running

**Blackboard Ports**:
- Input: `mission_phase_request` (from SetMissionPhase)
- Output: `mission_phase` (current phase)
- Output: `mission_id` (current mission ID)

### SetMissionPhase
**Type**: SyncActionNode  
**Purpose**: Update mission phase in blackboard

**Blackboard Ports**:
- Input: `value` (mission phase string: SEARCH, CONVERGE, PIPELINE, DONE)

### StartWaypointAction
**Type**: RosActionNode  
**Purpose**: Start WaypointManager action in persistent mode

**Behavior**:
- Sends action goal with `persistent=true` and empty waypoints
- Allows external nodes to send waypoints continuously via service

**Blackboard Ports**:
- Input: `action_name` (default: `/orca/waypoint_manager`)

### StopWaypointAction
**Type**: StatefulActionNode  
**Purpose**: Stop/cancel WaypointManager action

**Behavior**:
- Cancels all active goals for the WaypointManager action
- Prevents external waypoints from being accepted

**Blackboard Ports**:
- Input: `action_name` (default: `/orca/waypoint_manager`)

### SearchPattern
**Type**: StatefulActionNode  
**Purpose**: Execute search pattern

**Behavior**:
- Runs for specified duration or until halted
- External perception nodes send waypoints while this runs
- Halted when LandmarkEnumPolling succeeds

**Blackboard Ports**:
- Input: `duration` (maximum duration in seconds, default: 60.0)

### LandmarkEnumPolling
**Type**: RosActionNode  
**Purpose**: Poll for landmarks via action server

**Behavior**:
- Sends action goal to Landmark ENUM Action Server
- Goal contains landmark enum type
- Result returns `landmark_id` and optional `pose`
- Writes `landmark_id` and `landmark_pose` to blackboard

**Blackboard Ports**:
- Input: `action_name` (default: `/landmark_enum`)
- Input: `landmark_type` (BUOY=1, BOAT=2, WALL=69, default: 1)
- Output: `landmark_id`
- Output: `landmark_pose`

**Note**: Currently uses mock implementation that always returns success. Replace with actual action server implementation.

### LandmarkConverge
**Type**: RosActionNode  
**Purpose**: Converge to detected landmark

**Behavior**:
- Sends action goal to Landmark Convergence Action Server
- Uses ReferenceFilter directly for motion control
- Reads `landmark_id` from blackboard

**Blackboard Ports**:
- Input: `action_name` (default: `/landmark_convergence`)
- Input: `landmark_id` (from LandmarkEnumPolling)
- Input: `landmark_pose` (optional pose hint)
- Input: `convergence_threshold` (default: 0.5)

**Note**: Currently uses mock implementation that always returns success. Replace with actual action server implementation.

### PipelineFollowing
**Type**: StatefulActionNode  
**Purpose**: Monitor pipeline following completion

**Behavior**:
- Monitors `end_of_pipeline` flag from blackboard
- External pipeline node sends waypoints while this runs
- Returns success when `end_of_pipeline` is true

**Blackboard Ports**:
- Input: `end_of_pipeline` (flag indicating pipeline completion)

## Behavior Tree Structure

```
<Sequence>
  <MissionExecutionGate
      mission_phase="{mission_phase}"
      mission_id="{mission_id}" />
  <Switch value="{mission_phase}">
    <Case value="SEARCH">
      <Parallel success_threshold="1" failure_threshold="1">
        <Sequence>
          <StartWaypointAction />
          <SearchPattern />
          <StopWaypointAction />
          <SetMissionPhase value="CONVERGE"/>
        </Sequence>
        <LandmarkENUMPolling landmark_id="{landmark_id}" />
      </Parallel>
    </Case>
    <Case value="CONVERGE">
      <LandmarkConverge landmark_id="{landmark_id}" />
      <SetMissionPhase value="PIPELINE"/>
    </Case>
    <Case value="PIPELINE">
      <Sequence>
        <StartWaypointAction />
        <PipelineFollowing />
        <StopWaypointAction />
        <SetMissionPhase value="DONE"/>
      </Sequence>
    </Case>
    <Case value="DONE">
      <AlwaysSuccess/>
    </Case>
  </Switch>
</Sequence>
```

## ROS 2 Services

### /mission/start
**Service Type**: `vortex_msgs/srv/MissionStart`

**Request**:
- `mission_id` (string): Mission identifier

**Response**:
- `success` (bool): Whether start was successful
- `message` (string): Status message

**Behavior**: Transitions execution state from `IDLE` to `RUNNING`, sets mission phase to `SEARCH`.

### /mission/pause
**Service Type**: `vortex_msgs/srv/MissionPause`

**Request**: (empty)

**Response**:
- `success` (bool): Whether pause was successful
- `message` (string): Status message

**Behavior**: Transitions execution state from `RUNNING` to `PAUSED`. Mission phase is preserved.

### /mission/resume
**Service Type**: `vortex_msgs/srv/MissionResume`

**Request**: (empty)

**Response**:
- `success` (bool): Whether resume was successful
- `message` (string): Status message

**Behavior**: Transitions execution state from `PAUSED` to `RUNNING`. Mission phase is NOT reset - resumes from same phase.

### /mission/abort
**Service Type**: `vortex_msgs/srv/MissionAbort`

**Request**: (empty)

**Response**:
- `success` (bool): Whether abort was successful
- `message` (string): Status message

**Behavior**: Transitions execution state to `IDLE`, resets mission phase to `SEARCH`, clears mission ID.

## Pause/Resume Semantics

### Pause
When operator calls `/mission/pause`:
1. Execution state transitions to `PAUSED`
2. `MissionExecutionGate` returns `RUNNING` (keeps BT ticking but blocks progress)
3. Active BT actions are halted
4. WaypointAction is cancelled if active
5. Mission phase is preserved

### Resume
When operator calls `/mission/resume`:
1. Execution state transitions to `RUNNING`
2. `MissionExecutionGate` returns `SUCCESS` (allows BT to progress)
3. Mission phase is NOT reset - continues from same phase
4. BT continues from where it was paused

## WaypointAction Lifecycle

### Starting WaypointAction
- `StartWaypointAction` sends goal with `persistent=true`
- External nodes can now send waypoints via `SendWaypoints` service
- WaypointManager accepts and executes waypoints continuously

### Stopping WaypointAction
- `StopWaypointAction` cancels all active goals
- External waypoints are no longer accepted
- WaypointManager stops executing waypoints

## External Integration

### External Perception Nodes
- Send waypoints continuously via `SendWaypoints` service
- Waypoints are only accepted when WaypointAction is active
- Used during SEARCH phase

### External Pipeline Node
- Sends waypoints continuously via `SendWaypoints` service
- Waypoints are only accepted when WaypointAction is active
- Used during PIPELINE phase
- Sets `end_of_pipeline` flag when complete

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select mission_execution vortex_msgs
source install/setup.bash
```

## Usage

### Starting the BT Executor

```bash
ros2 run mission_execution <executor_node> --ros-args \
  -p bt_loop_duration:=10 \
  -p default_server_port:=1666 \
  -p plugin_lib_names:=mission_execution_plugin
```

### Controlling Mission Execution

```bash
# Start mission
ros2 service call /mission/start vortex_msgs/srv/MissionStart "{mission_id: 'mission_001'}"

# Pause mission
ros2 service call /mission/pause vortex_msgs/srv/MissionPause "{}"

# Resume mission
ros2 service call /mission/resume vortex_msgs/srv/MissionResume "{}"

# Abort mission
ros2 service call /mission/abort vortex_msgs/srv/MissionAbort "{}"
```

## Implementation Notes

### Mock Implementations

Currently, `LandmarkEnumPolling` and `LandmarkConverge` use mock implementations that always return success. These should be replaced with actual action server implementations:

1. **LandmarkEnumPolling**: Replace mock in `onResultReceived()` and `onFailure()` methods
2. **LandmarkConverge**: Replace mock in `onResultReceived()` and `onFailure()` methods

### Action Servers Required

The system expects the following action servers to be available:

- `/landmark_enum` - LandmarkEnum action server
- `/landmark_convergence` - LandmarkConvergence action server
- `/orca/waypoint_manager` - WaypointManager action server (existing)

### Service Definitions

Service definitions are in `vortex_msgs` package:
- `srv/MissionStart.srv`
- `srv/MissionPause.srv`
- `srv/MissionResume.srv`
- `srv/MissionAbort.srv`

### Action Definitions

Action definitions are in `vortex_msgs` package:
- `action/LandmarkEnum.action`
- `action/LandmarkConvergence.action`

## Dependencies

- `rclcpp`
- `rclcpp_action`
- `rclcpp_components`
- `geometry_msgs`
- `std_msgs`
- `vortex_msgs`
- `behaviortree_ros2`
- `behaviortree_cpp`

## File Structure

```
mission_execution/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── mission_execution/
│       ├── mission_execution_gate.hpp
│       ├── set_mission_phase.hpp
│       ├── start_waypoint_action.hpp
│       ├── stop_waypoint_action.hpp
│       ├── landmark_enum_polling.hpp
│       ├── landmark_converge.hpp
│       ├── search_pattern.hpp
│       └── pipeline_following.hpp
├── src/
│   ├── mission_execution_gate.cpp
│   ├── set_mission_phase.cpp
│   ├── start_waypoint_action.cpp
│   ├── stop_waypoint_action.cpp
│   ├── landmark_enum_polling.cpp
│   ├── landmark_converge.cpp
│   ├── search_pattern.cpp
│   ├── pipeline_following.cpp
│   └── plugin_register.cpp
└── behavior_trees/
    └── mission_execution.xml
```

