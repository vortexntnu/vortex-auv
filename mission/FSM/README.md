# Finite State Machine for Docking Task

## About
This package contains a Finite State Machine (FSM) for the docking task in TACC. It is built using the YASMIN library integrated with ROS2.

The FSM makes the  docking process autonomous, and the YASMIN library provides state management and execution flow for this task.

## States
### 1. FindDockingStationState
Finds the position of the docking station using pose action server. It takes num_measurements amount of measurements before switching state.
- Action: [Filtered pose](https://github.com/vortexntnu/vortex-msgs/blob/main/action/FilteredPose.action)

- Transitions
    - Success: ApproachDockingStationState
    - Abort: AbortState

### 2. ApproachDockingStationState
Go close to the docking station. Can be either DP or LOS.
- Action: [Reference filter waypoint](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action)

- Transitions
    - Success: GoAboveDockingStationState
    - Cancel: FindDockingStationState
    - Abort: AbortState

### 3. GoAboveDockingStationState
Go one unit above the docking station using DP.
- Action: [Reference filter waypoint](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action)

- Transitions
    - Success: UpdateDockingStationPositionState
    - Cancel: GoAboveDockingStationState
    - Abort: AbortState

### 5. UpdateDockingStationPositionState
Take more measurements to get a more accurate docking station position.
- Action: [Filtered pose](https://github.com/vortexntnu/vortex-msgs/blob/main/action/FilteredPose.action)

- Transitions
    - Success: ConvergeDockingStationState
    - Cancel: GoAboveDockingStationState
    - Abort: AbortState

### 6. ConvergeDockingStationState
Converge to the docking station and dock.
- Action: [Reference filter waypoint](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action)

- Transitions
    - Success: DockedState
    - Cancel: GoAboveDockingStationState
    - Abort: AbortState

### 7. DockedState
Waits dock_wait_time seconds at the dock.

- Transitions
    - Success: Finish
    - Abort: AbortState

### 7. ReturnHomeState
Go to the starting position.
- Action: [Reference filter waypoint](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action)

- Transitions
    - Success: FindDockingStationState
    - Cancel: ErrorState
    - Abort: AbortState

### 8. AbortState
Aborts the mission.

- Transitions
    - Success: Abort
    - Cancel: Abort
    - Abort: Abort

### 9. ErrorState
If an error occurs the FSM goes to this state.

- Transitions
    - Success: Error
    - Cancel: Error
    - Abort: Error


## Dependencies
Ensure the following dependencies are installed before running the FSM:

- `rclcpp`
- `yasmin`
- `yasmin_ros`
- `yasmin_viewer`
- `vortex_msgs`
- `std_msgs`
- `spdlog`
- `fmt`
