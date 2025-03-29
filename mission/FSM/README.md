# Finite State Machine for Docking Task

## About
This repository contains a Finite State Machine (FSM) for the docking task in TACC. It is built using the YASMIN library integrated with ROS2.

The FSM makes the  docking process autonomous, and the YASMIN library provides state management and execution flow for this task.

## States
### 1. FindDockingStationState
Finds the position of the docking station using pose action server. It takes num_measurements amount of measurements before switching state.
- Action: Filtered pose

### 2. ApproachDockingStationState
Go close to the docking station. Can be either DP or LOS.
- Action: Reference filter waypoint

### 3. GoAboveDockingStationState
Go one unit above the docking station using DP.
- Action: Reference filter waypoint

### 5. UpdateDockingStationPositionState
Take more measurements to get a more accurate docking station position.

- Action: Filtered pose

### 6. ConvergeDockingStationState
Converge to the docking station and dock.
- Action: Reference filter waypoint

### 7. Docked State
Waits dock_wait_time seconds at the dock.

### 7. ReturnHomeState
Go to the starting position if return_home is true.
- Action: Reference filter waypoint

### 8. AbortState
Aborts the mission.

### 9. ErrorState
If an error occurs the FSM goes to this state.



## How to Run

### 1. Build the Package
Before running the FSM, build the package:
```shell
colcon build --packages-select docking
```

### 2. Source the Workspace
After building, source the workspace:
```shell
source install/setup.bash
```

### 3. Run the FSM
You can run the FSM using one of the following methods:

- Run the script:
  ```shell
  ./src/vortex-auv/scripts/docking_fsm.sh
  ```

- Launch using ROS2 (Only starts the FSM, not other packages):
  ```shell
  ros2 launch docking docking.launch.py
  ```

### 4. Launch Yasmin Viewer (Optional for visualization)
To visualize the FSM using the Yasmin Viewer, run:
```shell
ros2 launch docking YASMIN_viewer.launch.py
```



## Dependencies
Ensure the following dependencies are installed before running the FSM:

- `rclcpp`
- `yasmin`
- `yasmin_ros`
- `yasmin_viewer`
- `vortex_msgs`
- `std_msgs`
