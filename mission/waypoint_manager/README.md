

# Waypoint Manager — ROS 2 Node


The **Waypoint Manager** node coordinates mission-level navigation by managing waypoint queues, forwarding them to a **Reference Filter** for trajectory generation, and exposing both an **action interface** (for mission planners) and a **service interface** (for perception-driven waypoint updates).



# Interfaces

* **[WaypointManager](https://github.com/vortexntnu/vortex-msgs/blob/main/action/WaypointManager.action) (action server)**
* **[WaypointAddition](https://github.com/vortexntnu/vortex-msgs/blob/main/srv/WaypointAddition.srv) (service server)**
* **[ReferenceFilterWaypoint](https://github.com/vortexntnu/vortex-msgs/blob/main/action/ReferenceFilterWaypoint.action) (action client)**


# Behavior Summary

### **Mission Start**

* On receiving a new action goal, any existing mission is aborted.
* Waypoints are stored, state is reset, and the first waypoint is sent to the reference filter.

### **During Mission**

* Each waypoint is executed sequentially.
* Pose feedback from the reference filter is forwarded to the mission planner.
* `convergence_threshold` determines when a waypoint is reached.
* If `persistent = true`, the action does not end even when waypoints run out.

### **Mission End**

* If `persistent = false`, the action completes when the last waypoint is reached.
* If cancelled externally, all state is cleared and reference filter goals are cancelled.

### **Dynamic Waypoint Addition**

Allowed only during **persistent** missions:

* `overwrite = true` → clears old queue and restarts from new waypoints
* `overwrite = false` → appends waypoints to queue
* `priority = true` → turn on priority mode. In this mode additional service requests with `priority = false` are ignored. Priority mode is set to false again when there are no more waypoints in queue.

---

### Frames, outcome and retargeting

* `frame` in the goal: `WORLD` (default, odom poses), `BODY_RELATIVE` (offsets in the vehicle frame at goal start) or `WORLD_RELATIVE` (offsets along the odom axes from the pose at goal start). Every waypoint of a goal is relative to the start pose.
* A new goal replaces the running one without stopping the vehicle: the reference filter retargets. The replaced goal ends with `PREEMPTED`.
* Result: `outcome` (`SUCCEEDED`, `PREEMPTED`, `CANCELED`, `INVALID_GOAL`, `REFERENCE_FILTER_ABORTED`), `message`, `reached_index` (-1 if none). Feedback: `current_waypoint`, `current_index`.
* `INVALID_GOAL`: NaN or inf in a pose, an unknown frame, or a relative frame before a pose was received. A running goal is not affected.
* Per waypoint, read by the reference filter: `position_tolerance` [m], `orientation_tolerance` [rad], `hold_time_sec`, `convergence_threshold` (0 = the goal's).
* Parameters: `action_servers.reference_filter`, `topics.pose`.

---

### Check available interfaces:

```bash
ros2 action list
ros2 service list
```
beware of namespace!


## Example Action Goal (CLI)

```bash
ros2 action send_goal /orca/waypoint_manager vortex_msgs/action/WaypointManager "{
  waypoints: [
    {
      pose: {
        position: {x: 5.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      mode: 1
    }
  ],
  convergence_threshold: 0.1,
  persistent: false
}" --feedback

```


## Example Waypoint Addition Service Call

```bash
ros2 service call /orca/waypoint_addition vortex_msgs/srv/SendWaypoints "{
  waypoints: [
    {
      pose: {
        position: {x: 2.0, y: 3.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      mode: 1
    }
  ],
  overwrite_prior_waypoints: false,
  take_priority: false
}"
```
