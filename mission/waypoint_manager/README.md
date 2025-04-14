# Waypoint Manager

The Waypoint Manager is a ROS 2 node designed for Orca (our autonomous underwater vehicle) that provides high-level waypoint navigation capabilities by coordinating with the Reference Filter for smooth trajectory execution.

## Overview

The Waypoint Manager acts as a coordinator between high-level mission planning and low-level trajectory generation. It manages sequences of waypoints and delegates the detailed trajectory generation to the Reference Filter, creating a layered navigation architecture for Orca.

## System Architecture

### Waypoint Manager Node

The Waypoint Manager handles higher-level navigation logic:
- Receives waypoint sequences via an action server
- Manages waypoint-to-waypoint transitions based on proximity
- Determines when waypoints are reached using configurable thresholds
- Handles preemption of current navigation goals for new requests
- Provides feedback on Orca's navigation progress

### Integration with Reference Filter

The Reference Filter provides smoother trajectories via a third-order model that generates reference values for position, velocity, and acceleration. The integration works as follows:

1. **Client-Server Model**: The Waypoint Manager is a client of the Reference Filter's action server
2. **Single Waypoint Delegation**: The Waypoint Manager sends one waypoint at a time to the Reference Filter
3. **Distance-Based Switching**: When Orca is within the switching threshold of the current waypoint, the Waypoint Manager advances to the next waypoint
4. **Asynchronous Communication**: Uses promises and futures to handle asynchronous action server interactions

## How It Works

The waypoint navigation process follows these steps:

1. **Waypoint Sequence Reception**: The Waypoint Manager receives a sequence of waypoints through its action server
2. **First Waypoint Dispatch**: It sends the first waypoint to the Reference Filter's action server
3. **Position Monitoring**: Continuously monitors Orca's position
4. **Threshold Checking**: Compares the distance to the current waypoint against the switching threshold
5. **Waypoint Advancement**: When within threshold, it:
   - Cancels the current Reference Filter goal (if needed)
   - Advances to the next waypoint
   - Sends the new waypoint to the Reference Filter
6. **Completion Notification**: When all waypoints are reached, the action is marked as succeeded

## Custom Action Types

The Waypoint Manager uses a custom action definition:

### WaypointManager.action

```
# Goal
geometry_msgs/PoseStamped[] waypoints  # Array of waypoints for Orca to visit
string target_server                   # Name of the target action server
float64 switching_threshold            # Distance threshold for switching waypoints
---
# Result
bool success
uint32 completed_waypoints             # Number of waypoints completed
---
# Feedback
geometry_msgs/Pose current_pose        # Current pose of Orca
uint32 current_waypoint_index          # Index of the current waypoint
float64 distance_to_waypoint           # Distance to the current waypoint
```

This action allows clients to:
- Send multiple waypoints in a single request
- Specify which server should handle the trajectory generation (currently only supporting "reference_filter")
- Set a custom threshold for when to consider a waypoint reached
- Receive feedback on progress including the current position, active waypoint, and distance remaining

The Waypoint Manager in turn uses the Reference Filter's action server:

### ReferenceFilterWaypoint.action

```
# Goal
geometry_msgs/PoseStamped goal
---
# Result
bool success
vortex_msgs/ReferenceFilter result
---
# Feedback
vortex_msgs/ReferenceFilter feedback
```

## Implementation Details

### Multi-threading Approach

The Waypoint Manager uses a multi-threaded architecture to handle:
- Main thread for ROS 2 callbacks
- Separate execution thread for waypoint navigation
- Thread-safe communication using mutexes and promises/futures

### Error Handling

The system includes robust error handling:
- Detection of Reference Filter action server unavailability
- Goal rejection handling
- Preemption of current goals
- Clean cancellation of navigation sequences

## Interfaces

### Action Servers

- **`/orca/waypoint_manager`**: Accepts navigation requests with lists of waypoints

### Action Clients

- **`/orca/reference_filter`**: Sends individual waypoints to the Reference Filter node

### Subscriptions

- **`/orca/pose`**: Orca's pose feedback (PoseWithCovarianceStamped)

## Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `topics.pose` | Topic for Orca's pose | `/orca/pose` |
| `action_servers.waypoint_manager` | Name of the waypoint manager action server | `waypoint_manager` |
| `action_servers.reference_filter` | Name of the reference filter action server | `reference_filter` |


## Dependencies

- ROS 2
- rclcpp
- rclcpp_action
- geometry_msgs
- vortex_msgs
- tf2, tf2_geometry_msgs
- spdlog (for logging)
- fmt (for string formatting)
