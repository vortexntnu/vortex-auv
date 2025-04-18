# Waypoint Manager

The Waypoint Manager is a ROS 2 node designed for Orca (our autonomous underwater vehicle) that provides high-level waypoint navigation capabilities by coordinating with the Reference Filter for smooth trajectory execution.

## Overview

The Waypoint Manager acts as a coordinator between high-level mission planning and low-level trajectory generation. It manages sequences of waypoints and delegates the detailed trajectory generation to the Reference Filter, creating a layered navigation architecture for Orca.

## System Architecture

### Waypoint Manager Node

The Waypoint Manager handles higher-level navigation logic:
- Receives waypoint sequences via an action server
- Processes waypoints sequentially
- Handles preemption of current navigation goals for new requests
- Provides feedback on navigation progress

### Integration with Reference Filter

The Reference Filter provides smoother trajectories via a third-order model that generates reference values for position, velocity, and acceleration. The integration works as follows:

1. **Client-Server Model**: The Waypoint Manager is a client of the Reference Filter's action server
2. **Single Waypoint Delegation**: The Waypoint Manager sends one waypoint at a time to the Reference Filter
3. **Completion-Based Switching**: The Waypoint Manager advances to the next waypoint when the Reference Filter reports successful completion
4. **Asynchronous Communication**: Uses promises and futures to handle asynchronous action server interactions

## How It Works

The waypoint navigation process follows these steps:

1. **Waypoint Sequence Reception**: The Waypoint Manager receives a sequence of waypoints through its action server
2. **Sequential Processing**: It sends each waypoint to the Reference Filter's action server one at a time
3. **Completion Monitoring**: Waits for the Reference Filter to report success or failure for each waypoint
4. **Waypoint Advancement**: When a waypoint is successfully reached, it:
   - Advances to the next waypoint in the sequence
   - Sends the new waypoint to the Reference Filter
5. **Completion Notification**: When all waypoints are reached, the action is marked as succeeded

## Custom Action Types

The Waypoint Manager uses a custom action definition:

### WaypointManager.action

```
# Goal
geometry_msgs/PoseStamped[] waypoints  # Array of waypoints for Orca to visit
string target_server                   # Name of the target action server
float64 switching_threshold            # Distance threshold (reserved for future use)
---
# Result
bool success
uint32 completed_waypoints             # Number of waypoints completed
---
# Feedback
uint32 current_waypoint_index          # Index of the current waypoint
```

This action allows clients to:
- Send multiple waypoints in a single request
- Specify which server should handle the trajectory generation (currently only supporting "reference_filter")
- Receive feedback on progress including the active waypoint

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
- Thread-safe communication using mutexes, atomic variables, and promises/futures

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

## Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
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
