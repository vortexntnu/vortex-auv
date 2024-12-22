# Guidance Action Server with Third-Order LOS Guidance

This repository implements a **ROS2 Action Server** for autonomous waypoint navigation using a **Third-Order Line-of-Sight (LOS) Guidance Algorithm**. The project provides a robust and modular framework for guiding vehicles to desired waypoints while ensuring smooth motion through advanced filtering and control strategies.

---

## Features

- **Third-Order Line-of-Sight (LOS) Guidance**:
  - Computes guidance commands for surge, pitch, and yaw.
  - Includes state-space-based reference filtering for smooth transitions.

- **ROS2 Integration**:
  - Fully integrated with ROS2 nodes, publishers, subscribers, and action servers.
  - Modular structure for waypoint navigation and guidance control.

- **Debugging and Logging**:
  - Optional debug mode for publishing reference states, errors, and logs.
  - Easy-to-read debugging information for development and analysis.

---

## How It Works

1. **Vehicle State Updates**:
   - Subscribes to `/nucleus/odom` to receive vehicle odometry updates.
   - Extracts pose and orientation for guidance calculations.

2. **Waypoint Navigation**:
   - Accepts waypoint navigation goals via a ROS2 Action Server (`navigate_waypoints`).
   - Computes guidance commands (`surge`, `pitch`, `yaw`) to navigate toward waypoints.

3. **Guidance Algorithm**:
   - Uses a Third-Order LOS algorithm to calculate commands based on the vehicle's current state and the target waypoint.
   - Smoothens commands using a third-order state-space reference filter.

4. **Debug Mode**:
   - Publishes intermediate calculations and logs to topics like `/guidance/debug`.

---

## Topics and Interfaces

### Published Topics

| Topic                         | Type                        | Description                                  |
|-------------------------------|-----------------------------|----------------------------------------------|
| `/guidance/LOS_commands`      | `vortex_msgs/LOSGuidance`   | Guidance commands (surge, pitch, yaw).       |
| `/guidance/debug/reference`   | `geometry_msgs/PoseStamped`| Reference point for guidance (debug mode).   |
| `/guidance/debug/errors`      | `geometry_msgs/Vector3Stamped` | Position errors (debug mode).               |
| `/guidance/debug/logs`        | `std_msgs/String`           | Debugging logs (debug mode).                 |

### Subscribed Topics

| Topic             | Type                 | Description                        |
|-------------------|----------------------|------------------------------------|
| `/nucleus/odom`   | `nav_msgs/Odometry` | Vehicle's odometry information.   |

### Action Server

| Name                 | Type                        | Description                        |
|----------------------|-----------------------------|------------------------------------|
| `navigate_waypoints` | `vortex_msgs/NavigateWaypoints` | Navigate to a sequence of waypoints. |

---
