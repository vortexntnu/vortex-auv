#!/usr/bin/env python3
"""Simulator tool: drive a loop past the RoboSub course with waypoint_manager.

Out along y = -3 (beside the gate and the slalom, no collisions) to the
torpedo board, turn, back, and finally in front of the gate again: the gate
seen at the start and at the end closes the loop. World frame (true odometry).
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import WaypointManager
from vortex_msgs.msg import Waypoint, WaypointMode

# x, y, yaw [deg], hold [s]
ROUTE = [
    (0.0, 0.0, 0.0, 4.0),
    (2.0, -3.0, 0.0, 0.0),
    (13.0, -3.0, 0.0, 4.0),
    (13.0, -3.0, 180.0, 0.0),
    (2.0, -3.0, 180.0, 0.0),
    (-1.0, 0.0, 180.0, 0.0),
    (-1.0, 0.0, 0.0, 8.0),
]


def waypoint(x, y, z, yaw_deg, hold):
    wp = Waypoint()
    wp.pose.position.x, wp.pose.position.y, wp.pose.position.z = x, y, z
    wp.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2.0)
    wp.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2.0)
    wp.waypoint_mode.mode = WaypointMode.FULL_POSE
    wp.position_tolerance = 0.3
    wp.orientation_tolerance = 0.15
    wp.hold_time_sec = hold
    return wp


class Route(Node):
    def __init__(self):
        super().__init__("drift_route")
        self.declare_parameter("depth", 2.0)
        self.declare_parameter("action", "/nautilus/waypoint_manager")
        z = self.get_parameter("depth").value
        self._client = ActionClient(
            self, WaypointManager, self.get_parameter("action").value
        )
        self._goal = WaypointManager.Goal()
        self._goal.waypoints = [
            waypoint(x, y, z, yaw, hold) for x, y, yaw, hold in ROUTE
        ]
        self._goal.convergence_threshold = 0.3
        self._goal.frame = WaypointManager.Goal.WORLD

    def run(self):
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("no waypoint_manager")
            return 1
        fut = self._client.send_goal_async(self._goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected")
            return 1
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        r = res.result().result
        self.get_logger().info(
            f"done: success={r.success} outcome={r.outcome} reached={r.reached_index} {r.message}"
        )
        return 0 if r.success else 1

    def _feedback(self, fb):
        i = fb.feedback.current_index
        if getattr(self, "_last", None) != i:
            self._last = i
            x, y, yaw, _ = ROUTE[i]
            self.get_logger().info(f"waypoint {i}: ({x}, {y}) yaw {yaw}")


def main():
    rclpy.init()
    sys.exit(Route().run())


if __name__ == "__main__":
    main()
