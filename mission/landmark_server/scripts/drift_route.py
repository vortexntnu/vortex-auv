#!/usr/bin/env python3
"""Simulator tool: drive a loop past the RoboSub course with waypoint_manager.

route:=short (default): out along y = -3 (beside the gate and the slalom, no collisions) to the
torpedo board, turn, back, and finally in front of the gate again: the gate
seen at the start and at the end closes the loop.

route:=long: two laps that also look at the bins and the table, so pipes,
board and bins are seen again after a lot of drift.

route:=robosub: the course as a run does it, for Search & Rescue (the
roles drawn with the simulator's seed 7): look at the gate from the start,
through its Search & Rescue half below the panels, stop in front of the
slalom to find it, through the three sets, to the torpedo board (from
2.5 m, 1.5 m, then lined up with the large Search & Rescue opening), over
the bins (down camera), look at the octagon, over the table under it and up
inside the octagon. Depth per waypoint.

route:=slalom: past the gate at y = -3, then laps through the slalom at the
depth of the poles (depth:=2.6): out through the three sets facing +x, turn,
back through them facing -x. Each set is passed between the red pole and the
white one on its left, 0.8 m from both. The front camera sees the poles on
every pass, so they are seen again after more and more drift.

World frame (true odometry).
"""

import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import WaypointManager
from vortex_msgs.msg import Waypoint, WaypointMode

# x, y, yaw [deg], hold [s]
SHORT = [
    (0.0, 0.0, 0.0, 4.0),
    (2.0, -3.0, 0.0, 0.0),
    (13.0, -3.0, 0.0, 4.0),
    (13.0, -3.0, 180.0, 0.0),
    (2.0, -3.0, 180.0, 0.0),
    (-1.0, 0.0, 180.0, 0.0),
    (-1.0, 0.0, 0.0, 8.0),
]

# Two laps, the bins and the table included: everything is seen again after a
# lot of drift (take-over of remembered pipes, bins and the board).
LONG = [
    (0.0, 0.0, 0.0, 4.0),
    (2.0, -3.0, 0.0, 0.0),
    (13.0, -3.0, 0.0, 3.0),
    (13.0, 2.5, 0.0, 4.0),
    (13.0, 2.5, 180.0, 0.0),
    (13.0, -3.0, 180.0, 0.0),
    (2.0, -3.0, 180.0, 0.0),
    (-1.0, 0.0, 180.0, 0.0),
    (-1.0, 0.0, 0.0, 4.0),
    (2.0, -3.0, 0.0, 0.0),
    (13.0, -3.0, 0.0, 3.0),
    (13.0, 2.5, 0.0, 6.0),
]
# Gaps of the three slalom sets (between the red pole and the white one on
# its left, robosub_dummy_publisher course_layout.py): x of the set, y of
# the middle of the gap.
SLALOM_GAPS = [(8.12, -0.45), (10.13, 0.15), (12.13, -0.85)]
SLALOM_LAPS = 3


def slalom_route():
    out = [(0.0, 0.0, 0.0, 2.0), (2.0, -3.0, 0.0, 0.0), (6.0, -3.0, 0.0, 0.0)]
    first_x, first_y = SLALOM_GAPS[0]
    last_x, last_y = SLALOM_GAPS[-1]
    for _ in range(SLALOM_LAPS):
        out.append((first_x - 2.0, first_y, 0.0, 3.0))
        out += [(x, y, 0.0, 0.0) for x, y in SLALOM_GAPS]
        out += [(last_x + 1.8, last_y, 0.0, 0.0), (last_x + 1.8, last_y, 180.0, 3.0)]
        out += [(x, y, 180.0, 0.0) for x, y in reversed(SLALOM_GAPS)]
        out.append((first_x - 2.0, first_y, 180.0, 0.0))
    out.append((first_x - 2.0, first_y, 0.0, 8.0))
    return out




# The RoboSub course in order (x, y, yaw [deg], hold [s], depth [m]); world
# positions from robosub_dummy_publisher course_layout.py. The gate's
# Search & Rescue half (seed 7) is at y 0.77, between the middle post (down
# to 2.76 m) and the right upright, below the role panels (2.27 m).
GATE_SR_Y = 0.77
# Large Search & Rescue opening of torpedo board version 1 (seed 7).
TORPEDO_SR = (-4.990, 2.770)
BOARD_X, BOARD_Y, BOARD_Z = 17.043, -5.204, 2.554


def robosub_route():
    out = [
        (0.0, 0.0, 0.0, 3.0, 2.3),                 # the gate, 4 m ahead
        (2.5, GATE_SR_Y, 0.0, 2.0, 2.95),          # lined up with our half
        (5.2, GATE_SR_Y, 0.0, 0.0, 2.95),          # through the gate
    ]
    first_x, first_y = SLALOM_GAPS[0]
    last_x, last_y = SLALOM_GAPS[-1]
    out.append((first_x - 2.0, first_y, 0.0, 3.0, 2.6))       # find the slalom
    out += [(x, y, 0.0, 0.0, 2.6) for x, y in SLALOM_GAPS]   # through the sets
    out.append((last_x + 1.4, last_y, 0.0, 0.0, 2.6))
    out += [
        (14.5, -3.0, -60.0, 0.0, 2.55),                       # towards the board
        (BOARD_X - 2.5, BOARD_Y, 0.0, 4.0, BOARD_Z),          # find it
        (BOARD_X - 1.5, BOARD_Y, 0.0, 4.0, BOARD_Z),          # closer
        (BOARD_X - 1.2, TORPEDO_SR[0], 0.0, 3.0, TORPEDO_SR[1]),  # shoot
        (BOARD_X - 2.5, BOARD_Y, 0.0, 0.0, BOARD_Z),          # back off
        (14.5, 1.5, 90.0, 0.0, 1.6),                          # to the bins
        (16.5, 4.2, 90.0, 5.0, 1.6),                          # over them
        (16.9, 4.2, 90.0, 2.0, 1.6),                          # drop over a bin
        (16.5, 0.11, 0.0, 3.0, 1.0),                          # the octagon ahead
        (19.25, 0.11, 0.0, 3.0, 1.0),                         # over the table
        (19.25, 0.11, 0.0, 6.0, 0.3),                         # up inside it
    ]
    return out


ROUTES = {"short": SHORT, "long": LONG, "slalom": slalom_route(), "robosub": robosub_route()}


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
        self.declare_parameter("route", "short")
        z = self.get_parameter("depth").value
        self._route = ROUTES[self.get_parameter("route").value]
        self._client = ActionClient(
            self, WaypointManager, self.get_parameter("action").value
        )
        self._goal = WaypointManager.Goal()
        # (x, y, yaw, hold) at the depth parameter, or (x, y, yaw, hold, z).
        self._goal.waypoints = [
            waypoint(w[0], w[1], w[4] if len(w) > 4 else z, w[2], w[3])
            for w in self._route
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
            x, y, yaw = self._route[i][:3]
            self.get_logger().info(f"waypoint {i}: ({x}, {y}) yaw {yaw}")


def main():
    rclpy.init()
    sys.exit(Route().run())


if __name__ == "__main__":
    main()
