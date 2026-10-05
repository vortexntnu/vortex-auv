#!/usr/bin/env python3
"""Drift calibration with an ArUco board: how far the odometry drifts, and
how noisy the detector is, measured against a board fixed in the pool.

The board does not move. With the map on the raw odometry
(landmark_server.launch.py calibration:=true: no graph, no course), every
time the board is seen again its apparent move is the drift since the first
time. Each period the board is followed is a visit; per visit:
  - its position and yaw in odom (mean after settle_sec);
  - the jump against the first visit, the distance driven and the time since;
  - while the vehicle holds still: how the board creeps (hover drift) and
    how the detections spread along and across the line of sight (detector
    noise, from the raw detections in the camera frame).
After each visit, and when stopped (Ctrl-C), it prints the values for
config/pool.yaml: about twice the measured drift (the drift is a bias, the
graph's steps are independent), and the detector noise fitted over range.

Run live in the pool, or on a recording:
  ros2 run landmark_server aruco_drift.py --ros-args -r __ns:=/nautilus
  ros2 run landmark_server aruco_drift.py --ros-args -r __ns:=/tune_raw \\
      -p detections:=/nautilus/landmarks -p odom:=/nautilus/odom -p use_sim_time:=true
"""

import csv
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from vortex_msgs.msg import LandmarkArray, LandmarkTrackArray

ARUCO_BOARD = 2
ARUCO_BOARD_CAMERA = 1


def stamp_sec(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def circular_mean(angles):
    return math.atan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))


class Visit:
    def __init__(self, number):
        self.number = number
        self.map = []         # (t, x, y, yaw) of the board in odom
        self.raw = []         # (t, vector to the board in the camera frame)
        self.path = []        # (t, distance driven so far, x, y)

    def duration(self):
        return self.map[-1][0] - self.map[0][0] if self.map else 0.0


class ArucoDrift(Node):
    def __init__(self):
        super().__init__("aruco_drift")
        p = self.declare_parameter
        self.board_type = p("board_type", ARUCO_BOARD).value
        self.board_subtype = p("board_subtype", ARUCO_BOARD_CAMERA).value
        self.gap = p("visit_gap_sec", 2.0).value
        self.settle = p("settle_sec", 2.0).value
        self.min_visit = p("min_visit_sec", 3.0).value
        self.still_m = p("still_m", 0.3).value
        self.csv_path = p("csv", "").value
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(LandmarkTrackArray, p("object_map", "landmark_server/object_map").value,
                                 self.on_map, rel)
        self.create_subscription(LandmarkArray, p("detections", "landmarks").value,
                                 self.on_detections, qos_profile_sensor_data)
        self.create_subscription(Odometry, p("odom", "odom").value, self.on_odom,
                                 qos_profile_sensor_data)
        self.travelled = 0.0
        self.last_xy = None
        self.visit = None
        self.visits = []
        self.last_seen = None
        self.get_logger().info("waiting for the board (type %d, subtype %d) in the map"
                               % (self.board_type, self.board_subtype))

    # -- input ---------------------------------------------------------------

    def on_odom(self, m):
        xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if self.last_xy is not None:
            self.travelled += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy
        if self.visit is not None:
            self.visit.path.append((stamp_sec(m.header), self.travelled, xy[0], xy[1]))

    def on_detections(self, m):
        if self.visit is None:
            return
        for lm in m.landmarks:
            if lm.type.value == self.board_type and lm.subtype.value == self.board_subtype:
                v = lm.pose.pose.position
                self.visit.raw.append((stamp_sec(m.header), np.array([v.x, v.y, v.z])))

    def on_map(self, m):
        t = stamp_sec(m.header)
        board = None
        for k in m.landmark_tracks:
            lm = k.landmark
            if (lm.type.value == self.board_type and lm.subtype.value == self.board_subtype
                    and not k.derived):
                board = k
                break
        live = board is not None and not board.retained
        if live:
            if self.visit is None:
                self.visit = Visit(len(self.visits) + 1)
                self.get_logger().info("visit %d: board in view" % self.visit.number)
            p = board.landmark.pose.pose
            self.visit.map.append((t, p.position.x, p.position.y, yaw_of(p.orientation)))
            self.last_seen = t
        elif self.visit is not None and self.last_seen is not None and t - self.last_seen > self.gap:
            self.close_visit()

    # -- per visit -----------------------------------------------------------

    def close_visit(self):
        v, self.visit = self.visit, None
        if v.duration() < self.min_visit:
            self.get_logger().info("visit %d too short (%.1f s), ignored" % (v.number, v.duration()))
            return
        t0 = v.map[0][0] + self.settle
        rows = np.array([r for r in v.map if r[0] >= t0] or v.map)
        res = {"visit": len(self.visits) + 1, "t": float(np.mean(rows[:, 0])),
               "duration_s": v.duration(), "x": float(np.mean(rows[:, 1])),
               "y": float(np.mean(rows[:, 2])), "yaw": circular_mean(rows[:, 3])}
        # only while the board was in view (not the driving away after it)
        t_end = v.map[-1][0]
        path = [r for r in v.path if v.map[0][0] <= r[0] <= t_end]
        path = np.array(path) if path else None
        res["travelled_m"] = float(np.mean(path[:, 1])) if path is not None else self.travelled
        moved = 0.0
        if path is not None and len(path) > 1:
            moved = float(np.max(np.hypot(path[:, 2] - path[0, 2], path[:, 3] - path[0, 3])))
        res["moved_m"] = moved
        res["still"] = moved < self.still_m
        # hover drift: how the board creeps while the vehicle holds still
        res["creep_m_per_s"] = res["creep_deg_per_s"] = float("nan")
        if res["still"] and res["duration_s"] >= 45.0:
            n = max(3, len(rows) // 6)
            a, b = rows[:n], rows[-n:]
            dt = np.mean(b[:, 0]) - np.mean(a[:, 0])
            res["creep_m_per_s"] = math.hypot(np.mean(b[:, 1]) - np.mean(a[:, 1]),
                                              np.mean(b[:, 2]) - np.mean(a[:, 2])) / dt
            res["creep_deg_per_s"] = abs(math.degrees(wrap(circular_mean(b[:, 3]) -
                                                           circular_mean(a[:, 3])))) / dt
        # detector noise: spread of the raw detections, along and across the
        # line of sight, while still
        res["range_m"] = res["std_along_m"] = res["std_across_m"] = float("nan")
        raw = [r for t, r in v.raw if t0 <= t <= t_end]
        if len(raw) >= 10:
            raw = np.array(raw)
            mean = raw.mean(axis=0)
            res["range_m"] = float(np.linalg.norm(mean))
            if res["still"]:
                u = mean / np.linalg.norm(mean)
                d = raw - mean
                along = d @ u
                across = d - np.outer(along, u)
                res["std_along_m"] = float(np.std(along))
                res["std_across_m"] = float(math.sqrt(np.mean(np.sum(across ** 2, axis=1)) / 2.0))
        first = self.visits[0] if self.visits else res
        res["jump_m"] = math.hypot(res["x"] - first["x"], res["y"] - first["y"])
        res["jump_deg"] = math.degrees(wrap(res["yaw"] - first["yaw"]))
        res["since_m"] = res["travelled_m"] - first["travelled_m"]
        res["since_s"] = res["t"] - first["t"]
        self.visits.append(res)
        self.print_visit(res)
        self.print_summary()

    def print_visit(self, r):
        self.get_logger().info(
            "visit %d: %.0f s%s, range %.1f m | board (%.2f, %.2f) yaw %.1f deg | since visit 1: "
            "%.1f m driven, %.0f s -> jump %.3f m, %+.2f deg%s"
            % (r["visit"], r["duration_s"], " still" if r["still"] else "", r["range_m"],
               r["x"], r["y"], math.degrees(r["yaw"]), r["since_m"], r["since_s"],
               r["jump_m"], r["jump_deg"],
               "" if math.isnan(r["std_along_m"]) else
               " | detector std %.3f along, %.3f across" % (r["std_along_m"], r["std_across_m"])))

    # -- summary -------------------------------------------------------------

    def print_summary(self):
        moving = [r for r in self.visits[1:] if r["since_m"] > 2.0]
        lines = []
        if moving:
            d = np.array([r["since_m"] for r in moving])
            j = np.array([r["jump_m"] for r in moving])
            y = np.array([abs(r["jump_deg"]) for r in moving])
            per_m = float(d @ j / (d @ d))
            yaw_per_m = float(d @ y / (d @ d))
            lines.append("drift per metre: %.3f m/m, %.3f deg/m (%d visits after driving)"
                         % (per_m, yaw_per_m, len(moving)))
            lines.append("  graph.odom_noise.pos_std_per_m: %.3f" % max(2.0 * per_m, 0.005))
            lines.append("  graph.odom_noise.yaw_std_deg_per_m: %.3f" % max(2.0 * yaw_per_m, 0.005))
        hover = [r for r in self.visits if not math.isnan(r["creep_m_per_s"])]
        if hover:
            c = float(np.median([r["creep_m_per_s"] for r in hover]))
            cy = float(np.median([r["creep_deg_per_s"] for r in hover]))
            lines.append("hover drift: %.4f m/s, %.4f deg/s (%d still visits >= 45 s)"
                         % (c, cy, len(hover)))
            # one keyframe while hovering is 5 s
            lines.append("  graph.odom_noise.min_pos_std_m: %.3f" % max(2.0 * c * 5.0, 0.005))
            lines.append("  graph.odom_noise.yaw_std_deg_per_sec: %.4f" % max(2.0 * cy, 0.001))
        still = [r for r in self.visits if not math.isnan(r["std_along_m"])]
        if still:
            rng = np.array([r["range_m"] for r in still])
            fit = []
            for key in ("std_along_m", "std_across_m"):
                s = np.array([r[key] for r in still])
                if len(set(np.round(rng, 1))) >= 2:
                    per_m, base = np.polyfit(rng, s, 1)
                    base = max(base, 0.0)
                    per_m = max(per_m, 0.0)
                else:
                    base, per_m = float(np.mean(s)), 0.0
                fit.append((base, per_m))
            base = min(fit[0][0], fit[1][0])
            lines.append("detector noise over %d still visits, ranges %s m:"
                         % (len(still), ", ".join("%.1f" % x for x in sorted(rng))))
            lines.append("  detector_noise: {base_std_m: %.3f, along_std_per_m: %.3f, "
                         "across_std_per_m: %.4f}"
                         % (base, fit[0][1] + (fit[0][0] - base) / max(np.mean(rng), 1.0),
                            fit[1][1] + (fit[1][0] - base) / max(np.mean(rng), 1.0)))
            if len(set(np.round(rng, 1))) < 2:
                lines.append("  (one range only: hold still at 2, 4, 6 m for the growth with range)")
        if lines:
            self.get_logger().info("for config/pool.yaml:\n    " + "\n    ".join(lines))

    def finish(self):
        if self.visit is not None:
            self.close_visit()
        if self.csv_path and self.visits:
            with open(self.csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(self.visits[0].keys()))
                w.writeheader()
                w.writerows(self.visits)
            self.get_logger().info("visits written to %s" % self.csv_path)


def main():
    rclpy.init()
    node = ArucoDrift()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.finish()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
