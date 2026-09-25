#!/usr/bin/env python3
"""Simulator tool: map error with and without the smoothing backend.

Compares two object_maps (landmark_server with graph.enable true and false,
fed the same drifting data from drift_injector) to the truth. The truth of a
landmark is its true world position (landmarks_true) expressed in the drifted
odom frame (drift_injector's /nautilus/drift): where it really is relative to
the vehicle, in the frame the vehicle navigates in.

Every second: mean/max error of remembered (not live) and all landmarks per
map. Also written to csv (param csv, relative to the working directory).
"""

import csv
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from vortex_msgs.msg import LandmarkArray, LandmarkTrackArray, LandmarkType

TYPE_NAMES = {
    v: k for k, v in vars(LandmarkType).items() if k.isupper() and isinstance(v, int)
}


def pose_to_mat(p):
    w, x, y, z = p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z
    t = np.eye(4)
    t[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]
    t[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return t


class GraphEval(Node):
    def __init__(self):
        super().__init__("graph_eval")
        self.declare_parameter(
            "maps",
            [
                "/nautilus/landmark_server/object_map",
                "/nautilus_raw/landmark_server/object_map",
            ],
        )
        self.declare_parameter("labels", ["graph", "raw"])
        self.declare_parameter("csv", "graph_eval.csv")
        self.declare_parameter(
            "z_locked_types", [LandmarkType.PATH_MARKER, LandmarkType.OCTAGON]
        )
        g = self.get_parameter
        self._labels = list(g("labels").value)
        self._maps = dict.fromkeys(self._labels)
        self._truth = {}  # (type, subtype) -> list of world positions
        self._c = np.eye(4)
        self._vehicle = None
        self._skip_z = set(g("z_locked_types").value)
        for topic, lab in zip(g("maps").value, self._labels):
            self.create_subscription(
                LandmarkTrackArray,
                topic,
                lambda m, lab=lab: self._maps.__setitem__(lab, m),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            )
        self.create_subscription(
            LandmarkArray,
            "/nautilus/landmarks_true",
            self._on_truth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            "/nautilus/drift",
            lambda m: setattr(self, "_c", pose_to_mat(m.pose)),
            10,
        )
        self.create_subscription(
            Odometry, "/nautilus/odom", self._on_odom, qos_profile_sensor_data
        )
        self._csv = open(g("csv").value, "w", newline="")
        self._w = csv.writer(self._csv)
        header = ["t", "x", "y", "drift_yaw_deg"]
        for lab in self._labels:
            header += [
                f"{lab}_retained_mean",
                f"{lab}_retained_max",
                f"{lab}_all_mean",
                f"{lab}_n",
            ]
        self._w.writerow(header)
        self._t0 = time.monotonic()
        self.create_timer(1.0, self._evaluate)

    def _on_odom(self, m):
        p = m.pose.pose.position
        self._vehicle = (p.x, p.y)

    def _on_truth(self, msg):
        for lm in msg.landmarks:
            key = (lm.type.value, lm.subtype.value)
            p = lm.pose.pose.position
            pos = np.array([p.x, p.y, p.z])
            lst = self._truth.setdefault(key, [])
            if all(np.linalg.norm(q - pos) > 0.05 for q in lst):
                lst.append(pos)

    def _errors(self, msg):
        rows = []
        for t in msg.landmark_tracks:
            if t.derived:
                continue
            key = (t.landmark.type.value, t.landmark.subtype.value)
            cands = self._truth.get(key)
            if not cands:
                continue
            p = t.landmark.pose.pose.position
            est = np.array([p.x, p.y, p.z])
            truth = [(self._c @ np.append(q, 1.0))[:3] for q in cands]
            d = [est - q for q in truth]
            if key[0] in self._skip_z:
                d = [np.array([v[0], v[1], 0.0]) for v in d]
            err = min(float(np.linalg.norm(v)) for v in d)
            rows.append((t.landmark.id, key, t.retained, err))
        return rows

    def _evaluate(self):
        if self._vehicle is None:
            return
        yaw = math.degrees(math.atan2(self._c[1, 0], self._c[0, 0]))
        line = [
            f"t={time.monotonic() - self._t0:5.0f}s pos=({self._vehicle[0]:5.1f},{self._vehicle[1]:5.1f}) drift={yaw:5.1f}deg"
        ]
        row = [round(time.monotonic() - self._t0, 1), *self._vehicle, round(yaw, 2)]
        for lab in self._labels:
            msg = self._maps[lab]
            rows = self._errors(msg) if msg else []
            ret = [r[3] for r in rows if r[2]]
            allv = [r[3] for r in rows]
            rm = float(np.mean(ret)) if ret else float("nan")
            rx = float(np.max(ret)) if ret else float("nan")
            am = float(np.mean(allv)) if allv else float("nan")
            line.append(
                f"{lab}: remembered {rm:4.2f}/{rx:4.2f} m (mean/max, n={len(ret)}) all {am:4.2f} m n={len(allv)}"
            )
            row += [rm, rx, am, len(allv)]
        self.get_logger().info(" | ".join(line))
        self._w.writerow(row)
        self._csv.flush()

    def report(self):
        """Per-landmark table of the latest maps."""
        for lab in self._labels:
            msg = self._maps[lab]
            if not msg:
                continue
            print(f"--- {lab}")
            for lid, key, retained, err in sorted(
                self._errors(msg), key=lambda r: -r[3]
            ):
                name = TYPE_NAMES.get(key[0], str(key[0]))
                print(
                    f"  #{lid:3d} {name}/{key[1]:<2d} {'retained' if retained else 'live    '} {err:5.2f} m"
                )


def main():
    rclpy.init()
    node = GraphEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()


if __name__ == "__main__":
    main()
