#!/usr/bin/env python3
"""Simulator tool: map error with and without the smoothing backend.

Compares two object_maps (landmark_server with graph.enable true and false,
fed the same drifting data from drift_injector) to the truth. The truth of a
landmark is its true world position (landmarks_true) expressed in the drifted
odom frame (drift_injector's /nautilus/drift): where it really is relative to
the vehicle, in the frame the vehicle navigates in.

Every second: mean/max error of remembered (not live) and all landmarks per
map, and id swaps: a map id that moves to another true object (a wrong
take-over). With truth_seed >= 0 the truth is the dummy's course layout for
that seed (use it with noisy or unstable dummy profiles, whose
landmarks_true carry the noise).

Trajectory: the true path (/nautilus/odom, published as
/landmark_eval/true_path), the graph's smoothed keyframes in the graph frame
(graph/start_frame_path, green in the Foxglove layout) and the raw drifting
odometry keyframes (graph/odom_path, orange), compared at each keyframe's
stamp. The graph frame is odom at the first keyframe; the drift injector
starts on the truth, so it is the true world (the truth is put through the
drift at the first keyframe in case the graph started later). The graph path
should lie on the true path, while the odometry drifts away from it over
time. Errors are horizontal (depth does not drift).

For Foxglove: /landmark_eval/markers (the truth as green spheres, a line from
each map landmark to its true object, per map), and per map
/landmark_eval/<label>/{remembered_mean,all_mean,count,swaps} plus
/landmark_eval/drift_yaw_deg and /landmark_eval/traj/{graph,odom}_{mean,max}
(std_msgs/Float64) for plots. Also written to csv (param csv, relative to the working
directory).
"""

import csv
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from vortex_msgs.msg import LandmarkArray, LandmarkTrackArray, LandmarkType

# Line colours per map, in label order.
COLOURS = [
    (0.18, 0.5, 0.93),
    (0.9, 0.2, 0.2),
    (0.95, 0.65, 0.1),
    (0.6, 0.3, 0.8),
    (0.2, 0.7, 0.7),
    (0.5, 0.5, 0.5),
]

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
        self.declare_parameter("truth_seed", -1)
        self.declare_parameter("frame_id", "nautilus/odom")
        self.declare_parameter("graph_frame_id", "")
        self.declare_parameter(
            "graph_path", "/nautilus/landmark_server/graph/start_frame_path"
        )
        self.declare_parameter(
            "odom_path", "/nautilus/landmark_server/graph/odom_path"
        )
        g = self.get_parameter
        self._labels = list(g("labels").value)
        self._maps = dict.fromkeys(self._labels)
        self._truth = {}  # (type, subtype) -> list of world positions
        self._c = np.eye(4)
        self._vehicle = None
        # True trajectory: stamps [s] and world positions.
        self._true_t = []
        self._true_p = []
        self._paths = {"graph": None, "odom": None}
        # Drift (odom_drift <- world) over time, for the graph frame.
        self._drift_t = []
        self._drift_c = []
        self._skip_z = set(g("z_locked_types").value)
        self._fixed_truth = g("truth_seed").value >= 0
        if self._fixed_truth:
            from robosub_dummy_publisher import course_layout

            picks, _ = course_layout.draw_role_picks(g("truth_seed").value)
            for task in course_layout.TASKS.values():
                for lm in task.landmarks(picks):
                    key = (lm.landmark_type, lm.landmark_subtype)
                    pos = np.array(task.base_pose) + np.array(lm.offset)
                    self._truth.setdefault(key, []).append(pos)
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
        self.create_subscription(PoseStamped, "/nautilus/drift", self._on_drift, 10)
        self.create_subscription(
            Odometry, "/nautilus/odom", self._on_odom, qos_profile_sensor_data
        )
        for name, param in (("graph", "graph_path"), ("odom", "odom_path")):
            self.create_subscription(
                Path,
                g(param).value,
                lambda m, name=name: self._paths.__setitem__(name, m),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
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
                f"{lab}_swaps",
            ]
        header += ["traj_graph_mean", "traj_graph_max", "traj_odom_mean", "traj_odom_max"]
        self._w.writerow(header)
        self._frame = g("frame_id").value
        # The true path is in the graph frame (odom at the first keyframe, the
        # true world in the drift simulation): its own frame for display.
        self._graph_frame = g("graph_frame_id").value or self._frame
        self._marker_pub = self.create_publisher(
            MarkerArray, "/landmark_eval/markers", 1
        )
        self._drift_pub = self.create_publisher(
            Float64, "/landmark_eval/drift_yaw_deg", 10
        )
        self._true_path_pub = self.create_publisher(
            Path, "/landmark_eval/true_path", 1
        )
        self._traj_pubs = {
            (name, stat): self.create_publisher(
                Float64, f"/landmark_eval/traj/{name}_{stat}", 10
            )
            for name in ("graph", "odom")
            for stat in ("mean", "max")
        }
        self._value_pubs = {
            (lab, name): self.create_publisher(
                Float64, f"/landmark_eval/{lab}/{name}", 10
            )
            for lab in self._labels
            for name in ("remembered_mean", "all_mean", "count", "swaps")
        }
        self._assign = {lab: {} for lab in self._labels}  # map id -> truth
        self._swaps = dict.fromkeys(self._labels, 0)
        self._t0 = time.monotonic()
        self.create_timer(1.0, self._evaluate)

    def _on_odom(self, m):
        p = m.pose.pose.position
        self._vehicle = (p.x, p.y)
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if not self._true_t or t > self._true_t[-1]:
            self._true_t.append(t)
            self._true_p.append((p.x, p.y, p.z))

    def _on_drift(self, m):
        self._c = pose_to_mat(m.pose)
        t = Time.from_msg(m.header.stamp).nanoseconds * 1e-9
        if not self._drift_t or t > self._drift_t[-1]:
            self._drift_t.append(t)
            self._drift_c.append(self._c)

    def _true_in_graph(self, positions):
        """World positions in the graph frame: through the drift at the
        first keyframe (identity when the graph started with the drift)."""
        c = np.eye(4)
        odom = self._paths["odom"]
        if odom is not None and odom.poses and self._drift_t:
            t0 = Time.from_msg(odom.poses[0].header.stamp).nanoseconds * 1e-9
            i = int(np.searchsorted(self._drift_t, t0))
            c = self._drift_c[min(i, len(self._drift_c) - 1)]
        pts = np.c_[positions, np.ones(len(positions))]
        return (c @ pts.T).T[:, :3]

    def _publish_true_path(self):
        """The true trajectory in the graph frame, every 10 cm."""
        path = Path()
        path.header.frame_id = self._graph_frame
        path.header.stamp = self.get_clock().now().to_msg()
        if len(self._true_p) < 2:
            self._true_path_pub.publish(path)
            return
        pts = np.asarray(self._true_p)
        keep = [0]
        for i in range(1, len(pts)):
            if np.linalg.norm(pts[i] - pts[keep[-1]]) > 0.1:
                keep.append(i)
        keep.append(len(pts) - 1)
        for i, q in zip(keep, self._true_in_graph(pts[keep])):
            ps = PoseStamped()
            ps.header.frame_id = self._graph_frame
            ps.header.stamp = Time(seconds=self._true_t[i]).to_msg()
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
                float(v) for v in q
            )
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._true_path_pub.publish(path)

    def _trajectory_errors(self, name):
        """Mean and max horizontal error of a keyframe path [m]."""
        path = self._paths[name]
        if path is None or not path.poses or len(self._true_t) < 2:
            return float("nan"), float("nan")
        tt = np.asarray(self._true_t)
        tp = np.asarray(self._true_p)
        stamps = np.array(
            [Time.from_msg(p.header.stamp).nanoseconds * 1e-9 for p in path.poses]
        )
        ok = (stamps >= tt[0]) & (stamps <= tt[-1])
        if not ok.any():
            return float("nan"), float("nan")
        truth = np.stack(
            [np.interp(stamps[ok], tt, tp[:, k]) for k in range(3)], axis=1
        )
        truth = self._true_in_graph(truth)
        est = np.array(
            [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        )[ok]
        err = np.linalg.norm(est - truth[:, :2], axis=1)
        return float(err.mean()), float(err.max())

    def _on_truth(self, msg):
        if self._fixed_truth:
            return
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
            norms = [float(np.linalg.norm(v)) for v in d]
            idx = int(np.argmin(norms))
            rows.append(
                (
                    t.landmark.id,
                    key,
                    t.retained,
                    norms[idx],
                    (key, idx),
                    est,
                    truth[idx],
                )
            )
        return rows

    def _evaluate(self):
        if self._vehicle is None:
            return
        yaw = math.degrees(math.atan2(self._c[1, 0], self._c[0, 0]))
        line = [
            f"t={time.monotonic() - self._t0:5.0f}s pos=({self._vehicle[0]:5.1f},{self._vehicle[1]:5.1f}) drift={yaw:5.1f}deg"
        ]
        row = [round(time.monotonic() - self._t0, 1), *self._vehicle, round(yaw, 2)]
        self._drift_pub.publish(Float64(data=yaw))
        per_map_rows = {}
        traj = []
        for name in ("graph", "odom"):
            mean, mx = self._trajectory_errors(name)
            traj += [mean, mx]
            self._traj_pubs[(name, "mean")].publish(Float64(data=mean))
            self._traj_pubs[(name, "max")].publish(Float64(data=mx))
        line.append(
            f"path: graph {traj[0]:4.2f}/{traj[1]:4.2f} m odom {traj[2]:4.2f}/{traj[3]:4.2f} m (mean/max)"
        )
        self._publish_true_path()
        for lab in self._labels:
            msg = self._maps[lab]
            rows = self._errors(msg) if msg else []
            # A swap: the id now sits on another true object (counted when it
            # is within 0.3 m of it, so noise near the middle does not count).
            for lid, _, _, err, truth, *_ in rows:
                prev = self._assign[lab].get(lid)
                if err < 0.3:
                    if prev is not None and prev != truth:
                        self._swaps[lab] += 1
                        self.get_logger().warn(
                            f"{lab}: id {lid} moved from {prev} to {truth}"
                        )
                    self._assign[lab][lid] = truth
            ret = [r[3] for r in rows if r[2]]
            allv = [r[3] for r in rows]
            rm = float(np.mean(ret)) if ret else float("nan")
            rx = float(np.max(ret)) if ret else float("nan")
            am = float(np.mean(allv)) if allv else float("nan")
            line.append(
                f"{lab}: remembered {rm:4.2f}/{rx:4.2f} m (mean/max, n={len(ret)}) all {am:4.2f} m n={len(allv)} swaps={self._swaps[lab]}"
            )
            row += [rm, rx, am, len(allv), self._swaps[lab]]
            for name, value in (
                ("remembered_mean", rm),
                ("all_mean", am),
                ("count", len(allv)),
                ("swaps", self._swaps[lab]),
            ):
                self._value_pubs[(lab, name)].publish(Float64(data=float(value)))
            per_map_rows[lab] = rows
        self.get_logger().info(" | ".join(line))
        self._publish_markers(per_map_rows)
        self._w.writerow(row + [round(v, 3) for v in traj])
        self._csv.flush()

    def _publish_markers(self, per_map_rows):
        """Truth spheres and one line per map landmark to its true object."""
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

        truth = Marker()
        truth.header.frame_id = self._frame
        truth.header.stamp = stamp
        truth.ns = "truth"
        truth.id = 0
        truth.type = Marker.SPHERE_LIST
        truth.pose.orientation.w = 1.0
        # Small dots, so the map markers on top of them stay visible.
        truth.scale.x = truth.scale.y = truth.scale.z = 0.08
        truth.color.r, truth.color.g, truth.color.b, truth.color.a = (
            0.2,
            0.85,
            0.3,
            0.9,
        )
        for positions in self._truth.values():
            for q in positions:
                p = (self._c @ np.append(q, 1.0))[:3]
                truth.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        out.markers.append(truth)

        for i, lab in enumerate(self._labels):
            lines = Marker()
            lines.header = truth.header
            lines.ns = f"error_{lab}"
            lines.id = 0
            lines.type = Marker.LINE_LIST
            lines.pose.orientation.w = 1.0
            lines.scale.x = 0.04
            r, g, b = COLOURS[i % len(COLOURS)]
            lines.color.r, lines.color.g, lines.color.b, lines.color.a = r, g, b, 0.9
            for _, _, _, _, _, est, true in per_map_rows.get(lab, []):
                lines.points.append(
                    Point(x=float(est[0]), y=float(est[1]), z=float(est[2]))
                )
                lines.points.append(
                    Point(x=float(true[0]), y=float(true[1]), z=float(true[2]))
                )
            out.markers.append(lines)
        self._marker_pub.publish(out)

    def report(self):
        """Per-landmark table of the latest maps."""
        for lab in self._labels:
            msg = self._maps[lab]
            if not msg:
                continue
            print(f"--- {lab}")
            for lid, key, retained, err, *_ in sorted(
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
