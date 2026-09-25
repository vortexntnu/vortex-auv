#!/usr/bin/env python3
"""Simulator tool: odometry that drifts, and detections in its frame.

The simulator's odometry is perfect, so the smoothing backend has nothing to
correct. This node plays a drifting state estimator:

- odom_in (true) -> odom_out: every step of the true motion is replayed with
  an extra yaw of drift_yaw_deg_per_m per metre and the distance scaled by
  (1 + scale_error).
- landmarks_in (true world positions, e.g. the dummy publisher) ->
  landmarks_out: the same positions in the drifted odom frame, i.e. where a
  camera on the vehicle would put them with the drifted pose.
- drift (PoseStamped): odom_drift <- world, for evaluation.

Run the dummy publisher with use_field_of_view on the true odometry and
topic landmarks_true; landmark_server on odom_out and landmarks_out.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.msg import LandmarkArray


def quat_to_rot(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat(r):
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(
        math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2]
    )
    y = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0]
    )
    z = math.copysign(
        math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1]
    )
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return w / n, x / n, y / n, z / n


def pose_to_mat(p):
    t = np.eye(4)
    t[:3, :3] = quat_to_rot(
        (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
    )
    t[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return t


def mat_to_pose(t, p):
    p.position.x, p.position.y, p.position.z = (float(v) for v in t[:3, 3])
    w, x, y, z = rot_to_quat(t[:3, :3])
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = w, x, y, z


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    t = np.eye(4)
    t[:2, :2] = [[c, -s], [s, c]]
    return t


class DriftInjector(Node):
    def __init__(self):
        super().__init__("drift_injector")
        self.declare_parameter("drift_yaw_deg_per_m", 0.5)
        self.declare_parameter("scale_error", 0.0)
        self.declare_parameter("odom_in", "/nautilus/odom")
        self.declare_parameter("odom_out", "/nautilus/odom_drift")
        self.declare_parameter("landmarks_in", "/nautilus/landmarks_true")
        self.declare_parameter("landmarks_out", "/nautilus/landmarks_drift")
        self.declare_parameter("frame_id", "nautilus/odom")
        g = self.get_parameter
        self._drift = math.radians(g("drift_yaw_deg_per_m").value)
        self._scale = g("scale_error").value
        self._frame = g("frame_id").value

        self._true_prev = None
        self._odom = None  # drifted pose (4x4)
        self._c = np.eye(4)  # odom_drift <- world
        self._travelled = 0.0

        self._odom_pub = self.create_publisher(
            Odometry, g("odom_out").value, qos_profile_sensor_data
        )
        self._lm_pub = self.create_publisher(
            LandmarkArray, g("landmarks_out").value, qos_profile_sensor_data
        )
        self._drift_pub = self.create_publisher(PoseStamped, "/nautilus/drift", 10)
        self.create_subscription(
            Odometry, g("odom_in").value, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            LandmarkArray,
            g("landmarks_in").value,
            self._on_landmarks,
            qos_profile_sensor_data,
        )
        self.create_timer(10.0, self._log)

    def _on_odom(self, msg):
        true = pose_to_mat(msg.pose.pose)
        if self._true_prev is None:
            self._odom = true.copy()
        else:
            rel = np.linalg.inv(self._true_prev) @ true
            dist = float(np.linalg.norm(rel[:3, 3]))
            rel[:3, 3] *= 1.0 + self._scale
            self._odom = self._odom @ rel @ rot_z(self._drift * dist)
            self._travelled += dist
        self._true_prev = true
        self._c = self._odom @ np.linalg.inv(true)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self._frame
        out.child_frame_id = msg.child_frame_id
        mat_to_pose(self._odom, out.pose.pose)
        out.pose.covariance = msg.pose.covariance
        out.twist = msg.twist
        self._odom_pub.publish(out)

        d = PoseStamped()
        d.header = out.header
        mat_to_pose(self._c, d.pose)
        self._drift_pub.publish(d)

    def _on_landmarks(self, msg):
        out = LandmarkArray()
        out.header = msg.header
        out.header.frame_id = self._frame
        for lm in msg.landmarks:
            t = self._c @ pose_to_mat(lm.pose.pose)
            mat_to_pose(t, lm.pose.pose)
            lm.header.frame_id = self._frame
            out.landmarks.append(lm)
        self._lm_pub.publish(out)

    def _log(self):
        yaw = math.degrees(math.atan2(self._c[1, 0], self._c[0, 0]))
        self.get_logger().info(
            f"travelled {self._travelled:.1f} m, drift: yaw {yaw:.1f} deg, "
            f"offset ({self._c[0, 3]:.2f}, {self._c[1, 3]:.2f}) m"
        )


def main():
    rclpy.init()
    rclpy.spin(DriftInjector())


if __name__ == "__main__":
    main()
