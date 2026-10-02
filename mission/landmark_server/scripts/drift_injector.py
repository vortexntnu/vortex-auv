#!/usr/bin/env python3
"""Simulator tool: odometry that drifts, and detections in its frame.

The simulator's odometry is perfect, so the smoothing backend has nothing to
correct. This node plays a drifting state estimator:

- odom_in (true) -> odom_out: every step of the true motion is replayed
  with the errors of an IMU + DVL estimator:
  - yaw, per metre: drift_yaw_deg_per_m (a simple distance-based drift);
  - yaw, over time (gyro): a constant bias gyro_bias_deg_per_h, the Earth
    rate earth_rate_deg_per_h (a filter that does not compensate it), a
    wandering bias (first-order Gauss-Markov, std gyro_gm_std_deg_per_h,
    correlation time gyro_gm_tau_s) and angle random walk gyro_arw_deg_per_sqrt_h;
  - horizontal steps (DVL): scaled by (1 + scale_error), rotated by
    dvl_misalignment_deg, plus a position random walk dvl_pos_rw_m_per_sqrt_s.
  Depth, roll and pitch stay true (pressure sensor and gravity).
  The random parts use odom_seed, so a run can be repeated.
- landmarks_in -> landmarks_out: the same positions in the drifted odom
  frame, i.e. where a camera on the vehicle would put them with the drifted
  pose. Input in world_frame is used as is (the dummy publisher); input in
  another frame (a real detector in the camera frame) is first put in
  world_frame with the true TF at its stamp.
- drift (PoseStamped): odom_drift <- world, for evaluation.
- TF world_frame -> frame_id (when they differ): the drift, so what is
  built on the drifted data is drawn where it is in the world.
- Optional camera noise on the detections (the dummy's are exact): along
  the line of sight from the true vehicle position std = depth_std_base +
  depth_std_per_m * d, across it lateral_std_base + lateral_std_per_m * d,
  and a constant range bias per landmark (bias_frac_std * d, drawn once per
  landmark: a detector that is consistently wrong about one object). The
  position covariance of the random part is written into the message (the
  bias is not in it, as a real detector would not know it). Only input in
  world_frame gets noise: a real detector has its own.

Run the dummy publisher with use_field_of_view on the true odometry and
topic landmarks_true; landmark_server on odom_out and landmarks_out.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
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
        self.declare_parameter("gyro_bias_deg_per_h", 0.0)
        self.declare_parameter("earth_rate_deg_per_h", 0.0)
        self.declare_parameter("gyro_gm_std_deg_per_h", 0.0)
        self.declare_parameter("gyro_gm_tau_s", 600.0)
        self.declare_parameter("gyro_arw_deg_per_sqrt_h", 0.0)
        self.declare_parameter("dvl_misalignment_deg", 0.0)
        self.declare_parameter("dvl_pos_rw_m_per_sqrt_s", 0.0)
        self.declare_parameter("odom_seed", 1)
        self.declare_parameter("odom_in", "/nautilus/odom")
        self.declare_parameter("odom_out", "/nautilus/odom_drift")
        self.declare_parameter("landmarks_in", "/nautilus/landmarks_true")
        self.declare_parameter("landmarks_out", "/nautilus/landmarks_drift")
        # Frame of the drifted odometry and detections. A name other than
        # world_frame gets a TF world_frame -> frame_id (the drift), so the
        # map built on the drifted data is drawn where it is in the world.
        self.declare_parameter("frame_id", "nautilus/odom")
        # The simulator's true odom frame (TF), for detections in other frames.
        self.declare_parameter("world_frame", "nautilus/odom")
        self.declare_parameter("noise", False)
        self.declare_parameter("depth_std_base", 0.05)
        self.declare_parameter("depth_std_per_m", 0.03)
        self.declare_parameter("lateral_std_base", 0.02)
        self.declare_parameter("lateral_std_per_m", 0.005)
        self.declare_parameter("bias_frac_std", 0.0)
        self.declare_parameter("noise_seed", 1)
        g = self.get_parameter
        self._noise = g("noise").value
        self._ds = (g("depth_std_base").value, g("depth_std_per_m").value)
        self._ls = (g("lateral_std_base").value, g("lateral_std_per_m").value)
        self._bias_std = g("bias_frac_std").value
        self._rng = np.random.default_rng(g("noise_seed").value)
        self._bias = {}
        self._drift = math.radians(g("drift_yaw_deg_per_m").value)
        self._scale = g("scale_error").value
        deg_per_h = math.radians(1.0) / 3600.0  # [rad/s]
        self._gyro_bias = (
            g("gyro_bias_deg_per_h").value + g("earth_rate_deg_per_h").value
        ) * deg_per_h
        self._gm_std = g("gyro_gm_std_deg_per_h").value * deg_per_h
        self._gm_tau = max(g("gyro_gm_tau_s").value, 1e-3)
        # ARW [deg/sqrt(h)] -> [rad/sqrt(s)]
        self._arw = math.radians(g("gyro_arw_deg_per_sqrt_h").value) / 60.0
        self._misalign = math.radians(g("dvl_misalignment_deg").value)
        self._pos_rw = g("dvl_pos_rw_m_per_sqrt_s").value
        self._odom_rng = np.random.default_rng(g("odom_seed").value)
        self._gm_bias = 0.0  # [rad/s]
        self._stamp_prev = None
        self._elapsed = 0.0
        self._frame = g("frame_id").value
        self._world_frame = g("world_frame").value
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self, spin_thread=True)
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._frame != self._world_frame else None
        )

        self._true_prev = None
        self._odom = None  # drifted pose (4x4)
        self._c = np.eye(4)  # odom_drift <- world
        self._travelled = 0.0

        # Reliable publishers reach both reliable and best-effort subscribers.
        self._odom_pub = self.create_publisher(Odometry, g("odom_out").value, 10)
        self._lm_pub = self.create_publisher(
            LandmarkArray, g("landmarks_out").value, 10
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
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self._true_prev is None:
            self._odom = true.copy()
        else:
            dt = min(max(stamp - self._stamp_prev, 0.0), 1.0)
            rel = np.linalg.inv(self._true_prev) @ true
            dist = float(np.linalg.norm(rel[:3, 3]))
            rel[:2, 3] = self._dvl_step(rel[:2, 3], dt)
            yaw = self._drift * dist + self._gyro_step(dt)
            self._odom = self._odom @ rel @ rot_z(yaw)
            # Depth comes from the pressure sensor: it does not drift.
            self._odom[2, 3] = true[2, 3]
            self._travelled += dist
            self._elapsed += dt
        self._true_prev = true
        self._stamp_prev = stamp
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

        if self._tf_broadcaster is not None:
            # world <- odom_drift: a point of the drifted frame in the world.
            inv = np.linalg.inv(self._c)
            p = Pose()
            mat_to_pose(inv, p)
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = self._world_frame
            tf.child_frame_id = self._frame
            tf.transform.translation.x = p.position.x
            tf.transform.translation.y = p.position.y
            tf.transform.translation.z = p.position.z
            tf.transform.rotation = p.orientation
            self._tf_broadcaster.sendTransform(tf)

    def _gyro_step(self, dt):
        """Heading error [rad] over dt: bias, Gauss-Markov bias, ARW."""
        if dt <= 0.0:
            return 0.0
        a = math.exp(-dt / self._gm_tau)
        self._gm_bias = (
            a * self._gm_bias
            + self._gm_std * math.sqrt(1.0 - a * a) * self._odom_rng.standard_normal()
        )
        arw = self._arw * math.sqrt(dt) * self._odom_rng.standard_normal()
        return (self._gyro_bias + self._gm_bias) * dt + arw

    def _dvl_step(self, xy, dt):
        """Horizontal step in the body frame: scale, misalignment, noise."""
        c, s = math.cos(self._misalign), math.sin(self._misalign)
        out = (1.0 + self._scale) * np.array(
            [c * xy[0] - s * xy[1], s * xy[0] + c * xy[1]]
        )
        if self._pos_rw > 0.0 and dt > 0.0:
            out += self._pos_rw * math.sqrt(dt) * self._odom_rng.standard_normal(2)
        return out

    def _on_landmarks(self, msg):
        in_world = msg.header.frame_id in ("", self._world_frame)
        world_from_msg = np.eye(4)
        if not in_world:
            try:
                tf = self._tf.lookup_transform(
                    self._world_frame,
                    msg.header.frame_id,
                    Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as ex:
                self.get_logger().warn(
                    f"No TF {self._world_frame} <- {msg.header.frame_id}: {ex}",
                    throttle_duration_sec=2.0,
                )
                return
            r = tf.transform.rotation
            t = tf.transform.translation
            world_from_msg[:3, :3] = quat_to_rot((r.w, r.x, r.y, r.z))
            world_from_msg[:3, 3] = (t.x, t.y, t.z)
        out = LandmarkArray()
        out.header = msg.header
        out.header.frame_id = self._frame
        for lm in msg.landmarks:
            world = world_from_msg @ pose_to_mat(lm.pose.pose)
            if in_world and self._noise and self._true_prev is not None:
                self._add_noise(lm, world)
            t = self._c @ world
            mat_to_pose(t, lm.pose.pose)
            lm.header.frame_id = self._frame
            out.landmarks.append(lm)
        self._lm_pub.publish(out)

    def _add_noise(self, lm, world):
        """Camera noise in the world frame; covariance into the drifted frame."""
        p = world[:3, 3]
        ray = p - self._true_prev[:3, 3]
        d = float(np.linalg.norm(ray))
        if d < 1e-6:
            return
        u = ray / d
        key = (lm.type.value, lm.subtype.value, *np.round(p, 1))
        if key not in self._bias:
            self._bias[key] = (
                self._rng.normal(0.0, self._bias_std) if self._bias_std > 0 else 0.0
            )
        sd = self._ds[0] + self._ds[1] * d
        sl = self._ls[0] + self._ls[1] * d
        cov = sl * sl * np.eye(3) + (sd * sd - sl * sl) * np.outer(u, u)
        noise = self._rng.multivariate_normal(np.zeros(3), cov)
        world[:3, 3] = p + noise + self._bias[key] * d * u
        r = self._c[:3, :3]
        cov_out = r @ cov @ r.T
        for i in range(3):
            for j in range(3):
                lm.pose.covariance[6 * i + j] = float(cov_out[i, j])

    def _log(self):
        yaw = math.degrees(math.atan2(self._c[1, 0], self._c[0, 0]))
        pos_err = 0.0
        if self._odom is not None and self._true_prev is not None:
            pos_err = float(np.linalg.norm(self._odom[:2, 3] - self._true_prev[:2, 3]))
        self.get_logger().info(
            f"{self._elapsed:.0f} s, travelled {self._travelled:.1f} m, "
            f"drift: yaw {yaw:.1f} deg, position {pos_err:.2f} m"
        )


def main():
    rclpy.init()
    rclpy.spin(DriftInjector())


if __name__ == "__main__":
    main()
