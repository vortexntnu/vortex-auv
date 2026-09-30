#!/usr/bin/env python3
"""Evaluation only: time-aligned IMU-origin truth and Foxglove vehicle trails.

The first matched truth sample defines the local position and heading gauge.
No estimated pose is used to fit/alignment-correct truth. The first-output gauge
is approximate to initialization (one publication interval); it is not an
absolute-position or absolute-heading validation.
"""

import copy
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_model import IMU_OFFSET, rotation_z
from stonefish_math import interpolate
from visualization_msgs.msg import Marker, MarkerArray


def vector(v):
    return np.array([v.x, v.y, v.z])


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Evaluation(Node):
    def __init__(self):
        super().__init__('stonefish_gtsam_evaluation')
        self.history = deque(maxlen=10000)
        self.pending = deque(maxlen=250)
        self.anchor = None
        self.trails = [deque(maxlen=1500), deque(maxlen=1500)]
        self.last_visual = 0.0
        self.truth = self.create_publisher(
            Odometry, 'gtsam/truth', qos_profile_sensor_data
        )
        self.visual = self.create_publisher(MarkerArray, 'gtsam/vehicles', 1)
        self.create_subscription(
            Odometry, 'stonefish/truth', self.on_truth, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, 'gtsam/odom', self.on_estimate, qos_profile_sensor_data
        )

    def on_truth(self, msg):
        t = stamp(msg)
        if self.history and t <= self.history[-1][0]:
            return
        q = msg.pose.pose.orientation
        self.history.append(
            (
                t,
                vector(msg.pose.pose.position),
                np.array([q.x, q.y, q.z, q.w]),
                vector(msg.twist.twist.linear),
                vector(msg.twist.twist.angular),
            )
        )
        self.match()

    def on_estimate(self, msg):
        self.pending.append(msg)
        self.match()

    def match(self):
        while self.pending and len(self.history) >= 2:
            msg = self.pending[0]
            t = stamp(msg)
            if t > self.history[-1][0]:
                return
            self.pending.popleft()
            while len(self.history) > 2 and self.history[1][0] < t:
                self.history.popleft()
            state = interpolate(self.history[0], self.history[1], t)
            if state is None:
                continue
            _, p, q, v, w = state
            r = Rotation.from_quat(q).as_matrix()
            # Truth sensor is mounted directly at IMU origin in the scenario.
            if self.anchor is None:
                yaw = np.arctan2(r[1, 0], r[0, 0])
                self.anchor = p.copy(), rotation_z(-yaw)
            origin, world = self.anchor
            p, r = world @ (p - origin), world @ r
            truth = Odometry()
            truth.header = copy.deepcopy(msg.header)
            truth.child_frame_id = msg.child_frame_id
            (
                truth.pose.pose.position.x,
                truth.pose.pose.position.y,
                truth.pose.pose.position.z,
            ) = p.tolist()
            q = Rotation.from_matrix(r).as_quat().tolist()
            (
                truth.pose.pose.orientation.x,
                truth.pose.pose.orientation.y,
                truth.pose.pose.orientation.z,
                truth.pose.pose.orientation.w,
            ) = q
            (
                truth.twist.twist.linear.x,
                truth.twist.twist.linear.y,
                truth.twist.twist.linear.z,
            ) = v.tolist()
            (
                truth.twist.twist.angular.x,
                truth.twist.twist.angular.y,
                truth.twist.twist.angular.z,
            ) = w.tolist()
            self.truth.publish(truth)
            if t - self.last_visual >= 0.1:
                self.draw(msg, truth)
                self.last_visual = t

    def draw(self, estimate, truth):
        markers = MarkerArray()
        for i, msg in enumerate((estimate, truth)):
            pose = copy.deepcopy(msg.pose.pose)
            q = pose.orientation
            body = vector(pose.position) - Rotation.from_quat(
                [q.x, q.y, q.z, q.w]
            ).apply(IMU_OFFSET)
            pose.position.x, pose.position.y, pose.position.z = body.tolist()
            marker = Marker()
            marker.header = copy.deepcopy(msg.header)
            marker.ns, marker.id = 'estimate' if i == 0 else 'truth', i
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose = pose
            marker.scale.x, marker.scale.y, marker.scale.z = 0.95, 0.65, 0.3
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                (0.1, 0.5, 1.0, 0.65) if i == 0 else (1.0, 0.25, 0.1, 0.4)
            )
            markers.markers.append(marker)
            arrow = copy.deepcopy(marker)
            arrow.id, arrow.type = i + 2, Marker.ARROW
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 1.0, 0.06, 0.06
            markers.markers.append(arrow)
            self.trails[i].append(
                Point(x=float(body[0]), y=float(body[1]), z=float(body[2]))
            )
            line = copy.deepcopy(marker)
            line.id, line.type = i + 4, Marker.LINE_STRIP
            line.pose.position = Point()
            line.pose.orientation.x = line.pose.orientation.y = (
                line.pose.orientation.z
            ) = 0.0
            line.pose.orientation.w = 1.0
            line.scale.x = 0.025
            line.points = list(self.trails[i])
            markers.markers.append(line)
        self.visual.publish(markers)


def main():
    rclpy.init()
    node = Evaluation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
