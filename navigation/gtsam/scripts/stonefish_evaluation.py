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
