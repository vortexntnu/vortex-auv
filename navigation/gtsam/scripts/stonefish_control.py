#!/usr/bin/env python3
"""Convert estimated IMU state for DP and guard commands before allocation."""

import copy
import time

import numpy as np
import rclpy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    WrenchStamped,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from stonefish_math import body_state, command_allowed
from tf2_ros import Buffer, TransformException, TransformListener
from vortex_msgs.msg import OperationMode
from vortex_msgs.srv import GetOperationMode


def vector(v):
    return np.array([v.x, v.y, v.z])


def rotation(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


class Control(Node):
    def __init__(self):
        super().__init__('gtsam_control_adapter')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.mount = None
        self.mode, self.killed = OperationMode.MANUAL, True
        self.estimate_stamp = -float('inf')
        self.estimate_received = self.joy_received = -float('inf')
        self.commands = {}
        self.pose = self.create_publisher(
            PoseWithCovarianceStamped, 'pose', qos_profile_sensor_data
        )
        self.twist = self.create_publisher(
            TwistWithCovarianceStamped, 'twist', qos_profile_sensor_data
        )
        self.wrench = self.create_publisher(
            WrenchStamped, 'wrench_input', qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, 'gtsam/odom', self.on_estimate, qos_profile_sensor_data
        )
        self.create_subscription(Bool, 'killswitch', self.on_kill, 1)
        self.create_subscription(OperationMode, 'operation_mode', self.on_mode, 1)
        self.create_subscription(Joy, 'joy', self.on_joy, qos_profile_sensor_data)
        for source in ('manual', 'dp'):
            self.create_subscription(
                WrenchStamped,
                'gtsam/command/' + source,
                lambda msg, s=source: self.on_command(s, msg),
                qos_profile_sensor_data,
            )
        self.client = self.create_client(GetOperationMode, 'get_operation_mode')
        self.pending = None
        self.synchronized = False
        self.create_timer(0.1, self.setup)
        self.create_timer(0.01, self.output)

    def setup(self):
        if self.mount is None:
            try:
                t = self.buffer.lookup_transform(
                    'nautilus/base_link', 'nautilus/imu_link', rclpy.time.Time()
                ).transform
                self.mount = vector(t.translation), rotation(t.rotation)
            except TransformException:
                pass
        if (
            not self.synchronized
            and self.pending is None
            and self.client.service_is_ready()
        ):
            self.pending = self.client.call_async(GetOperationMode.Request())
            self.pending.add_done_callback(self.initial_mode)

    def initial_mode(self, future):
        try:
