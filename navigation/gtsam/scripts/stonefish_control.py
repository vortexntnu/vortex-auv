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
            result = future.result()
            self.mode = result.current_operation_mode.operation_mode
            self.killed = result.killswitch_status
            self.synchronized = True
        finally:
            self.pending = None

    def on_mode(self, msg):
        self.mode = msg.operation_mode
        self.commands.clear()

    def on_kill(self, msg):
        self.killed = msg.data
        self.commands.clear()

    def on_joy(self, msg):
        self.joy_received = time.monotonic()

    def on_command(self, source, msg):
        values = np.r_[vector(msg.wrench.force), vector(msg.wrench.torque)]
        if np.isfinite(values).all():
            self.commands[source] = (time.monotonic(), msg)

    def on_estimate(self, msg):
        if (
            self.mount is None
            or msg.header.frame_id != 'nautilus/odom'
            or msg.child_frame_id != 'nautilus/imu_link'
        ):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.estimate_stamp:
            return
        p, r, v, w, pc, tc = body_state(
            vector(msg.pose.pose.position),
            rotation(msg.pose.pose.orientation),
            vector(msg.twist.twist.linear),
            vector(msg.twist.twist.angular),
            np.array(msg.pose.covariance).reshape(6, 6),
            np.array(msg.twist.covariance).reshape(6, 6),
            *self.mount,
        )
        if not all(np.isfinite(x).all() for x in (p, r, v, w, pc, tc)):
            return
        pose = PoseWithCovarianceStamped()
        pose.header = copy.deepcopy(msg.header)
        (
            pose.pose.pose.position.x,
            pose.pose.pose.position.y,
            pose.pose.pose.position.z,
        ) = p.tolist()
        q = Rotation.from_matrix(r).as_quat().tolist()
        (
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w,
        ) = q
        pose.pose.covariance = pc.ravel().tolist()
        twist = TwistWithCovarianceStamped()
        twist.header = copy.deepcopy(msg.header)
        twist.header.frame_id = 'nautilus/base_link'
        (
            twist.twist.twist.linear.x,
            twist.twist.twist.linear.y,
            twist.twist.twist.linear.z,
        ) = v.tolist()
        (
            twist.twist.twist.angular.x,
            twist.twist.twist.angular.y,
            twist.twist.twist.angular.z,
        ) = w.tolist()
        twist.twist.covariance = tc.ravel().tolist()
        self.pose.publish(pose)
        self.twist.publish(twist)
        self.estimate_stamp, self.estimate_received = stamp, time.monotonic()

    def output(self):
        now = time.monotonic()
        source = 'manual' if self.mode == OperationMode.MANUAL else 'dp'
        receipt, command = self.commands.get(source, (-float('inf'), None))
        stamp_age = self.get_clock().now().nanoseconds * 1e-9 - self.estimate_stamp
        age = (
            max(now - self.estimate_received, stamp_age)
            if stamp_age >= -0.02
            else float('inf')
        )
        allowed = self.synchronized and command_allowed(
            self.mode, self.killed, now - receipt, age, now - self.joy_received
        )
