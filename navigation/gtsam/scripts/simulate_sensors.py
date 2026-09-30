#!/usr/bin/env python3
"""Publish synthetic STIM300/Nucleus measurements, a simulation clock and truth."""

import math
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_model import (
    DVL_OFFSET,
    DVL_YAW,
    IMU_OFFSET,
    SensorModel,
    bottom_lock_available,
    ideal_measurements,
    quaternion_from_rotation,
    trajectory,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


class SensorSimulator(Node):
    """Use a fixed simulation timestep so each seed produces identical samples."""

    def __init__(self):
        super().__init__("gtsam_sensor_simulator")
        self.kind = self.declare_parameter("trajectory", "turn").value
        self.duration = self.declare_parameter(
            "duration", 100.0 if self.kind == "square_barrel_roll" else 60.0
        ).value
        self.rate = self.declare_parameter("imu_rate", 1000.0).value
        self.dvl_rate = self.declare_parameter("dvl_rate", 8.0).value
        self.dropout_start = self.declare_parameter(
            "dropout_start", -1.0 if self.kind == "square_barrel_roll" else 20.0
        ).value
        self.dropout_end = self.declare_parameter(
            "dropout_end", -1.0 if self.kind == "square_barrel_roll" else 25.0
        ).value
        self.dvl_max_tilt_deg = self.declare_parameter("dvl_max_tilt_deg", 30.0).value
        bottom_lock_available(np.eye(3), self.dvl_max_tilt_deg)  # Validate at startup.
        prefix = self.declare_parameter("frame_prefix", "nautilus").value
        self.prefix = prefix + "/" if prefix else ""
        self.model = SensorModel(
            seed=self.declare_parameter("seed", 42).value,
            profile=self.declare_parameter(
                "imu_profile", "stim300_10g_provisional"
            ).value,
            noise=self.declare_parameter("noise", True).value,
            stress_scale=self.declare_parameter("stress_scale", 1.0).value,
        )
        if (
            self.rate <= 0
            or self.dvl_rate <= 0
            or self.dvl_rate > self.rate
            or self.duration <= 0
        ):
            raise ValueError("Invalid simulation rates/duration")
        trajectory(0.0, self.kind)  # Validate before starting timers.
        self.imu_pub = self.create_publisher(
            Imu, "imu/data_raw", qos_profile_sensor_data
        )
        self.dvl_pub = self.create_publisher(
            TwistWithCovarianceStamped, "dvl/twist", qos_profile_sensor_data
        )
        self.lock_pub = self.create_publisher(
            Bool,
            "dvl/bottom_lock",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.truth_pub = self.create_publisher(
            Odometry, "gtsam/truth", qos_profile_sensor_data
        )
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for child, offset, yaw in (
            ("imu_link", IMU_OFFSET, 0.0),
            ("dvl_link", DVL_OFFSET, DVL_YAW),
        ):
            transform = TransformStamped()
            transform.header.frame_id = self.prefix + "base_link"
            transform.child_frame_id = self.prefix + child
