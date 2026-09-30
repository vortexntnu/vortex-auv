#!/usr/bin/env python3
"""Native Stonefish ideal sensors -> noisy IMU and valid bottom-track velocity.

No odometry/truth subscriptions. Noise is applied here exactly once; the dedicated
scene disables native noise. Wall timestamps are retained, not synthesized.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_model import SensorModel
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from stonefish_math import bottom_track_tilt_valid, bottom_track_valid
from stonefish_ros2.msg import DVL


class Sensors(Node):
    def __init__(self):
        super().__init__('stonefish_gtsam_sensors')
        self.model = SensorModel(seed=self.declare_parameter('seed', 42).value)
        self.max_tilt = self.declare_parameter('dvl_max_tilt_deg', 25.0).value
        if not np.isfinite(self.max_tilt) or not 0 < self.max_tilt <= 90:
            raise ValueError('dvl_max_tilt_deg must be in (0, 90]')
        self.attitude_stamp = None
        self.tilt_valid = False
        self.imu = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.dvl = self.create_publisher(
            TwistWithCovarianceStamped, 'dvl/twist', qos_profile_sensor_data
        )
        self.lock = self.create_publisher(
            Bool,
            'dvl/bottom_lock',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(
            Imu, 'stonefish/imu', self.on_imu, qos_profile_sensor_data
        )
        self.create_subscription(DVL, 'dvl/sim', self.on_dvl, qos_profile_sensor_data)

    def on_imu(self, msg):
        # Native attitude supplies geometry for simulated availability only.
        # It never enters GTSAM, DP feedback, or measurement noise correction.
        q = msg.orientation
        self.tilt_valid = bottom_track_tilt_valid([q.x, q.y, q.z, q.w], self.max_tilt)
        self.attitude_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        a = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )
        w = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        # Each native sample represents a 1 ms physical step. Scheduling jitter
        # must not change its noise variance; timing validity is tested separately.
        a, w = self.model.imu(a, w, 0.001)
        (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ) = a.tolist()
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
            w.tolist()
        )
        msg.orientation_covariance = [-1.0] + [0.0] * 8
        msg.linear_acceleration_covariance = (
            (np.eye(3) * self.model.accel_density**2 / 0.001).ravel().tolist()
        )
        msg.angular_velocity_covariance = (
            (np.eye(3) * self.model.gyro_density**2 / 0.001).ravel().tolist()
        )
        self.imu.publish(msg)

    def on_dvl(self, msg):
        v = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        # The installed ROS bridge encodes native status 0/2 as positive altitude,
        # and water-only/no-ping as -1, even though it still publishes velocity.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        fresh_attitude = (
            self.attitude_stamp is not None and abs(stamp - self.attitude_stamp) <= 0.02
        )
        valid = (
            bottom_track_valid(msg.altitude, v) and fresh_attitude and self.tilt_valid
        )
        self.lock.publish(Bool(data=valid))
