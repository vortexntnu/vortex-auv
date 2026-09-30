"""Run actual headless physics and exercise joystick -> estimated-state DP.

Run from a sourced workspace on an unused ROS_DOMAIN_ID. Never sends commands
to another domain. Owns and shuts down only the launch it starts.
"""

import argparse
import json
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    WrenchStamped,
)
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool
from stonefish_ros2.msg import DVL
from vortex_msgs.msg import ReferenceFilterQuat, ThrusterForces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rendering', action='store_true')
    parser.add_argument('--bridge-port', type=int, default=0)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '169':
        raise RuntimeError('This audit requires isolated ROS_DOMAIN_ID=169')
    output = Path(__file__).resolve().parents[1] / '.deps/audit'
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('stonefish_closed_loop_audit')
    records, latest = {}, {}

    def collect(msg, name):
        latest[name] = msg
        records.setdefault(name, []).append((time.monotonic(), msg))

    for name, topic, typ in (
        ('imu', 'imu/data_raw', Imu),
        ('dvl', 'dvl/twist', TwistWithCovarianceStamped),
        ('native_dvl', 'dvl/sim', DVL),
        ('odom', 'gtsam/odom', Odometry),
        ('truth', 'gtsam/truth', Odometry),
        ('native', 'stonefish/truth', Odometry),
        ('pose', 'pose', PoseWithCovarianceStamped),
        ('wrench', 'wrench_input', WrenchStamped),
        ('ref', 'guidance/dp_quat', ReferenceFilterQuat),
        ('thrusters', 'thruster_forces', ThrusterForces),
    ):
        node.create_subscription(
            typ,
            '/nautilus/' + topic,
            lambda m, n=name: collect(m, n),
            qos_profile_sensor_data,
        )
    node.create_subscription(
        DiagnosticArray,
        '/nautilus/gtsam/status',
        lambda m: collect(m, 'status'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        Bool,
        '/nautilus/dvl/bottom_lock',
        lambda m: collect(m, 'lock'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    joy = node.create_publisher(Joy, '/nautilus/joy', 10)
    command = Joy(axes=[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0], buttons=[0] * 12)

    def advance(seconds, heartbeat=True):
        end = time.monotonic() + seconds
        next_joy = 0.0
        while time.monotonic() < end:
            if heartbeat and time.monotonic() >= next_joy:
                command.header.stamp = node.get_clock().now().to_msg()
                joy.publish(command)
                next_joy = time.monotonic() + 0.01
