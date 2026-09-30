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
            rclpy.spin_once(node, timeout_sec=0.001)

    def button(index):
        command.buttons[index] = 1
        advance(0.05)
        command.buttons[index] = 0
        advance(0.4)

    report = {}
    stopped = []
    with (output / 'stonefish-launch.log').open('w') as log:
        process = subprocess.Popen(
            [
                'ros2',
                'launch',
                'gtsam_navigation',
                'stonefish.launch.py',
                'rendering:=' + str(args.rendering).lower(),
                'input:=none',
                'foxglove:=' + str(args.bridge_port > 0).lower(),
                'foxglove_port:=' + str(args.bridge_port or 8765),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            for _ in range(50):
                advance(1.0)
                if 'odom' in latest and 'truth' in latest and 'pose' in latest:
                    break
            report['initialized'] = all(x in latest for x in ('odom', 'truth', 'pose'))
            print('Initialization:', report['initialized'], flush=True)
            if args.bridge_port:
                with socket.create_connection(
                    ('127.0.0.1', args.bridge_port), timeout=3
                ) as conn:
                    conn.sendall(
                        b'GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n'
                    )
                    report['foxglove_handshake'] = (
                        b'101' in conn.makefile('rb').readline().split()
                    )
                    assert report['foxglove_handshake']
            if not report['initialized']:
                if 'status' in latest:
                    report['last_status'] = str(latest['status'])
                raise RuntimeError(
                    'No usable estimator/controller state within 50 seconds'
                )
            advance(3.0)
            # B unkills; manual upward thrust via right trigger for departure.
            button(1)
            command.axes[5] = 0.0
            advance(3.0)
            command.axes[5] = 1.0
            button(3)  # Y captures estimated current pose and selects DP reference.
            hold_start = time.monotonic()
            advance(10.0)
            print('DP hold completed', flush=True)
            command.axes[1] = (
                0.15  # small forward reference change through real joystick interface
            )
            advance(2.0)
            command.axes[1] = 0.0
            command.axes[3] = -0.1
            advance(2.0)
            command.axes[3] = 0.0
            advance(12.0)
            last_reference = latest['ref']
            last_position = latest['pose'].pose.pose.position
            report['settled_estimated_tracking_error_m'] = float(
                np.linalg.norm(
                    [
                        last_position.x - last_reference.x,
                        last_position.y - last_reference.y,
                        last_position.z - last_reference.z,
                    ]
                )
            )
            final_truth = latest['truth'].pose.pose
            q = final_truth.orientation
            truth_body = np.array(
                [final_truth.position.x, final_truth.position.y, final_truth.position.z]
            ) - Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([-0.100, -0.001, 0.085])
            report['settled_true_tracking_error_m'] = float(
                np.linalg.norm(
                    truth_body
                    - np.array([last_reference.x, last_reference.y, last_reference.z])
                )
