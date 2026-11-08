#!/usr/bin/env python3
"""Read-only live joystick, controller and estimator diagnostics."""

import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from vortex_msgs.msg import OperationMode
from vortex_msgs.srv import GetOperationMode

rclpy.init()
n = rclpy.create_node(f'nautilus_live_controls_{os.getpid()}')
latest = {}
counts = {}


def receive(key, msg):
    latest[key] = msg
    counts[key] = counts.get(key, 0) + 1


for key, topic, typ in [
    ('joy', 'joy', Joy),
    ('odom', 'gtsam/odom', Odometry),
    ('status', 'gtsam/status', DiagnosticArray),
    ('wrench', 'wrench_input', WrenchStamped),
    ('kill', 'killswitch', Bool),
    ('mode', 'operation_mode', OperationMode),
]:
    n.create_subscription(
        typ,
        '/nautilus/' + topic,
        lambda m, k=key: receive(k, m),
        qos_profile_sensor_data,
    )
client = n.create_client(GetOperationMode, '/nautilus/get_operation_mode')
future = None
print(
    'Tap B: toggle killswitch | A: manual | Y: DP reference | Ctrl+C: close this monitor',
    flush=True,
)
while rclpy.ok():
    end = time.monotonic() + 1
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    if future is None and client.service_is_ready():
        future = client.call_async(GetOperationMode.Request())
    if future is not None and future.done():
        result = future.result()
        latest['kill'] = Bool(data=result.killswitch_status)
        latest['mode'] = result.current_operation_mode
        future = None
    status = latest.get('status')
    joy = latest.get('joy')
    w = latest.get('wrench')
    mode = latest.get('mode')
    kill = latest.get('kill')
    print(
        'joy/s={} odom/s={} KILLED={} MODE={} buttons={} wrench={} status={}'.format(
            counts.get('joy', 0),
            counts.get('odom', 0),
            kill.data if kill else '?',
            {1: 'AUTO (blocked)', 2: 'MANUAL', 3: 'DP'}.get(mode.operation_mode, '?')
            if mode
            else '?',
            [i for i, x in enumerate(joy.buttons) if x] if joy else '?',
            [
                round(x, 2)
                for x in (
                    w.wrench.force.x,
                    w.wrench.force.y,
                    w.wrench.force.z,
                    w.wrench.torque.x,
                    w.wrench.torque.y,
                    w.wrench.torque.z,
                )
            ]
            if w
            else '?',
            status.status[0].message if status and status.status else '?',
        ),
        flush=True,
    )
    counts.clear()
