"""Observe an installed motion scenario on an isolated ROS domain."""

import argparse
import json
import math
import pathlib
import shutil
import signal
import subprocess
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory",
        choices=("barrel_roll", "square_barrel_roll"),
        default="barrel_roll",
    )
    args = parser.parse_args()
    square = args.trajectory == "square_barrel_roll"
    duration = 100.0 if square else 18.0
    label = "square" if square else "scenario"
    output = pathlib.Path(__file__).resolve().parents[1] / ".deps" / "audit"
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("gtsam_scenario_audit")
    records = {name: [] for name in ("imu", "dvl", "truth", "odom")}
    locks, statuses = [], []

    def collect(msg, name):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - 10
        records[name].append((stamp, time.monotonic(), msg))

    for name, topic, message in (
        ("imu", "imu/data_raw", Imu),
        ("dvl", "dvl/twist", TwistWithCovarianceStamped),
        ("truth", "gtsam/truth", Odometry),
        ("odom", "gtsam/odom", Odometry),
    ):
        node.create_subscription(
            message,
            "/nautilus/" + topic,
            lambda msg, name=name: collect(msg, name),
            qos_profile_sensor_data,
        )
    node.create_subscription(
        Bool,
        "/nautilus/dvl/bottom_lock",
        lambda msg: locks.append(msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    node.create_subscription(
        DiagnosticArray,
        "/nautilus/gtsam/status",
        statuses.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    with (output / f"{label}-launch.log").open("w") as log:
        process = subprocess.Popen(
            [
                shutil.which("ros2"),
                "launch",
                "gtsam_navigation",  # noqa: S603
                "simulation.launch.py",
                f"trajectory:={args.trajectory}",
                *([] if square else ["duration:=18.0"]),
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + duration + 90
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if records["odom"] and records["odom"][-1][0] >= duration - 0.01:
                    break
                assert process.poll() is None, (
                    "Launch exited before scenario completion"
                )
            assert records["odom"][-1][0] >= duration - 0.01
