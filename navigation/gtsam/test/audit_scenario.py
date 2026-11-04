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
            summary = {}
            for name, data in records.items():
                window = [row for row in data if 3 <= row[0] <= 7]
                stamps = np.array([row[0] for row in window])
                assert np.all(np.diff(stamps) > 0)
                summary[name] = {
                    "messages": len(data),
                    "median_header_rate_hz": float(1 / np.median(np.diff(stamps))),
                    "wall_rate_hz": float(
                        (len(window) - 1) / (window[-1][1] - window[0][1])
                    ),
                }
            assert abs(summary["imu"]["median_header_rate_hz"] - 1000) < 1e-6
            assert abs(summary["dvl"]["median_header_rate_hz"] - 8) < 1e-6
            assert 110 < summary["odom"]["wall_rate_hz"] < 140
            if square:
                for instant, position in ((77, [0, 0, 0]), (90, [2.1, 0, 0])):
                    row = min(records["truth"], key=lambda row: abs(row[0] - instant))
                    assert abs(row[0] - instant) < 0.002
                    p, q, v = (
                        row[2].pose.pose.position,
                        row[2].pose.pose.orientation,
                        row[2].twist.twist.linear,
                    )
                    np.testing.assert_allclose([p.x, p.y, p.z], position, atol=1e-6)
                    np.testing.assert_allclose([v.x, v.y, v.z], 0, atol=1e-6)
                    assert abs(abs(q.w) - 1) < 1e-5
                square_pings = [row[0] for row in records["dvl"] if 5 <= row[0] <= 78]
                assert max(np.diff(square_pings)) < 0.26
                assert not any(81 < row[0] < 82 for row in records["dvl"])
                assert any(85 < row[0] < 90 for row in records["dvl"])
                summary["returned_to_start_and_final_stop"] = True
            else:
                for instant in (12, 17):
                    row = min(records["truth"], key=lambda row: abs(row[0] - instant))
                    assert abs(row[0] - instant) < 0.002
                    q = row[2].pose.pose.orientation
                    assert abs(abs(q.w) - 1) < 1e-5
                    assert abs(row[2].twist.twist.angular.x - 2 * math.pi / 5) < 1e-12
                assert not any(7.2 < row[0] < 11.58 for row in records["dvl"])
                assert any(11.6 < row[0] < 12.4 for row in records["dvl"])
                summary["full_revolutions_checked_at_s"] = [12, 17]
            assert True in locks
            assert False in locks
            assert not any(
                msg.status[0].level == 2
                for msg in statuses
                if msg.header.stamp.sec > 13
            )
            summary["lock_loss_and_recovery"] = True
            (output / f"{label}-rates.json").write_text(
                json.dumps(summary, indent=2) + "\n"
            )
            print(json.dumps(summary, indent=2))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
