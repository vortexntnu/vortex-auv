"""Compare real ROS nodes with genuine, corrupted, and absent truth publications.

Run on an isolated ROS domain after sourcing the workspace. Does not launch a
bridge or the production sensor simulator, and terminates only its own children.
"""

import argparse
import copy
import json
import math
import pathlib
import signal
import subprocess
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from audit_simulation import MODEL, measurements
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster


def state_vector(msg):
    p, q, v, w = (
        msg.pose.pose.position,
        msg.pose.pose.orientation,
        msg.twist.twist.linear,
        msg.twist.twist.angular,
    )
    return np.array(
        [
            p.x,
            p.y,
            p.z,
            q.x,
            q.y,
            q.z,
            q.w,
            v.x,
            v.y,
            v.z,
            w.x,
            w.y,
            w.z,
            *msg.pose.covariance,
            *msg.twist.covariance,
        ]
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, truths, _ = measurements("barrel_roll", 12, 42)
    names = ("reference", "poisoned", "absent", "changed_dvl")
    rclpy.init()
    node = rclpy.create_node("gtsam_boundary_audit")
    processes, logs, results, statuses = [], [], {}, {}
    publishers = {}
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for child, offset, yaw in (
        ("imu_link", MODEL.IMU_OFFSET, 0.0),
        ("dvl_link", MODEL.DVL_OFFSET, MODEL.DVL_YAW),
    ):
        msg = TransformStamped()
        msg.header.frame_id = "nautilus/base_link"
        msg.child_frame_id = "nautilus/" + child
        (
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ) = map(float, offset)
        msg.transform.rotation.z = math.sin(yaw / 2)
        msg.transform.rotation.w = math.cos(yaw / 2)
        transforms.append(msg)
    broadcaster.sendTransform(transforms)
    executable = (
        pathlib.Path(get_package_prefix("gtsam_navigation"))
        / "lib/gtsam_navigation/gtsam_navigation_node"
    )

    def spin_until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError("Timed out waiting for audit nodes")

    try:
        for name in names:
            ns = f"/audit_{name}"
            results[name] = {}

            def capture(msg, name=name):
                stamp = msg.header.stamp
                results[name][stamp.sec * 10**9 + stamp.nanosec] = state_vector(msg)

            node.create_subscription(
                Odometry, ns + "/gtsam/odom", capture, qos_profile_sensor_data
            )
            node.create_subscription(
                DiagnosticArray,
                ns + "/gtsam/status",
                lambda msg, name=name: statuses.__setitem__(
                    name, msg.status[0].message
                ),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            publishers[name] = (
                node.create_publisher(Imu, ns + "/imu/data_raw", 100),
                node.create_publisher(
                    TwistWithCovarianceStamped, ns + "/dvl/twist", 100
                ),
                node.create_publisher(Odometry, ns + "/gtsam/truth", 100),
            )
            log = (args.output / f"ros-{name}.log").open("w")
            logs.append(log)
            processes.append(
                subprocess.Popen(  # noqa: S603
                    [
                        str(executable),
                        "--ros-args",
                        "-r",
                        f"__ns:={ns}",
                        "-p",
                        "use_sim_time:=true",
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            )
        spin_until(
            lambda: len(statuses) == len(names)
            and all(text == "waiting for stationary IMU" for text in statuses.values())
        )
        subscriptions = {}
        for name in names:
            topics = node.get_subscriber_names_and_types_by_node(
                "gtsam_navigation", f"/audit_{name}"
            )
            subscriptions[name] = [topic for topic, _ in topics]
            assert not any(
                "truth" in topic or "bottom_lock" in topic for topic, _ in topics
            )
        for index, row in enumerate(rows):
            start = time.monotonic()
            sim_time = float(row[0])
            stamp = rclpy.time.Time(nanoseconds=round((10 + sim_time) * 1e9)).to_msg()
            clock_pub.publish(Clock(clock=stamp))
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = "nautilus/imu_link"
            imu.orientation_covariance[0] = -1.0
            (
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ) = row[1:4]
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
                row[4:7]
            )
            dvl = TwistWithCovarianceStamped()
            dvl.header.stamp = stamp
            dvl.header.frame_id = "nautilus/dvl_link"
            (
                dvl.twist.twist.linear.x,
                dvl.twist.twist.linear.y,
                dvl.twist.twist.linear.z,
            ) = row[8:11]
