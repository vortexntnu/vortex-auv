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
