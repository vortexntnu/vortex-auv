"""Exercise actual ROS messages, namespaces, diagnostics and IMU-frame output."""

import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from launch_ros.actions import Node
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


@pytest.mark.launch_test
def generate_test_description():
    estimator = Node(
        package="gtsam_navigation",
        executable="gtsam_navigation_node",
        namespace="nautilus",
        parameters=[{"use_sim_time": True, "initialization_duration": 0.5}],
        output="screen",
    )
    simulator = Node(
        package="gtsam_navigation",
        executable="simulate_sensors.py",
        namespace="nautilus",
        parameters=[{"duration": 8.0, "trajectory": "stationary", "noise": False}],
        output="screen",
    )
    return launch.LaunchDescription(
        [estimator, simulator, launch_testing.actions.ReadyToTest()]
    )


class TestNavigationTopics(unittest.TestCase):
    def test_imu_frame_odometry_and_no_bias_topics(self):
        rclpy.init()
        node = rclpy.create_node("gtsam_navigation_test")
        odometry, statuses, bottom_locks = [], [], []
        node.create_subscription(
            Bool,
            "/nautilus/dvl/bottom_lock",
            bottom_locks.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        node.create_subscription(
            Odometry, "/nautilus/gtsam/odom", odometry.append, qos_profile_sensor_data
        )
        node.create_subscription(
            DiagnosticArray,
            "/nautilus/gtsam/status",
            statuses.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline and (
                len(odometry) < 20 or not statuses or not bottom_locks
            ):
                rclpy.spin_once(node, timeout_sec=0.1)
            assert len(odometry) >= 20
            assert statuses
            assert bottom_locks
            assert all(msg.data for msg in bottom_locks)
            stamps = []
            for msg in odometry:
                assert msg.header.frame_id == "nautilus/odom"
                assert msg.child_frame_id == "nautilus/imu_link"
                assert abs(msg.pose.pose.position.z) < 1e-5
                assert abs(msg.pose.pose.orientation.w - 1) < 1e-6
                assert msg.pose.covariance[0] > 0
                stamps.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
            assert all(a < b for a, b in zip(stamps, stamps[1:]))
            topics = dict(node.get_topic_names_and_types())
            assert not any("bias" in name for name in topics if "/gtsam/" in name)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
