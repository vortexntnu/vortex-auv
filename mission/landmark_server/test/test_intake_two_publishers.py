"""Baseline test: two detectors publishing on ``landmarks`` at different rates.

Before rework step 2 ``landmark_server`` kept only the latest message between
two ticks (``measurements_`` was overwritten), so a slow publisher was starved
by a fast one and its track never survived. Intake now keeps every message.

The default N/M windows (confirm 3/5, delete 5/7) are counted in ticks (200 ms)
and cannot be satisfied by a 2 Hz detector (a hit only every 2-3 ticks), so the
test uses windows sized for that rate. Per-class configs come in step 5.
"""

import os
import time
import unittest
import uuid

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, TimerAction
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.msg import (
    Landmark,
    LandmarkArray,
    LandmarkSubtype,
    LandmarkTrackArray,
    LandmarkType,
)

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)

NAMESPACE = "nautilus"
ODOM_FRAME = "nautilus/odom"
CAMERA_FRAME = "test_camera"

FAST_HZ = 10.0
SLOW_HZ = 2.0
RUN_SEC = 60.0
CHECK_FROM_SEC = 30.0  # only judge the last half; tracks need time to confirm
MIN_PRESENT_FRACTION = 0.9


def launch_setup(context, *args, **kwargs):
    global NAMESPACE
    drone, namespace = resolve_drone_and_namespace(context)
    NAMESPACE = namespace

    landmark_config = os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "landmark_server_config.yaml",
    )
    sim_config = os.path.join(
        get_package_share_directory("landmark_server"), "config", "sim.yaml"
    )
    drone_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", ODOM_FRAME, "--child-frame-id", CAMERA_FRAME],
        output="screen",
    )

    landmark_server = launch_ros.actions.Node(
        package="landmark_server",
        executable="landmark_server_node",
        name="landmark_server_node",
        namespace=namespace,
        parameters=[
            landmark_config,
            sim_config,
            drone_config,
            {
                "use_sim_time": False,
                "debug.enable": True,
                "track_config.default.nm.confirm_n": 2,
                "track_config.default.nm.confirm_m": 5,
                "track_config.default.nm.delete_n": 6,
                "track_config.default.nm.delete_m": 8,
            },
        ],
        output="screen",
    )

    return [static_tf, landmark_server]


def generate_test_description():
    return launch.LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
            TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


def _landmark(landmark_type, landmark_subtype, x, y, z):
    lm = Landmark()
    lm.header.frame_id = CAMERA_FRAME
    lm.type.value = landmark_type
    lm.subtype.value = landmark_subtype
    lm.pose.pose.position.x = x
    lm.pose.pose.position.y = y
    lm.pose.pose.position.z = z
    lm.pose.pose.orientation.w = 1.0
    return lm


class TestIntakeTwoPublishers(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'test_intake_two_publishers_{uuid.uuid4().hex[:8]}')

    def tearDown(self):
        self.node.destroy_node()

    def test_both_tracks_survive_two_publishers(self):
        node = self.node
        topic = f'/{NAMESPACE}/landmarks'
        fast_pub = node.create_publisher(LandmarkArray, topic, qos_profile_sensor_data)
        slow_pub = node.create_publisher(LandmarkArray, topic, qos_profile_sensor_data)

        # (time since start, {type: has a confirmed track})
        samples = []
        start = time.monotonic()

        def on_tracks(msg):
            confirmed = {
                (t.landmark.type.value, t.landmark.subtype.value)
                for t in msg.landmark_tracks
                if t.confirmed
            }
            samples.append((time.monotonic() - start, confirmed))

        node.create_subscription(
            LandmarkTrackArray,
            f'/{NAMESPACE}/landmark_server/debug/landmark_tracks',
            on_tracks,
            qos_profile_sensor_data,
        )

        fast_key = (LandmarkType.GATE, LandmarkSubtype.GATE_SEARCH_RESCUE)
        slow_key = (LandmarkType.BIN, LandmarkSubtype.BIN_SEARCH_RESCUE)

        def publish_fast():
            msg = LandmarkArray()
            msg.header.frame_id = CAMERA_FRAME
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.landmarks = [_landmark(*fast_key, 5.0, 0.0, 2.0)]
            fast_pub.publish(msg)

        def publish_slow():
            msg = LandmarkArray()
            msg.header.frame_id = CAMERA_FRAME
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.landmarks = [_landmark(*slow_key, 8.0, 3.0, 2.0)]
            slow_pub.publish(msg)

        node.create_timer(1.0 / FAST_HZ, publish_fast)
        node.create_timer(1.0 / SLOW_HZ, publish_slow)

        while time.monotonic() - start < RUN_SEC:
            rclpy.spin_once(node, timeout_sec=0.1)

        judged = [s for t, s in samples if t >= CHECK_FROM_SEC]
        self.assertGreater(len(judged), 0, 'landmark_server published no debug tracks')

        for name, key in (('fast', fast_key), ('slow', slow_key)):
            present = sum(1 for s in judged if key in s) / len(judged)
            self.assertGreaterEqual(
                present,
                MIN_PRESENT_FRACTION,
                f'{name} publisher track confirmed in only {present:.0%} of '
                f'samples in the last {RUN_SEC - CHECK_FROM_SEC:.0f} s',
            )


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
