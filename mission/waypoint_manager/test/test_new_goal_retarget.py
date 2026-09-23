"""A new WaypointManager goal retargets the reference filter without a stop.

Five goals are sent at 5 Hz while the vehicle (a fake pose that stays at the
origin) is being guided. The four goals that are replaced must end with
outcome PREEMPTED, and the reference velocity must not fall back to zero
between the goals, which is what a cancel followed by a cold start would do.
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
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from launch.actions import OpaqueFunction, TimerAction
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.action import WaypointManager
from vortex_msgs.msg import ReferenceFilterQuat, Waypoint, WaypointMode

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)

NAMESPACE = "orca"
GOAL_RATE_HZ = 5.0
TARGET_X = [10.0, 12.0, 14.0, 16.0, 18.0]


def launch_setup(context, *args, **kwargs):
    global NAMESPACE
    drone, namespace = resolve_drone_and_namespace(context)
    NAMESPACE = namespace

    rf_config = os.path.join(
        get_package_share_directory("reference_filter_dp_quat"),
        "config",
        "reference_filter_params.yaml",
    )
    drone_config = os.path.join(
        get_package_share_directory("auv_setup"), "config", "robots", f"{drone}.yaml"
    )

    wm_node = launch_ros.actions.Node(
        package="waypoint_manager",
        executable="waypoint_manager_node",
        name="waypoint_manager_node",
        namespace=namespace,
        parameters=[drone_config],
        output="screen",
    )
    rf_node = launch_ros.actions.Node(
        package="reference_filter_dp_quat",
        executable="reference_filter_dp_quat_node",
        name="reference_filter_node",
        namespace=namespace,
        parameters=[
            rf_config,
            drone_config,
            {
                "altitude_control_enabled": False,
                # Fast enough that the reference velocity is clearly non-zero after a
                # few goals, slow enough that it is still increasing (no natural peak).
                "omega": [0.5] * 6,
                "zeta": [1.0] * 6,
            },
        ],
        output="screen",
    )
    return [wm_node, rf_node]


def generate_test_description():
    return launch.LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
            TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


def _goal(x):
    goal = WaypointManager.Goal()
    wp = Waypoint()
    wp.waypoint_mode.mode = WaypointMode.ONLY_POSITION
    wp.pose.position.x = x
    wp.pose.orientation.w = 1.0
    goal.waypoints = [wp]
    goal.persistent = False
    goal.convergence_threshold = 0.1
    return goal


class TestNewGoalRetarget(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'test_new_goal_retarget_{uuid.uuid4().hex[:8]}')

    def tearDown(self):
        self.node.destroy_node()

    def test_new_goals_retarget_without_stopping(self):
        node = self.node

        pose_pub = node.create_publisher(
            PoseWithCovarianceStamped, f'/{NAMESPACE}/pose', qos_profile_sensor_data
        )
        twist_pub = node.create_publisher(
            TwistWithCovarianceStamped, f'/{NAMESPACE}/twist', qos_profile_sensor_data
        )

        def publish_state():
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'odom'
            pose.header.stamp = node.get_clock().now().to_msg()
            pose.pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            twist = TwistWithCovarianceStamped()
            twist.header.frame_id = 'odom'
            twist.header.stamp = pose.header.stamp
            twist_pub.publish(twist)

        node.create_timer(0.05, publish_state)

        references = []  # (time, x, x_dot)
        node.create_subscription(
            ReferenceFilterQuat,
            f'/{NAMESPACE}/guidance/dp_quat',
            lambda m: references.append((time.monotonic(), m.x, m.x_dot)),
            qos_profile_sensor_data,
        )

        client = ActionClient(node, WaypointManager, f'/{NAMESPACE}/waypoint_manager')
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))

        # Let the fake pose reach the reference filter first.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

        handles = []
        second_goal_time = None
        for i, x in enumerate(TARGET_X):
            fut = client.send_goal_async(_goal(x))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            self.assertTrue(fut.done(), f'no response for goal {i}')
            self.assertTrue(fut.result().accepted, f'goal {i} rejected')
            handles.append(fut.result())
            if i == 1:
                second_goal_time = time.monotonic()
            end = time.monotonic() + 1.0 / GOAL_RATE_HZ
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)

        # Keep guiding a little after the last goal.
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

        # The four replaced goals end with PREEMPTED.
        for i, handle in enumerate(handles[:-1]):
            res_fut = handle.get_result_async()
            rclpy.spin_until_future_complete(node, res_fut, timeout_sec=5.0)
            self.assertTrue(res_fut.done(), f'no result for goal {i}')
            wrapped = res_fut.result()
            self.assertEqual(wrapped.status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(
                wrapped.result.outcome,
                WaypointManager.Result.PREEMPTED,
                f'goal {i}: {wrapped.result.message}',
            )
            self.assertFalse(wrapped.result.success)

        # No stop in the reference: from the second goal on the reference
        # velocity keeps growing towards the far targets (a cancel followed by
        # a cold start would reset it to the measured twist, i.e. zero).
        window = [xd for t, _, xd in references if t >= second_goal_time]
        self.assertGreater(len(window), 20, 'reference filter published too little')
        self.assertGreater(max(window), 0.1, 'reference never picked up speed')
        drops = [b - a for a, b in zip(window, window[1:])]
        self.assertGreaterEqual(min(drops), -1e-3, 'reference velocity dropped')

        # The last goal is still running; cancel it and check the outcome.
        cancel_fut = handles[-1].cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=5.0)
        res_fut = handles[-1].get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=5.0)
        self.assertTrue(res_fut.done(), 'last goal never finished after cancel')
        self.assertEqual(res_fut.result().status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(res_fut.result().result.outcome, WaypointManager.Result.CANCELED)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
