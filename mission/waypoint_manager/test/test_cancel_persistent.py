"""Baseline test: cancelling a persistent goal that has no waypoints.

``handle_waypoint_cancel`` accepts the cancel request but never terminates the
goal handle when there is no reference filter goal to cancel, so the goal
stays in CANCELING and no result is ever delivered. This test FAILS on the
current code.
"""

import os
import unittest
import uuid

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, TimerAction
from rclpy.action import ActionClient
from vortex_msgs.action import WaypointManager

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)

NAMESPACE = "orca"


def launch_setup(context, *args, **kwargs):
    global NAMESPACE
    drone, namespace = resolve_drone_and_namespace(context)
    NAMESPACE = namespace

    rf_pkg_share = get_package_share_directory("reference_filter_dp")
    rf_config = os.path.join(rf_pkg_share, "config", "reference_filter_params.yaml")
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
        package="reference_filter_dp",
        executable="reference_filter_dp_node",
        name="reference_filter_node",
        namespace=namespace,
        parameters=[rf_config, drone_config],
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


class TestCancelPersistentGoal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'test_cancel_persistent_{uuid.uuid4().hex[:8]}')

    def tearDown(self):
        self.node.destroy_node()

    def test_cancel_persistent_goal_without_waypoints(self):
        client = ActionClient(
            self.node, WaypointManager, f'/{NAMESPACE}/waypoint_manager'
        )
        self.assertTrue(
            client.wait_for_server(timeout_sec=10.0),
            'WaypointManager action server not available',
        )

        goal = WaypointManager.Goal()
        goal.waypoints = []
        goal.persistent = True
        goal.convergence_threshold = 0.3

        send_fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_fut, timeout_sec=10.0)
        self.assertTrue(send_fut.done(), 'Timed out waiting for goal response')
        handle = send_fut.result()
        self.assertTrue(handle.accepted, 'Persistent goal without waypoints rejected')

        result_fut = handle.get_result_async()

        cancel_fut = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_fut, timeout_sec=10.0)
        self.assertTrue(cancel_fut.done(), 'Timed out waiting for cancel response')

        rclpy.spin_until_future_complete(self.node, result_fut, timeout_sec=5.0)
        self.assertTrue(
            result_fut.done(),
            'Goal never reached a terminal state after cancel (stuck in CANCELING)',
        )
        self.assertEqual(result_fut.result().status, GoalStatus.STATUS_CANCELED)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
