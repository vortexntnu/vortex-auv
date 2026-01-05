import os
import unittest
import uuid

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from vortex_msgs.action import WaypointManager
from vortex_msgs.msg import Waypoint
from vortex_msgs.srv import SendWaypoints


def generate_test_description():
    rf_pkg_share = get_package_share_directory('reference_filter_dp')
    rf_config = os.path.join(rf_pkg_share, 'config', 'reference_filter_params.yaml')

    auv_setup_share = get_package_share_directory('auv_setup')
    orca_config = os.path.join(auv_setup_share, 'config', 'robots', 'orca.yaml')

    wm_node = launch_ros.actions.Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        namespace='orca',
        output='screen',
    )

    rf_node = launch_ros.actions.Node(
        package='reference_filter_dp',
        executable='reference_filter_dp_node',
        name='reference_filter_node',
        namespace='orca',
        parameters=[rf_config, orca_config],
        output='screen',
    )

    return launch.LaunchDescription(
        [
            wm_node,
            rf_node,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestWaypointManagerService(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(
            f'test_waypoint_manager_client_{uuid.uuid4().hex[:8]}'
        )

    def tearDown(self):
        self.node.destroy_node()

    def _call_send_waypoints(
        self,
        waypoints,
        switching_threshold=0.3,
        overwrite=False,
        take_priority=False,
        timeout=5.0,
    ):
        client = self.node.create_client(SendWaypoints, '/orca/waypoint_addition')
        assert client.wait_for_service(timeout_sec=5.0), (
            'SendWaypoints service not available'
        )

        req = SendWaypoints.Request()
        req.waypoints = list(waypoints)
        req.switching_threshold = float(switching_threshold)
        req.overwrite_prior_waypoints = bool(overwrite)
        req.take_priority = bool(take_priority)

        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        assert fut.done(), 'Timed out waiting for SendWaypoints response'
        return fut.result()

    def test_service_returns_false_when_no_persistent_action(self):
        # When no persistent action is active, service should return false
        wp = Waypoint()
        wp.pose.position.x = 1.0
        wp.pose.position.y = 2.0
        wp.pose.position.z = 1.0

        resp = self._call_send_waypoints([wp], overwrite=False, take_priority=False)
        assert not resp.success, (
            'Service should return false when no persistent action is active'
        )

    def test_priority_blocks_non_priority_calls(self):
        # Send a persistent action goal so the waypoint manager is in persistent mode
        action_client = ActionClient(
            self.node, WaypointManager, '/orca/waypoint_manager'
        )
        assert action_client.wait_for_server(timeout_sec=10.0), (
            'WaypointManager action server not available'
        )

        goal_msg = WaypointManager.Goal()
        wp1 = Waypoint()
        wp1.pose.position.x = 0.0
        wp1.pose.position.y = 0.0
        wp1.pose.position.z = 1.0
        wp2 = Waypoint()
        wp2.pose.position.x = 1.0
        wp2.pose.position.y = 1.0
        wp2.pose.position.z = 1.0

        goal_msg.waypoints = [wp1, wp2]
        goal_msg.persistent = True
        goal_msg.convergence_threshold = 0.3

        send_fut = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_fut, timeout_sec=20.0)
        assert send_fut.done(), 'Timed out waiting for goal response'
        goal_handle = send_fut.result()
        assert goal_handle.accepted, 'Goal was rejected'

        # Call service to take priority
        resp1 = self._call_send_waypoints([], overwrite=False, take_priority=True)
        assert resp1.success, 'Priority service call should succeed'

        # Now a non-priority call while priority is active should be rejected
        wp_extra = Waypoint()
        wp_extra.pose.position.x = 2.0
        wp_extra.pose.position.y = 2.0
        wp_extra.pose.position.z = 1.0

        resp2 = self._call_send_waypoints(
            [wp_extra], overwrite=False, take_priority=False
        )
        assert not resp2.success, (
            'Non-priority call should be rejected while priority is active'
        )


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
