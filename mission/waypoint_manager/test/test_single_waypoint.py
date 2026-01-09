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
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from vortex_msgs.action import WaypointManager
from vortex_msgs.msg import Waypoint


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


class TestWaypointManagerAcceptsGoal(unittest.TestCase):
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

    def _publish_fake_odom(self, x, y, z, duration_sec=5.0, rate_hz=10.0):
        pub = self.node.create_publisher(
            Odometry,
            '/orca/odom',
            qos_profile_sensor_data,
        )

        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z

        end_time = time.time() + duration_sec
        period = 1.0 / rate_hz

        while time.time() < end_time:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(period)

        self.node.destroy_publisher(pub)

    def test_accepts_and_executes_goal(self):
        client = ActionClient(self.node, WaypointManager, '/orca/waypoint_manager')

        assert client.wait_for_server(timeout_sec=10.0), (
            'WaypointManager action server not available'
        )

        goal_msg = WaypointManager.Goal()
        wp = Waypoint()
        wp.pose.position.x = 0.0
        wp.pose.position.y = 0.0
        wp.pose.position.z = 1.0

        goal_msg.waypoints = [wp]
        goal_msg.persistent = False
        goal_msg.convergence_threshold = 0.3

        send_fut = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self.node,
            send_fut,
            timeout_sec=20.0,
        )

        assert send_fut.done(), 'Timed out waiting for goal response'

        goal_handle = send_fut.result()
        assert goal_handle.accepted, 'Goal was rejected'

        # Publish fake odometry at the goal position
        self._publish_fake_odom(
            x=wp.pose.position.x,
            y=wp.pose.position.y,
            z=wp.pose.position.z,
            duration_sec=5.0,
        )

        result_fut = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node,
            result_fut,
            timeout_sec=20.0,
        )

        assert result_fut.done(), 'Timed out waiting for result'

        result = result_fut.result().result
        assert result.success, 'Waypoint execution failed'


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
