import os
import tempfile
import time as pytime
import math
import unittest
import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
import tf2_geometry_msgs
import launch_testing
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
import launch.actions

from test_utils.sim_setup import generate_sim_test_description


def generate_test_description():
    """Launch the simulation and add the waypoint_manager node."""

    config_file_path = os.path.join(
        get_package_share_directory('reference_filter_dp'),
        'config',
        'reference_filter_params.yaml',
    )

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller"),
        "config",
        "adapt_params.yaml",
    )

    orca_config = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        'orca.yaml',
    )



    extra_nodes = [
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
        package='reference_filter_dp',
        executable='reference_filter_dp_node',
        name='reference_filter_node',
        namespace='orca',
        parameters=[
            config_file_path,
            orca_config,
        ],
        output='screen',
        ),
        Node(
        package="dp_adapt_backs_controller",
        executable="dp_adapt_backs_controller_node",
        name="dp_adapt_backs_controller_node",
        namespace="orca",
        parameters=[
            adapt_params,
            orca_config,
        ],
        output="screen",
        )
    ]


    return generate_sim_test_description(
        scenario_value="default",
        extra_nodes=extra_nodes,
        delay=5.0,
        record_bag=True,
    )


class TestWaypointManagerRuntime(unittest.TestCase):
    """Runtime tests for waypoint_manager inside simulation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("waypoint_manager_test_node")

    def tearDown(self):
        self.node.destroy_node()


    def test_node_starts(self):
        """Ensure that the waypoint_manager_node appears in the ROS graph."""
        end_time = pytime.time() + 5.0
        while pytime.time() < end_time:
            nodes = self.node.get_node_names()
            if "waypoint_manager_node" in nodes:
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)
        else:
            self.fail("waypoint_manager_node not found in ROS graph")




@launch_testing.post_shutdown_test()
class TestWaypointManagerAfterShutdown(unittest.TestCase):
    """Verify processes exit cleanly after shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
