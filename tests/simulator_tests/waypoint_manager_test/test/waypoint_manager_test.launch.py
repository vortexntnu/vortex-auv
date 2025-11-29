import math
import os
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from vortex_msgs.action import WaypointManager as WaypointManagerAction
from vortex_msgs.msg import Waypoint
from vortex_msgs.srv import WaypointAddition

import launch_testing
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from test_utils.sim_setup import generate_sim_test_description


# -------------------------------------------------------------------
# Constants / configuration
# -------------------------------------------------------------------

ORCA_NS = "orca"
WAYPOINT_MANAGER_ACTION_NAME = f"/{ORCA_NS}/waypoint_manager"
WAYPOINT_ADDITION_SERVICE_NAME = f"/{ORCA_NS}/waypoint_addition"
OPERATION_MODE_TOPIC = f"/{ORCA_NS}/operation_mode"
OPERATION_MODE_AUTONOMOUS = "autonomous mode"

DEFAULT_CONVERGENCE_THRESHOLD = 0.3


# -------------------------------------------------------------------
# Launch description
# -------------------------------------------------------------------

def generate_test_description():
    """Launch the simulation and add the waypoint_manager + dependencies."""

    config_file_path = os.path.join(
        get_package_share_directory("reference_filter_dp"),
        "config",
        "reference_filter_params.yaml",
    )

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller"),
        "config",
        "adapt_params.yaml",
    )

    orca_config = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        "orca.yaml",
    )

    extra_nodes = [
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager_node",
            namespace=ORCA_NS,
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="reference_filter_dp",
            executable="reference_filter_dp_node",
            name="reference_filter_node",
            namespace=ORCA_NS,
            parameters=[
                config_file_path,
                orca_config,
            ],
            output="screen",
        ),
        Node(
            package="dp_adapt_backs_controller",
            executable="dp_adapt_backs_controller_node",
            name="dp_adapt_backs_controller_node",
            namespace=ORCA_NS,
            parameters=[
                adapt_params,
                orca_config,
            ],
            output="screen",
        ),
        Node(
            package="thrust_allocator_auv",
            executable="thrust_allocator_auv_node",
            name="thrust_allocator_auv_node",
            namespace=ORCA_NS,
            parameters=[orca_config],
            output="screen",
        ),
    ]

    # We use full GPU sim here; if you want, you can set rendering_enabled="false"
    return generate_sim_test_description(
        scenario_value="default",
        rendering_enabled="true",
        extra_nodes=extra_nodes,
        delay=5.0,
        record_bag=True,
    )


# -------------------------------------------------------------------
# Test helpers
# -------------------------------------------------------------------

def make_pose(x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
    """Create a simple pose with yaw only (RPY = (0, 0, yaw))."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    half_yaw = yaw * 0.5
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)

    return pose


def make_waypoint(x, y, z, yaw=0.0, mode=None) -> Waypoint:
    """Convenience builder for vortex_msgs/Waypoint."""
    wp = Waypoint()
    wp.pose = make_pose(x, y, z, yaw)
    wp.mode = mode if mode is not None else Waypoint.FULL_POSE
    return wp


# -------------------------------------------------------------------
# Base test class with clients + utilities
# -------------------------------------------------------------------

class WaypointManagerTestBase(unittest.TestCase):
    """Base class that sets up an rclpy node, executor and clients."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Give each test class its own node name to avoid collisions
        node_name = f"waypoint_manager_test_client_{self.__class__.__name__}"
        self.node = rclpy.create_node(node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Operation mode publisher to enable DP controller
        self.op_mode_pub = self.node.create_publisher(
            String,
            OPERATION_MODE_TOPIC,
            10,
        )

        # Action client for /orca/waypoint_manager
        self.action_client = ActionClient(
            self.node,
            WaypointManagerAction,
            WAYPOINT_MANAGER_ACTION_NAME,
        )

        # Service client for /orca/waypoint_addition
        self.wp_add_client = self.node.create_client(
            WaypointAddition,
            WAYPOINT_ADDITION_SERVICE_NAME,
        )

        # Wait for server & service
        if not self.action_client.wait_for_server(timeout_sec=20.0):
            self.fail("WaypointManager action server not available")

        if not self.wp_add_client.wait_for_service(timeout_sec=20.0):
            self.fail("waypoint_addition service not available")

        # Put controller into autonomous mode
        msg = String()
        msg.data = OPERATION_MODE_AUTONOMOUS
        for _ in range(10):
            self.op_mode_pub.publish(msg)
            self.executor.spin_once(timeout_sec=0.1)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    # ------------- small helpers -------------

    def spin_until_future_complete(self, future, timeout_sec: float = 120.0):
        start = time.time()
        while not future.done():
            if (time.time() - start) > timeout_sec:
                self.fail("Timed out waiting for future")
            self.executor.spin_once(timeout_sec=0.1)
        return future.result()

    def send_waypoint_goal(
        self,
        waypoints,
        persistent=False,
        convergence_threshold=DEFAULT_CONVERGENCE_THRESHOLD,
        feedback_list=None,
    ):
        """
        Send a WaypointManager action goal and wait for result.
        feedback_list: optional list that will be appended with feedback messages.
        """
        goal_msg = WaypointManagerAction.Goal()
        goal_msg.waypoints = list(waypoints)
        goal_msg.persistent = persistent
        goal_msg.convergence_threshold = convergence_threshold

        def fb_cb(feedback):
            if feedback_list is not None:
                feedback_list.append(feedback.feedback)

        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=fb_cb if feedback_list is not None else None,
        )
        goal_handle = self.spin_until_future_complete(send_future, timeout_sec=120.0)

        self.assertTrue(goal_handle.accepted, "WaypointManager goal was rejected")

        result_future = goal_handle.get_result_async()
        result = self.spin_until_future_complete(result_future, timeout_sec=180.0)
        return goal_handle, result.result  # result.result is the actual action result

    def call_wp_addition(
        self,
        waypoints,
        overwrite: bool = False,
        non_interruptible: bool = False,
    ) -> WaypointAddition.Response:
        req = WaypointAddition.Request()
        req.overwrite = overwrite
        req.non_interruptible = non_interruptible
        req.waypoints = list(waypoints)

        future = self.wp_add_client.call_async(req)
        resp = self.spin_until_future_complete(future, timeout_sec=30.0)
        return resp

    def assert_pose_close(self, pose: Pose, target: Pose, tol: float = 0.5, msg=""):
        dx = pose.position.x - target.position.x
        dy = pose.position.y - target.position.y
        dz = pose.position.z - target.position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        self.assertLess(
            dist,
            tol,
            msg or (
                f"Pose {pose.position} not within {tol} m of "
                f"target {target.position} (dist={dist})"
            ),
        )


# -------------------------------------------------------------------
# Runtime tests while system is up
# -------------------------------------------------------------------

class TestWaypointManagerMissions(WaypointManagerTestBase):
    """High-level mission logic tests using the real simulator + DP + reference filter."""

    def test_single_waypoint_non_persistent(self):
        """
        Non-persistent mission with a single waypoint should complete,
        success=true, pose_valid=true, final_pose near the waypoint.
        """
        target_wp = make_waypoint(2.0, 0.0, 1.0)

        _, result = self.send_waypoint_goal(
            waypoints=[target_wp],
            persistent=False,
            convergence_threshold=0.3,
        )

        self.assertTrue(result.success, "Single waypoint mission did not succeed")
        self.assertTrue(result.pose_valid, "Result did not contain a valid pose")
        self.assertIsNotNone(result.final_pose, "No final_pose in result")

        self.assert_pose_close(
            result.final_pose,
            target_wp.pose,
            tol=0.6,
            msg="Final pose not close to single waypoint",
        )

    def test_persistent_append_waypoints(self):
        """
        Start a persistent mission, then append additional waypoints via service.
        The final result (on cancel) should reflect the last appended waypoint.
        """
        initial_wp = make_waypoint(2.0, 0.0, 1.0)
        append_wp = make_waypoint(4.0, 0.0, 1.0)

        goal_msg = WaypointManagerAction.Goal()
        goal_msg.waypoints = [initial_wp]
        goal_msg.persistent = True
        goal_msg.convergence_threshold = 0.3

        feedbacks = []

        def fb_cb(fb):
            feedbacks.append(fb.feedback)

        send_future = self.action_client.send_goal_async(goal_msg, feedback_callback=fb_cb)
        goal_handle = self.spin_until_future_complete(send_future, timeout_sec=60.0)
        self.assertTrue(goal_handle.accepted, "Persistent mission goal was rejected")

        # Let the vehicle start moving toward the first waypoint
        end_time = time.time() + 5.0
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

        # Append waypoint(s)
        resp = self.call_wp_addition(
            [append_wp],
            overwrite=False,
            non_interruptible=False,
        )
        self.assertTrue(resp.success, "WaypointAddition append failed during persistent mission")

        # Let the filter/DP walk towards the appended waypoint
        end_time = time.time() + 40.0
        while time.time() < end_time and not goal_handle.is_done:
            self.executor.spin_once(timeout_sec=0.2)

        # Cancel to get final pose
        cancel_future = goal_handle.cancel_goal_async()
        self.spin_until_future_complete(cancel_future, timeout_sec=10.0)
        result_future = goal_handle.get_result_async()
        result_wrapper = self.spin_until_future_complete(result_future, timeout_sec=10.0)
        result = result_wrapper.result

        self.assertFalse(result.success, "Persistent mission unexpectedly succeeded on cancel")
        if result.pose_valid:
            self.assert_pose_close(
                result.final_pose,
                append_wp.pose,
                tol=1.0,
                msg="Final pose after persistent mission not close to appended waypoint",
            )

    def test_persistent_overwrite_waypoints(self):
        """
        Persistent mission where waypoints are overwritten via service.
        The mission should head towards the overwrite waypoint instead of the original.
        """
        original_wp = make_waypoint(2.0, 0.0, 1.0)
        overwrite_wp = make_waypoint(0.0, 2.0, 1.0)

        goal_msg = WaypointManagerAction.Goal()
        goal_msg.waypoints = [original_wp]
        goal_msg.persistent = True
        goal_msg.convergence_threshold = 0.3

        send_future = self.action_client.send_goal_async(goal_msg)
        goal_handle = self.spin_until_future_complete(send_future, timeout_sec=60.0)
        self.assertTrue(goal_handle.accepted, "Persistent mission goal was rejected")

        # Let it start heading to the original waypoint
        end_time = time.time() + 5.0
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

        # OVERWRITE with a different waypoint
        resp = self.call_wp_addition(
            [overwrite_wp],
            overwrite=True,
            non_interruptible=False,
        )
        self.assertTrue(resp.success, "WaypointAddition overwrite failed")

        # Let it move towards the new target
        end_time = time.time() + 40.0
        while time.time() < end_time and not goal_handle.is_done:
            self.executor.spin_once(timeout_sec=0.2)

        # Cancel to get final pose
        cancel_future = goal_handle.cancel_goal_async()
        self.spin_until_future_complete(cancel_future, timeout_sec=10.0)

        result_future = goal_handle.get_result_async()
        result = self.spin_until_future_complete(result_future, timeout_sec=10.0).result

        self.assertFalse(result.success, "Overwrite test goal should end as canceled")
        if result.pose_valid:
            self.assert_pose_close(
                result.final_pose,
                overwrite_wp.pose,
                tol=1.0,
                msg="Final pose after overwrite not close to overwrite waypoint",
            )

    def test_action_preemption(self):
        """
        Sending a second waypoint_manager goal should abort the first one according
        to WaypointManagerNode::handle_waypoint_goal.
        """
        wp1 = make_waypoint(2.0, 0.0, 1.0)
        wp2 = make_waypoint(4.0, 1.0, 1.0)

        # First goal
        goal1 = WaypointManagerAction.Goal()
        goal1.waypoints = [wp1]
        goal1.persistent = False
        goal1.convergence_threshold = 0.3

        send_future1 = self.action_client.send_goal_async(goal1)
        gh1 = self.spin_until_future_complete(send_future1, timeout_sec=60.0)
        self.assertTrue(gh1.accepted, "First goal was rejected")

        # Give it a short time to become active
        end_time = time.time() + 2.0
        while time.time() < end_time:
            self.executor.spin_once(timeout_sec=0.1)

        # Second goal should preempt / abort the first
        goal2 = WaypointManagerAction.Goal()
        goal2.waypoints = [wp2]
        goal2.persistent = False
        goal2.convergence_threshold = 0.3

        send_future2 = self.action_client.send_goal_async(goal2)
        gh2 = self.spin_until_future_complete(send_future2, timeout_sec=60.0)
        self.assertTrue(gh2.accepted, "Second goal was rejected")

        # First goal result should come back aborted with success=false
        res1_future = gh1.get_result_async()
        res1 = self.spin_until_future_complete(res1_future, timeout_sec=60.0).result
        self.assertFalse(res1.success, "First goal not aborted on preemption")

        # Second goal should succeed (given enough time)
        res2_future = gh2.get_result_async()
        res2 = self.spin_until_future_complete(res2_future, timeout_sec=180.0).result
        self.assertTrue(res2.success, "Second goal did not succeed after preemption")


# -------------------------------------------------------------------
# Post-shutdown exit-code check
# -------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestProcessesAfterShutdown(unittest.TestCase):
    """Verify processes exit cleanly after shutdown."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            name = info.process_name
            rc = info.returncode

            # GPU sim exits with 0 on SIGINT in your current setup
            self.assertEqual(
                rc,
                0,
                f"{name} exited with non-zero code {rc}",
            )
