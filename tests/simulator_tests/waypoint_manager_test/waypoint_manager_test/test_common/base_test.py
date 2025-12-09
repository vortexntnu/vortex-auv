# waypoint_manager_test/test_common/base_test.py
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from vortex_msgs.action import WaypointManager
from vortex_msgs.srv import SendWaypoints

from .utils import pose_distance

ORCA_NS = "orca"
WM_ACTION = f"/{ORCA_NS}/waypoint_manager"
WP_ADD_SRV = f"/{ORCA_NS}/waypoint_addition"
OP_MODE_TOPIC = f"/{ORCA_NS}/operation_mode"
OP_AUTONOMOUS = "autonomous mode"


class WaypointManagerTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f"wm_test_client_{type(self).__name__}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Publisher to enable DP controller
        self.mode_pub = self.node.create_publisher(String, OP_MODE_TOPIC, 10)
        msg = String()
        msg.data = OP_AUTONOMOUS
        self.mode_pub.publish(msg)

        # Action + service clients
        self.action_client = ActionClient(self.node, WaypointManager, WM_ACTION)
        self.wp_add_client = self.node.create_client(SendWaypoints, WP_ADD_SRV)

        if not self.action_client.wait_for_server(timeout_sec=20.0):
            self.fail("WaypointManager action server unavailable")
        if not self.wp_add_client.wait_for_service(timeout_sec=20.0):
            self.fail("SendWaypoints service unavailable")

        self.executor.spin_once(timeout_sec=0.1)

    def tearDown(self):
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    # ---------------- helper utils ----------------

    def spin_until_done(self, future, timeout=120):
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.fail("Timed out waiting for future result")
            self.executor.spin_once(timeout_sec=0.1)
        return future.result()

    def send_goal(self, waypoints, persistent=False, conv=0.3):
        goal = WaypointManager.Goal()
        goal.waypoints = list(waypoints)
        goal.persistent = persistent
        goal.convergence_threshold = conv

        send_fut = self.action_client.send_goal_async(goal)
        handle = self.spin_until_done(send_fut, timeout=120)
        assert handle.accepted, "Goal was rejected"

        result_fut = handle.get_result_async()
        res = self.spin_until_done(result_fut, timeout=180)
        return handle, res.result

    def call_add(self, waypoints, overwrite_prior_waypoints=False, take_priority=False):
        req = SendWaypoints.Request()
        req.overwrite_prior_waypoints = overwrite_prior_waypoints
        req.take_priority = take_priority
        req.waypoints = list(waypoints)
        fut = self.wp_add_client.call_async(req)
        return self.spin_until_done(fut)

    def assert_pose_close(self, pose, target, tol=0.5, msg=""):
        d = pose_distance(pose, target)
        assert d < tol, msg or f"Pose too far from target ({d})"
