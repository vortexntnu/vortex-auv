# import math
# import os
# import time
# import unittest

# import rclpy
# from rclpy.action import ActionClient
# from rclpy.executors import SingleThreadedExecutor

# from std_msgs.msg import String
# from vortex_msgs.action import WaypointManager as WaypointManagerAction
# from vortex_msgs.msg import Waypoint
# from geometry_msgs.msg import Pose

# import launch_testing
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# from test_utils.sim_setup import generate_sim_test_description

# ORCA_NS = "orca"
# WAYPOINT_MANAGER_ACTION_NAME = f"/{ORCA_NS}/waypoint_manager"
# OPERATION_MODE_TOPIC = f"/{ORCA_NS}/operation_mode"
# OPERATION_MODE_AUTONOMOUS = "autonomous mode"

# DEFAULT_CONVERGENCE_THRESHOLD = 0.3


# def generate_test_description():
#     config_file_path = os.path.join(
#         get_package_share_directory("reference_filter_dp"),
#         "config/reference_filter_params.yaml",
#     )

#     orca_config = os.path.join(
#         get_package_share_directory("auv_setup"),
#         "config/robots/orca.yaml",
#     )

#     extra_nodes = [
#         Node(
#             package="waypoint_manager",
#             executable="waypoint_manager_node",
#             namespace=ORCA_NS,
#             parameters=[{"use_sim_time": True}],
#             output="screen",
#         ),
#         Node(
#             package="reference_filter_dp",
#             executable="reference_filter_dp_node",
#             namespace=ORCA_NS,
#             parameters=[config_file_path, orca_config],
#             output="screen",
#         ),
#     ]

#     return generate_sim_test_description(
#         scenario_value="default",
#         rendering_enabled="false",
#         extra_nodes=extra_nodes,
#         delay=5.0,
#     )


# def make_pose(x, y, z, yaw=0.0):
#     p = Pose()
#     p.position.x = x
#     p.position.y = y
#     p.position.z = z
#     half = yaw * 0.5
#     p.orientation.z = math.sin(half)
#     p.orientation.w = math.cos(half)
#     return p


# def make_waypoint(x, y, z, yaw=0.0):
#     wp = Waypoint()
#     wp.pose = make_pose(x, y, z, yaw)
#     wp.mode = Waypoint.FULL_POSE
#     return wp


# class TestSingleWaypoint(unittest.TestCase):
#     @classmethod
#     def setUpClass(cls):
#         rclpy.init()

#     @classmethod
#     def tearDownClass(cls):
#         rclpy.shutdown()

#     def setUp(self):
#         self.node = rclpy.create_node("single_wp_client")
#         self.executor = SingleThreadedExecutor()
#         self.executor.add_node(self.node)

#         self.op_pub = self.node.create_publisher(String, OPERATION_MODE_TOPIC, 10)

#         self.ac = ActionClient(self.node, WaypointManagerAction, WAYPOINT_MANAGER_ACTION_NAME)

#         if not self.ac.wait_for_server(20.0):
#             self.fail("WaypointManager action server unavailable")

#         msg = String()
#         msg.data = OPERATION_MODE_AUTONOMOUS
#         self.op_pub.publish(msg)

#     def tearDown(self):
#         self.executor.remove_node(self.node)
#         self.node.destroy_node()

#     def spin_until(self, future, timeout=120):
#         start = time.time()
#         while not future.done():
#             if time.time() - start > timeout:
#                 self.fail("Timeout waiting for future")
#             self.executor.spin_once(0.1)
#         return future.result()

#     def test_single_waypoint_non_persistent(self):
#         wp = make_waypoint(2.0, 0.0, 1.0)

#         goal = WaypointManagerAction.Goal()
#         goal.waypoints = [wp]
#         goal.persistent = False
#         goal.convergence_threshold = DEFAULT_CONVERGENCE_THRESHOLD

#         send_future = self.ac.send_goal_async(goal)
#         gh = self.spin_until(send_future)

#         self.assertTrue(gh.accepted)

#         res_future = gh.get_result_async()
#         res = self.spin_until(res_future).result

#         self.assertTrue(res.success)
#         self.assertTrue(res.pose_valid)

#         dx = res.final_pose.position.x - wp.pose.position.x
#         dy = res.final_pose.position.y - wp.pose.position.y
#         dz = res.final_pose.position.z - wp.pose.position.z
#         dist = math.sqrt(dx*dx + dy*dy + dz*dz)

#         self.assertLess(dist, 0.6)


# @launch_testing.post_shutdown_test()
# class TestProcessesAfterShutdown(unittest.TestCase):
#     """Verify processes exit cleanly after shutdown."""

#     def test_exit_code(self, proc_info):
#         for info in proc_info:
#             name = info.process_name

#             # Allow the nogpu simulator to exit with -2 (SIGINT) or 0
#             if "stonefish_simulator_nogpu" in name:
#                 self.assertIn(
#                     info.returncode,
#                     (0, -2),
#                     f"{name} exited with unexpected code {info.returncode}",
#                 )
#             else:
#                 # Everyone else must exit cleanly
#                 self.assertEqual(
#                     info.returncode,
#                     0,
#                     f"{name} exited with non-zero code {info.returncode}",
#                 )