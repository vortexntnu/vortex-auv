"""Launch test for the iSAM2 backend: drifting odometry, one loop closure.

A simulated vehicle drives 12 m out along x, turns and comes back to x = 2.
Its odometry drifts 1 degree of yaw on every metre. A forward camera sees a
table near the start (going out and coming back) and a torpedo board at the
far end, and reports them exactly in the odom frame, as the real pipeline
does after TF.

When the table is seen again the map takes over its old id, the graph closes
the loop, and the board, last seen ~10 m earlier, moves to where it really is
relative to the vehicle. Without the graph it would stay where the drifted
odometry put it.
"""

import math
import os
import time
import unittest
import uuid

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch.actions import OpaqueFunction, TimerAction
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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

DRIFT_DEG_PER_M = 1.0
STEP_M = 0.1
RATE_HZ = 10.0
TABLE = (LandmarkType.TABLE, LandmarkSubtype.TABLE_WHOLE, np.array([0.0, 1.5, 2.5]))
BOARD = (
    LandmarkType.TORPEDO_BOARD,
    LandmarkSubtype.TORPEDO_BOARD_WHOLE,
    np.array([13.0, -2.0, 1.5]),
)


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
                "graph.enable": True,
                # The drift here is a bias: allow it in the random-walk model.
                "graph.odom_noise.yaw_std_deg_per_m": 3.0,
            },
        ],
        output="screen",
    )
    return [landmark_server]


def generate_test_description():
    return launch.LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
            TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Pose2:
    """Position (3D) and yaw."""

    def __init__(self, p, yaw):
        self.p = np.array(p, dtype=float)
        self.yaw = yaw

    def step(self, forward, turn):
        self.p = self.p + rot_z(self.yaw) @ np.array([forward, 0.0, 0.0])
        self.yaw += turn

    def to_body(self, point):
        return rot_z(self.yaw).T @ (point - self.p)

    def from_body(self, point):
        return self.p + rot_z(self.yaw) @ point


def route():
    """(forward [m], turn [rad]) per tick: out, turn around, back to x = 2."""
    steps = []
    steps += [(STEP_M, 0.0)] * int(round(14.0 / STEP_M))
    steps += [(0.0, math.radians(10.0))] * 18
    steps += [(STEP_M, 0.0)] * int(round(10.0 / STEP_M))
    steps += [(0.0, 0.0)] * int(3.0 * RATE_HZ)  # hold, looking at the table
    return steps


class TestGraphDrift(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f"test_graph_drift_{uuid.uuid4().hex[:8]}")

    def tearDown(self):
        self.node.destroy_node()

    def test_loop_closure_corrects_the_far_landmark(self):
        node = self.node
        object_map = []
        node.create_subscription(
            LandmarkTrackArray,
            f"/{NAMESPACE}/landmark_server/object_map",
            lambda m: object_map.append(m),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        odom_pub = node.create_publisher(
            Odometry, f"/{NAMESPACE}/odom", qos_profile_sensor_data
        )
        lm_pub = node.create_publisher(
            LandmarkArray, f"/{NAMESPACE}/landmarks", qos_profile_sensor_data
        )

        truth = Pose2([-2.0, 0.0, 2.0], 0.0)
        odom = Pose2([-2.0, 0.0, 2.0], 0.0)
        last_raw = {}
        table_ids = set()

        for forward, turn in route():
            truth.step(forward, turn)
            odom.step(forward, turn + math.radians(DRIFT_DEG_PER_M * forward))
            stamp = node.get_clock().now().to_msg()

            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = ODOM_FRAME
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (
                float(v) for v in odom.p
            )
            o.pose.pose.orientation.z = math.sin(odom.yaw / 2.0)
            o.pose.pose.orientation.w = math.cos(odom.yaw / 2.0)
            odom_pub.publish(o)

            arr = LandmarkArray()
            arr.header.stamp = stamp
            arr.header.frame_id = ODOM_FRAME
            for type_, subtype, world in (TABLE, BOARD):
                body = truth.to_body(world)
                rng = float(np.linalg.norm(body))
                if body[0] <= 0.0 or rng > 6.0:
                    continue
                if abs(math.atan2(body[1], body[0])) > math.radians(60.0):
                    continue
                p = odom.from_body(body)
                lm = Landmark()
                lm.header = arr.header
                lm.type.value = type_
                lm.subtype.value = subtype
                lm.pose.pose.position.x, lm.pose.pose.position.y = float(p[0]), float(p[1])
                lm.pose.pose.position.z = float(p[2])
                lm.pose.pose.orientation.w = 1.0
                # Position only.
                for i in (21, 28, 35):
                    lm.pose.covariance[i] = 1000.0
                arr.landmarks.append(lm)
                last_raw[type_] = p
            lm_pub.publish(arr)

            end = time.monotonic() + 1.0 / RATE_HZ
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            if object_map:
                for t in object_map[-1].landmark_tracks:
                    if t.landmark.type.value == LandmarkType.TABLE:
                        table_ids.add(t.landmark.id)

        self.assertTrue(object_map, "no object_map")
        by_type = {}
        for t in object_map[-1].landmark_tracks:
            if not t.derived:
                p = t.landmark.pose.pose.position
                by_type[t.landmark.type.value] = (t, np.array([p.x, p.y, p.z]))
        self.assertIn(LandmarkType.TABLE, by_type)
        self.assertIn(LandmarkType.TORPEDO_BOARD, by_type)
        # Seen again, the table kept its id: the loop was closed on it.
        self.assertEqual(len(table_ids), 1, f"table ids: {table_ids}")

        board, board_pos = by_type[LandmarkType.TORPEDO_BOARD]
        self.assertTrue(board.retained, "board should be out of view by now")
        expected = odom.from_body(truth.to_body(BOARD[2]))
        raw_err = float(np.linalg.norm(last_raw[LandmarkType.TORPEDO_BOARD] - expected))
        map_err = float(np.linalg.norm(board_pos - expected))
        print(f"board: raw error {raw_err:.2f} m, graph error {map_err:.2f} m")
        self.assertGreater(raw_err, 0.8, "the drift is too small for the test")
        self.assertLess(map_err, 0.5 * raw_err)

        _, table_pos = by_type[LandmarkType.TABLE]
        table_expected = odom.from_body(truth.to_body(TABLE[2]))
        self.assertLess(float(np.linalg.norm(table_pos - table_expected)), 0.3)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
