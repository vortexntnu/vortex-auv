"""Launch test for the landmark map: stable ids, memory, clear, course frame.

- a gate that stops being detected stays in ``object_map`` with the same id
  and is marked retained
- ``landmark_server/clear`` empties the map
- ``course_frame_state`` is UNSET and no TF ``nautilus/course`` exists before
  ``set_course_frame`` is called; COARSE with a TF afterwards; an illegal coin
  flip angle (0.4 rad) is rejected with success = false
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
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_srvs.srv import Empty
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from vortex_msgs.msg import (
    CourseFrameState,
    Landmark,
    LandmarkArray,
    LandmarkSubtype,
    LandmarkTrackArray,
    LandmarkType,
)
from vortex_msgs.srv import SetCourseFrame

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)

NAMESPACE = "nautilus"
ODOM_FRAME = "nautilus/odom"
CAMERA_FRAME = "test_camera"
COURSE_FRAME = "nautilus/course"


def launch_setup(context, *args, **kwargs):
    global NAMESPACE
    drone, namespace = resolve_drone_and_namespace(context)
    NAMESPACE = namespace

    landmark_config = os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "landmark_server_config.yaml",
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
            drone_config,
            {"use_sim_time": False},
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


def _gate_msg(node, x, y, z):
    lm = Landmark()
    lm.header.frame_id = CAMERA_FRAME
    lm.type.value = LandmarkType.GATE
    lm.subtype.value = LandmarkSubtype.GATE_WHOLE
    lm.pose.pose.position.x = x
    lm.pose.pose.position.y = y
    lm.pose.pose.position.z = z
    lm.pose.pose.orientation.w = 1.0
    msg = LandmarkArray()
    msg.header.frame_id = CAMERA_FRAME
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.landmarks = [lm]
    return msg


class TestLandmarkMap(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node(f'test_landmark_map_{uuid.uuid4().hex[:8]}')

    def tearDown(self):
        self.node.destroy_node()

    def _spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _call(self, client, request, timeout=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), client.srv_name)
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        self.assertTrue(fut.done(), f'no response from {client.srv_name}')
        return fut.result()

    def test_gate_is_remembered_with_stable_id_and_clear_empties_the_map(self):
        node = self.node
        object_map = []
        node.create_subscription(
            LandmarkTrackArray,
            f'/{NAMESPACE}/landmark_server/object_map',
            lambda m: object_map.append(m),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        pub = node.create_publisher(
            LandmarkArray, f'/{NAMESPACE}/landmarks', qos_profile_sensor_data
        )
        timer = node.create_timer(
            0.1, lambda: pub.publish(_gate_msg(node, 10.0, 0.0, 2.0))
        )

        # Seen: the gate appears in the map once its track is confirmed.
        end = time.monotonic() + 15.0
        while time.monotonic() < end and not (
            object_map and object_map[-1].landmark_tracks
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        self.assertTrue(object_map and object_map[-1].landmark_tracks, 'gate never appeared')
        gate = object_map[-1].landmark_tracks[0]
        gate_id = gate.landmark.id
        self.assertEqual(gate.landmark.type.value, LandmarkType.GATE)
        self.assertFalse(gate.retained)

        # The map is also published as markers for Foxglove.
        markers = []
        node.create_subscription(
            MarkerArray,
            f'/{NAMESPACE}/landmark_server/markers',
            lambda m: markers.append(m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._spin(1.0)
        latest_markers = markers[-1].markers
        labels = [m.text for m in latest_markers if m.ns == 'label']
        self.assertTrue(
            any(t.startswith('GATE_WHOLE #') for t in labels), f'labels: {labels}'
        )
        # The whole gate is drawn as its outline box, without a centre cube
        # (only the poster plates are cubes).
        self.assertTrue(any(m.ns == 'structure' for m in latest_markers))
        self.assertFalse(any(m.ns == 'landmark' for m in latest_markers))

        # Not seen any more: the tracker deletes its track, the map keeps it.
        timer.cancel()
        node.destroy_timer(timer)
        self._spin(6.0)
        latest = object_map[-1].landmark_tracks
        self.assertEqual(len(latest), 1, 'gate was forgotten')
        self.assertEqual(latest[0].landmark.id, gate_id)
        self.assertTrue(latest[0].retained)
        age = (
            node.get_clock().now()
            - rclpy.time.Time.from_msg(latest[0].last_measurement)
        ).nanoseconds * 1e-9
        self.assertGreater(age, 3.0)

        # Seen again: same id.
        timer = node.create_timer(
            0.1, lambda: pub.publish(_gate_msg(node, 10.1, 0.1, 2.0))
        )
        self._spin(4.0)
        latest = object_map[-1].landmark_tracks
        self.assertEqual(len(latest), 1)
        self.assertEqual(latest[0].landmark.id, gate_id)
        timer.cancel()

        # clear empties the map.
        client = node.create_client(Empty, f'/{NAMESPACE}/landmark_server/clear')
        self._call(client, Empty.Request())
        self._spin(1.0)
        self.assertEqual(len(object_map[-1].landmark_tracks), 0)

    def test_course_frame_state_and_tf(self):
        node = self.node
        states = []
        node.create_subscription(
            CourseFrameState,
            f'/{NAMESPACE}/landmark_server/course_frame_state',
            lambda m: states.append(m),
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        tf_buffer = Buffer()
        TransformListener(tf_buffer, node)

        # Before the service is called: UNSET and no TF.
        self._spin(1.5)
        self.assertTrue(states, 'no course_frame_state published')
        self.assertEqual(states[-1].state, CourseFrameState.UNSET)
        self.assertFalse(
            tf_buffer.can_transform(ODOM_FRAME, COURSE_FRAME, rclpy.time.Time())
        )

        client = node.create_client(
            SetCourseFrame, f'/{NAMESPACE}/landmark_server/set_course_frame'
        )

        # An illegal coin flip angle is rejected and changes nothing.
        bad = SetCourseFrame.Request()
        bad.start_pose.orientation.w = 1.0
        bad.heading_offset_rad = 0.4
        res = self._call(client, bad)
        self.assertFalse(res.success, res.message)
        self.assertEqual(res.state.state, CourseFrameState.UNSET)

        # A legal one gives COARSE and a TF.
        good = SetCourseFrame.Request()
        good.start_pose.position.x = 1.0
        good.start_pose.position.y = 2.0
        good.start_pose.orientation.w = 1.0
        good.heading_offset_rad = 1.5707963267948966
        res = self._call(client, good)
        self.assertTrue(res.success, res.message)
        self.assertEqual(res.state.state, CourseFrameState.COARSE)

        self._spin(1.5)
        self.assertEqual(states[-1].state, CourseFrameState.COARSE)
        self.assertTrue(
            tf_buffer.can_transform(
                ODOM_FRAME, COURSE_FRAME, rclpy.time.Time(), Duration(seconds=2.0)
            )
        )
        tf = tf_buffer.lookup_transform(ODOM_FRAME, COURSE_FRAME, rclpy.time.Time())
        self.assertAlmostEqual(tf.transform.translation.x, 1.0, places=6)
        self.assertAlmostEqual(tf.transform.translation.y, 2.0, places=6)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
