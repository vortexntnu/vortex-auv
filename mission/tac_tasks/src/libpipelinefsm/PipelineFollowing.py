#!/usr/bin/python3

import rospy
import tf

from geometry_msgs.msg import Pose, Wrench
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from landmarks.srv import request_position

from dp_client_py.DPClient import DPClient

from task_manager_defines import defines  # type: ignore
from task_manager_client.TaskManagerClient import TaskManagerClient  # type: ignore


class PipelineFollowing:

    def __init__(self):
        # =====[Attributes]===== #
        self.rate = rospy.Rate(10)
        self.sending_rate = rospy.Rate(1)

        self.odom_pose = Pose()

        # =====[Params]===== #
        # Height which is used to follow the pipeline
        # self.following_height = rospy.get_param(
        #     "/tac/pipeline/following_height")

        # =====[Services, clients, handles]===== #
        # DP action client
        self.dp_client = DPClient()
        self.dp_client.goal.DOF = [1, 1, 1, 0, 0, 1]

        # Task Manager client
        self.task_manager_client = TaskManagerClient(defines.Tasks.pipeline_inspection.id)
        self.task_manager_client.is_enabled = False

        # Landmarks client
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.loginfo(f"{rospy.get_name()}: Waiting for Landmark Server...")
        rospy.wait_for_service("send_positions")
        rospy.loginfo(f"{rospy.get_name()}: Connected to Landmark Server!")
        self.object = self.landmarks_client("pipeline").object

        # TF listener to get TF from body to SPP
        tf_listener = tf.TransformListener()
        rospy.loginfo(
            f"{rospy.get_name()}: Waiting for base link to UDFC TF...")
        tf_listener.waitForTransform('udfc_link', 'base_link', rospy.Time(),
                                     rospy.Duration(1.0))
        rospy.loginfo(f"{rospy.get_name()}: Received base link to UDFC TF!")
        self.UDFC_offset, _ = tf_listener.lookupTransform(
            'udfc_link', 'base_link', rospy.Time(0))

        # =====[Publishers]===== #
        # Current state of the FSM
        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        # Direct publisher to thrust for anchoring to the docking station
        self.thrust_pub = rospy.Publisher(
            rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)

        # =====[Subscribers]===== #
        # Odometry
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_pose_cb)

    def odom_pose_cb(self, msg):
        body_pose = msg.pose.pose

        self.odom_pose.position.x = body_pose.position.x + self.UDFC_offset[0]
        self.odom_pose.position.y = body_pose.position.y + self.UDFC_offset[1]
        self.odom_pose.position.z = body_pose.position.z + self.UDFC_offset[2]
