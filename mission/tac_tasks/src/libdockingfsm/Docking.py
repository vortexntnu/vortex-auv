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


class Docking:

    def __init__(self):
        # =====[Attributes]===== #
        self.rate = rospy.Rate(10)
        self.sending_rate = rospy.Rate(1)

        self.odom_pose = Pose()

        # =====[Params]===== #
        # Height where the DP is turned off to minimize issues because of drift close to platform
        self.final_descent_height = rospy.get_param(
            "/tac/docking/final_descent_height")

        self.coeff_a = rospy.get_param("/tac/docking/error_coefficients/a")
        self.coeff_b = rospy.get_param("/tac/docking/error_coefficients/b")

        # =====[Services, clients, handles]===== #
        # DP action client
        self.dp_client = DPClient()
        self.dp_client.goal.DOF = [1, 1, 1, 0, 0, 1]

        # Task Manager client
        self.task_manager_client = TaskManagerClient(defines.Tasks.docking.id)
        self.task_manager_client.is_enabled = False

        # Landmarks client
        self.landmarks_client = rospy.ServiceProxy("send_positions",
                                                   request_position)
        rospy.loginfo(f"{rospy.get_name()}: Waiting for Landmark Server...")
        rospy.wait_for_service("send_positions")
        rospy.loginfo(f"{rospy.get_name()}: Connected to Landmark Server!")
        self.object = self.landmarks_client("docking_point").object

        # TF listener to get TF from body to SPP
        tf_listener = tf.TransformListener()
        rospy.loginfo(
            f"{rospy.get_name()}: Waiting for base link to SPP TF...")
        tf_listener.waitForTransform('SPP_link', 'base_link', rospy.Time(),
                                     rospy.Duration(1.0))
        rospy.loginfo(f"{rospy.get_name()}: Received base link to SPP TF!")
        self.SPP_offset, _ = tf_listener.lookupTransform(
            'SPP_link', 'base_link', rospy.Time(0))

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

        self.odom_pose.position.x = body_pose.position.x + self.SPP_offset[0]
        self.odom_pose.position.y = body_pose.position.y + self.SPP_offset[1]
        self.odom_pose.position.z = body_pose.position.z + self.SPP_offset[2]
