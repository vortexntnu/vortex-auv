#!/usr/bin/python3

import rospy
import tf

from geometry_msgs.msg import Pose, Wrench, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from landmarks.srv import request_position
from vortex_msgs.msg import ObjectPosition

from dp_client_py.DPClient import DPClient

from task_manager_defines import defines  # type: ignore
from task_manager_client.TaskManagerClient import TaskManagerClient  # type: ignore


class Docking:

    def __init__(self):
        # =====[Attributes]===== #
        self.rate = rospy.Rate(10)

        # TF listener to get TF from body to SPP
        self.tf_listener = tf.TransformListener()

        # =====[Publishers]===== #
        # Current state of the FSM
        self.obj_pub = rospy.Publisher("/object_positions_in", ObjectPosition, queue_size=1)

        # Direct publisher to thrust for anchoring to the docking station
        self.thrust_pub = rospy.Publisher(
            rospy.get_param("/thrust/thrust_topic"), Wrench, queue_size=1)

        # =====[Subscribers]===== #
        # Odometry
        # rospy.Subscriber("/odometry/filtered", Odometry, self.odom_pose_cb)
        rospy.Subscriber("/aruco_udfc_pose", PoseStamped, self.odom_pose_cb)

    def odom_pose_cb(self, msg):
        body_pose = msg.pose.pose

        rospy.loginfo(
            f"{rospy.get_name()}: Waiting for base link to SPP TF...")
        self.tf_listener.waitForTransform('aruco_udfc_link', 'odom', rospy.Time(),
                                     rospy.Duration(1.0))
        rospy.loginfo(f"{rospy.get_name()}: Received base link to SPP TF!")
        self.SPP_offset, _ = self.tf_listener.lookupTransform(
            'SPP_link', 'base_link', rospy.Time(0))
        
        
        self.odom_pose.position.x = body_pose.position.x + self.SPP_offset[0]
        self.odom_pose.position.y = body_pose.position.y + self.SPP_offset[1]
        self.odom_pose.position.z = body_pose.position.z + self.SPP_offset[2]
