#! /usr/bin/env python3

import rospy
import tf

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry

from DPClient import DPClient

class WaypointInterface:
    def __init__(self):
        rospy.init_node('waypoint_interface', anonymous=False)

        # Attributes
        self.init_z = False

        self.nav_goal = PoseStamped()
        self.nav_goal_z = 0.0

        # TF
        self.tf_listener = tf.TransformListener()

        # Subscribers
        rospy.Subscriber("/move_base_simple/goal",
                         PoseStamped,
                         self.nav_goal_cb,
                         queue_size=1)
        rospy.Subscriber("/move_base_simple/goal/z",
                         Float32,
                         self.nav_goal_z_cb,
                         queue_size=1)
        rospy.Subscriber("/odometry/filtered",
                         Odometry,
                         self.odom_cb,
                         queue_size=1)

        self.dp_client = DPClient()

        rospy.loginfo("Waypoint Interface Initialized!")

    def nav_goal_to_dp_goal(self, enable_z=False):
        if not self.dp_client.get_enabled_status:
            self.dp_client.enable()

            self.dp_client.DOF = [1, 1, 0, 0, 0, 1]

        try:
            self.tf_listener.waitForTransform('odom',
                                              self.nav_goal.header.frame_id,
                                              rospy.Time(),
                                              rospy.Duration(4.0))
            self.dp_client.goal_pose = self.tf_listener.transformPose(
                'odom', self.nav_goal).pose
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            rospy.logwarn("Could not retreive transform to 'odom' frame...")
            return False
        
        if enable_z or self.init_z:
            self.dp_client.DOF = [1, 1, 1, 0, 0, 1]
            self.init_z = True

            self.dp_client.goal_pose.position.z = self.nav_goal_z
        
        return True

    def nav_goal_cb(self, posestamped_msg):
        self.nav_goal = posestamped_msg
        self.nav_goal_to_dp_goal()
        self.dp_client.send_goal()

    def nav_goal_z_cb(self, float32_msg):
        self.nav_goal_z = float32_msg.data
        self.nav_goal_to_dp_goal(enable_z=True)
        self.dp_client.send_goal()
