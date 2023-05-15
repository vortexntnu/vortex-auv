#! /usr/bin/env python3

import rospy
import actionlib
import tf
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped
from vortex_msgs.msg import (dpAction, dpGoal, dpResult)
from nav_msgs.msg import Odometry


class WaypointDPInterface:

    def __init__(self):
        rospy.init_node('waypoint_dp_interface', anonymous=False)

        # Attributes
        self.init_wp = False
        self.init_z = False

        self.ownstate = PoseStamped()
        self.pose_target = PoseStamped()

        self.nav_goal = PoseStamped()
        self.nav_goal_z = 0.0

        self.dp_goal = dpAction()

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
                         queue_size=10)

        # DP server and client
        dp_action_server = "DpAction"
        self.dp_client = actionlib.SimpleActionClient(dp_action_server,
                                                      dpAction)
        rospy.loginfo("Waiting for DP Server...")
        self.dp_client.wait_for_server()

        rospy.Subscriber("/DpAction/result", dpResult, self.dp_goal_cb)

        self.reached_dp_goal = False
        self.current_goal_pose = Pose()

        rospy.loginfo("Waypoint Interface Initialized!")

    def nav_goal_to_dp_setpoint(self, enable_z=False):
        if not self.init_wp:
            rospy.set_param("/DP/Enable", True)
            rospy.set_param("/setpoint/DOF", [1, 1, 0, 0, 0, 1])
            self.init_wp = True

        if enable_z and not self.init_z:
            rospy.set_param("/setpoint/DOF", [1, 1, 1, 0, 0, 1])
            self.init_z = True

        try:
            self.tf_listener.waitForTransform('odom',
                                              self.nav_goal.header.frame_id,
                                              rospy.Time(),
                                              rospy.Duration(4.0))
            self.pose_target = self.tf_listener.transformPose(
                'odom', self.nav_goal)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            pass

        self.dp_position = [
            self.pose_target.pose.position.x, self.pose_target.pose.position.y,
            self.pose_target.pose.position.z
        ]

        if self.init_z:
            self.dp_position[2] = self.nav_goal_z
            self.pose_target.pose.position.z = self.nav_goal_z

        quaternion = (self.pose_target.pose.orientation.x,
                      self.pose_target.pose.orientation.y,
                      self.pose_target.pose.orientation.z,
                      self.pose_target.pose.orientation.w)

        self.dp_orientation = euler_from_quaternion(quaternion)

        self.dp_client.wait_for_server()
        rospy.set_param("/setpoint/position", [
            float(self.dp_position[0]),
            float(self.dp_position[1]),
            float(self.dp_position[2])
        ])
        rospy.set_param("/setpoint/orientation", self.dp_orientation)

    def send_dp_goal(self):
        dp_goal = dpGoal()
        dp_goal.x_ref = self.pose_target.pose
        if self.init_z:
            dp_goal.DOF = [1, 1, 1, 0, 0, 1]
        else:
            dp_goal.DOF = [1, 1, 0, 0, 0, 1]

        self.dp_client.wait_for_server()
        self.dp_client.send_goal(dp_goal)

    def nav_goal_cb(self, msg):
        self.nav_goal = msg
        self.nav_goal_to_dp_setpoint()
        self.send_dp_goal()

    def nav_goal_z_cb(self, msg):
        self.nav_goal_z = msg.data
        self.nav_goal_to_dp_setpoint(enable_z=True)
        self.send_dp_goal()

    def odom_cb(self, msg):
        self.ownstate = msg

    def dp_goal_cb(self, msg):
        self.dp_goal = msg


if __name__ == '__main__':
    WPDPI = WaypointDPInterface()

    while not rospy.is_shutdown():
        rospy.spin()
