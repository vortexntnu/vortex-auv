#!/usr/bin/python3

import rospy
import smach

import actionlib
#from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from fsm_helper import create_circle_coordinates


class Circle(smach.State):

    def __init__(self, ccw=False, is_path_dependent=True):
        smach.State.__init__(self,
                             outcomes=["preempted", "succeeded", "aborted"])

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(vtf_action_server,
                                                       VtfPathFollowingAction)

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

        self.ccw = ccw
        self.is_path_dependent = is_path_dependent

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("circle")

        goal = VtfPathFollowingGoal()
        start = self.odom.pose.pose.position
        centre = Point(
            self.odom.pose.pose.position.x + 1,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z,
        )
        goal.waypoints = create_circle_coordinates(start, centre, 330,
                                                   self.ccw)
        goal.forward_speed = rospy.get_param("/fsm/fast_speed")
        goal.heading_point.x = centre.x
        goal.heading_point.y = centre.y
        goal.heading_point.z = centre.z

        if self.is_path_dependent:
            goal.heading = "path_dependent_heading"  # path or point
        else:
            goal.heading = "point_dependent_heading"  # path or point

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if (self.vtf_client.simple_state ==
                    actionlib.simple_action_client.SimpleGoalState.DONE):
                break
            rate.sleep()
        return "succeeded"
