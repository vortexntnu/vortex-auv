#!/usr/bin/python3

import rospy
import smach
from smach import StateMachine

import actionlib
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

from vortex_msgs.msg import VtfPathFollowingAction, VtfPathFollowingGoal
from fsm_helper import create_circle_coordinates


class Circle(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=["preempted", "succeeded", "aborted"]
        )

        vtf_action_server = "/controllers/vtf_action_server"
        self.vtf_client = actionlib.SimpleActionClient(
            vtf_action_server, VtfPathFollowingAction
        )

        self.state_pub = rospy.Publisher("/fsm/state", String, queue_size=1)

        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_cb)
        self.odom = Odometry()

    def odom_cb(self, msg):
        self.odom = msg

    def execute(self, userdata):
        self.state_pub.publish("circle")

        goal = VtfPathFollowingGoal()
        start = self.odom.pose.pose.position
        print(userdata)
        #centre = Point(
        #    userdata.pole.objectPose.pose.position.x,
        #    userdata.pole.objectPose.pose.position.y,
        #    userdata.pole.objectPose.pose.position.z,
        #)
        centre = Point(
            self.odom.pose.pose.position.x + 3,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z,
        )
        goal.waypoints = create_circle_coordinates(start, centre, 330)
        goal.forward_speed = rospy.get_param("/fsm/medium_speed")
        goal.heading_point.x = centre.x # ??
        goal.heading_point.y = centre.y # ??
        goal.heading_point.z = centre.z # ??
        #goal.heading_point.x = userdata.pole.objectPose.pose.position.x
        #goal.heading_point.y = userdata.pole.objectPose.pose.position.y
        #goal.heading_point.z = userdata.pole.objectPose.pose.position.z

        goal.heading = "point_dependent_heading"

        self.vtf_client.wait_for_server()
        self.vtf_client.send_goal(goal)
        rate = rospy.Rate(1)
        rate.sleep()
        while not rospy.is_shutdown():
            if (
                self.vtf_client.simple_state
                == actionlib.simple_action_client.SimpleGoalState.DONE
            ):
                break
            rate.sleep()
        return "succeeded"