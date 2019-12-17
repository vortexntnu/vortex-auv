#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np

# action message
import actionlib
from vortex_msgs.msg import LosPathFollowingAction, LosPathFollowingGoal, LosPathFollowingResult, LosPathFollowingFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion


class PathFollowingClient():

    def __init__(self):


        rospy.init_node('waypoint_client')

        #Create action client
        self.client = actionlib.SimpleActionClient('los_path',LosPathFollowingAction)
        rospy.loginfo("Waiting for LOS path action server...")
        wait = self.client.wait_for_server(rospy.Duration(15.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to LOS path server")
        rospy.loginfo("Starting goals achievements ...")
        self.path_client()

    def path_client(self):

        """
            action client guide
            https://github.com/strawlab/ros_common/blob/master/actionlib/src/actionlib/simple_action_client.py
        """

        # creates a goal to send to the action server
        _goal = LosPathFollowingGoal()

        # create line segment
        _goal.next_waypoint.x = 3.0 # 3.0
        _goal.next_waypoint.y = 1.0 # -2.0
        _goal.prev_waypoint.x = 1.0 # 0.0
        _goal.prev_waypoint.y = 1.0 # 0.0

        # set speed goal
        _goal.forward_speed.linear.x = 0.20

        # set desired depth
        _goal.desired_depth.z = -0.50

        # sphere of acceptance
        _goal.sphereOfAcceptance = 0.3

        rospy.loginfo("sending goal pose to Action Server")

        # Send goal
        self.client.send_goal(_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def done_cb(self, status, result):
        # Gets called on transition state
        if status == 1:
            rospy.loginfo("active tracking")
        if status == 2:
            rospy.loginfo("preempted")
        if status == 3:
            rospy.loginfo("Goal pose reached")
        if status == 4:
            rospy.loginfo("Goal aborted")
        if status == 5:
            rospy.loginfo("Goal rejected")

        # creates a goal to send to the action server
        _goal = LosPathFollowingGoal()

        # create line segment
        _goal.next_waypoint.x = 7.0 # 6.0
        _goal.next_waypoint.y =-2.0 # 2.0
        _goal.prev_waypoint.x = 3.0 # 3.0
        _goal.prev_waypoint.y =-2.0 #-2.0

        # set speed goal
        _goal.forward_speed.linear.x = 0.20

        # set desired depth
        _goal.desired_depth.z = -0.50

        # sphere of acceptance
        _goal.sphereOfAcceptance = 0.3

        rospy.loginfo("sending goal pose to Action Server")

        self.client.send_goal(_goal, self.done_cb, self.active_cb, self.feedback_cb)

    def active_cb(self):
        #gets called on transitions to Active.
        # status SUCCEDED=1
        rospy.loginfo("active tracking")

    def feedback_cb(self, feedback):
        # gets called whenever feedback
        # for this goal is received.
        rospy.loginfo(feedback)



if __name__ == '__main__':
    try:
        PathFollowingClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
