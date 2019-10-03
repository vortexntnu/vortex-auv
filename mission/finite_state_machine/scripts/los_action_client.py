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

        # creates a goal to send to the action server
        goal = LosPathFollowingGoal()

        # create line segment
        goal.next_waypoint.x = 10.0
        goal.next_waypoint.y = -2.0
        goal.prev_waypoint.x = -5.0
        goal.prev_waypoint.y = 5.0

        # set speed goal
        goal.forward_speed.linear.x = 2.0

        # sphere of acceptance
        goal.sphereOfAcceptance = 1.0

        rospy.loginfo("sending goal pose to Action Server")

        # Send goal
        self.client.send_goal(goal)

        # waits for the server to finish performing the action
        self.client.wait_for_result()

        # Prints out the result of executing the action
        return self.client.get_result()


if __name__ == '__main__':
    try:
        PathFollowingClient()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
