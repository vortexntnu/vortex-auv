#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2019 Manta AUV, Vortex NTNU.
# All rights reserved.

import rospy
import math

import actionlib
from load_waypoints import PrepareWaypoints
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians, pi


class WaypointClient():

    def __init__(self):


        rospy.init_node('waypoint_client')

        # waypoint goal count
        self.goal_cnt = 0

        # Create a list to hold the target quaternions (orientations)
        quaternions = list()

        # First define the corner orientations as Euler angles
        euler_angles = (pi/2, pi, 3*pi/2, 0)

        # Then convert the angles to quaterions
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes = 'sxyz')
            q = Quaternion(*q_angle)
            quaternions.append(q)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

        # Append each of the four waypoints to the list. Each waypoint
        # is a pose consisting of a position and orientation in the map frame
        self.waypoints.append(Pose(Point(5.0, -3.0, -3.0), quaternions[0]))
        self.waypoints.append(Pose(Point(8.0, -3.0, -3.0), quaternions[1]))
        self.waypoints.append(Pose(Point(8.0, -10.0, -3.0), quaternions[2]))
        self.waypoints.append(Pose(Point(5.0, -10.0, 0.0), quaternions[3]))

        #Create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(15.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")
        self.movebase_client()


    # Callback triggered if the controller recieves the goal for the client and processes it
    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")


    # Callback triggered by feedback from the controller action server
    def feedback_cb(self, feedback):
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")


    # Checks the status message from the server
    def status_cb(self, status, result):

    #uint8 PENDING=0
    #uint8 ACTIVE=1
    #uint8 PREEMPTED=2
    #uint8 SUCCEEDED=3
    #uint8 ABORTED=4
    #uint8 REJECTED=5
    #uint8 PREEMPTING=6
    #uint8 RECALLING=7
    #uint8 RECALLED=8
    #uint8 LOST=9

        #add a new goal count when goal reached
        self.goal_cnt += 1

        # status PREEMPTED=2
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        # status SUCCEDED=3
        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")

            if self.goal_cnt<len(self.waypoints):
                # produce the next goal
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.waypoints[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.waypoints[self.goal_cnt]))
                self.client.send_goal(next_goal, self.status_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final waypoint have been reached")
                rospy.signal_shutdown("Mission success, shutdown client")
                return

        # status ABORTED
        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            return

        # status REJECTED
        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            return

        # status RECALLED - The goal received a cancel request before it started executing
        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.waypoints[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.waypoints[self.goal_cnt]))

        # Send goal
        self.client.send_goal(goal, self.status_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        WaypointClient()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
