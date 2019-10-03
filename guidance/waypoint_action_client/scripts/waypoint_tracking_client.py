#!/usr/bin/env python
# Written by Kristoffer Rakstad Solberg, Student
# Copyright (c) 2020 Manta AUV, Vortex NTNU.
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

        # check for waypoint file
        if rospy.is_shutdown():
                rospy.logerr('ROS master not running!')
                sys.exit(-1)
                rospy.logerr('Found waypoint file!')
        if rospy.has_param('~filename'):

                filename = rospy.get_param('~filename')
        else:
                raise rospy.ROSException('No filename found')
        
        # Start tracking by reading waypoints from file
        load = PrepareWaypoints()
        load.read_from_file(filename)

        # Create a list to hold the waypoint poses
        self.waypoints = list()

        # waypoint goal count
        self.goal_cnt = 0

        
        for tup in range(0,len(load.waypoints)):
            xyzRPY = load.waypoints[tup]
            xyz = xyzRPY[0:3]
            RPY = xyzRPY[3:6]
            
            #Convert attitude from euler to quaternion
            q_angle = quaternion_from_euler(RPY[0], RPY[1], RPY[2], axes = 'sxyz')
            q = Quaternion(*q_angle)

            # append waypoints
            self.waypoints.append(Pose(Point(xyz[0], xyz[1], xyz[2]),q))


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

        # creates a goal to send to the action server
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
