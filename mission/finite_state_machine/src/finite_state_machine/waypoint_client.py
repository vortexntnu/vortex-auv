#!/usr/bin/env	python
import	rospy
from	move_base_msgs.msg	import	MoveBaseAction,	MoveBaseGoal
from    tf.transformations import quaternion_from_euler


class WaypointClient():

    def __init__(self):

        pass

    def trackNewWaypoint(self, w):

        # object constructor
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = "map"
        goal_pose.target_pose.header.stamp = rospy.Time.now()

        # adding coordiantes
        goal_pose.target_pose.pose.position.x = w[1][0]
        goal_pose.target_pose.pose.position.y = w[1][1]
        goal_pose.target_pose.pose.position.z = w[1][2]

        #Convert attitude from euler to quaternion
        quat = quaternion_from_euler(w[2][0], w[2][1], w[2][2], axes = 'sxyz')

        # adding attitude
        goal_pose.target_pose.pose.orientation.x = quat[0]
        goal_pose.target_pose.pose.orientation.y = quat[1]
        goal_pose.target_pose.pose.orientation.z = quat[2]
        goal_pose.target_pose.pose.orientation.w = quat[3]

        return goal_pose
