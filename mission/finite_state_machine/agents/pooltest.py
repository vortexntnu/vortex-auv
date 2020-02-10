#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence
from smach_ros import SimpleActionState
from geometry_msgs.msg import Pose, Point, Quaternion
from vortex_msgs.msg import MoveGoal, MoveAction
from tf.transformations import quaternion_from_euler


def dp_move(x, y, z=-0.5, yaw=0):
    goal = MoveGoal()

    goal.controller_name = 'DP'
    goal.target_pose.position = Point(x, y, z)
    goal.target_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw))

    return SimpleActionState('move', MoveAction, goal=goal)


def los_move(x, y, z=-0.5):
    goal = MoveGoal()

    goal.controller_name = 'LOS'
    goal.target_pose.position = Point(x, y, z)

    return SimpleActionState('move', MoveAction, goal=goal)


rospy.init_node('the_great_testing_node')

sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')

with sm:

    Sequence.add('ONE', dp_move(1, 0))
    Sequence.add('TWO', los_move(-7, 0))
    Sequence.add('THREE', los_move(0, 0))
    Sequence.add('FOUR', dp_move(2, 0))

sm.execute()