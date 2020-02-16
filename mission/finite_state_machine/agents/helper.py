#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach_ros import SimpleActionState
from smach import StateMachine, Sequence
from geometry_msgs.msg import Point, Quaternion
from vortex_msgs.msg import MoveGoal, MoveAction
from tf.transformations import quaternion_from_euler

def dp_move(x, y, z=-0.5, yaw_rad=0):
    goal = MoveGoal()

    goal.controller_name = 'DP'
    goal.target_pose.position = Point(x, y, z)
    goal.target_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, yaw_rad))

    return SimpleActionState('move', MoveAction, goal=goal)


def los_move(x, y, z=-0.5):
    goal = MoveGoal()

    goal.controller_name = 'LOS'
    goal.target_pose.position = Point(x, y, z)

    return SimpleActionState('move', MoveAction, goal=goal)


def patrol_sequence(states):

    sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')
    counter = 0

    with sm:

        for state in states:
            counter = counter + 1
            sm.add("State-%d" %counter, state)

    return sm

