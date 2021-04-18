#!/usr/bin/env python
# coding: UTF-8

from math import pi

import rospy
from smach import StateMachine, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import quaternion_from_euler

from helper import create_sequence, point, pose
from common_states import GoToState, dp_state, los_state, vel_state


def visit_waypoints():

    pose1 = Pose()
    pose1.position.z = 1.0
    pose1.position.x = 2

    pose2 = Pose()
    pose2.position.z = 0.5

    sm = create_sequence(
        [
            GoToState(pose1),
            GoToState(pose2),
        ]
    )
    introspection_server = IntrospectionServer(str(rospy.get_name()), sm, '/sm_root')

    introspection_server.start()
    sm.execute()

def test_restoring():
    twist = Twist()
    state = vel_state(twist)
    res = state.execute(None)
    rospy.loginfo(str(res))

def test_vel():
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.5
    state = vel_state(twist)
    state.execute(None)

def test_dp():
    test_pose = pose(0, 0, 1, 0, 0, 0)
    state = dp_state(test_pose)
    res = state.execute(None)

def test_los():
    goal_pos = Point()
    goal_pos.z = 1
    state = los_state(goal_pos)
    state.execute(None)


if __name__ == "__main__":
    rospy.init_node("pooltest_fsm", log_level=rospy.DEBUG)
    test_dp()
