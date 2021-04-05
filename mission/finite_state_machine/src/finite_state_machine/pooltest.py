#!/usr/bin/env python
# coding: UTF-8

from math import pi

import rospy
from smach import StateMachine, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist

from helper import create_sequence, point, pose
from common_states import GoToState, dp_state, los_state, vel_state


def visit_waypoints():

    test_pose = Pose()
    test_pose.position.z = -0.5

    sm = create_sequence(
        [
            GoToState(test_pose),
            los_state(point(2, 0, -0.5)),
            los_state(point(0, 0, -0.5)),
            dp_state(test_pose),
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
    
def test_dp():
    test_pose = Pose()
    test_pose.position.z = -0.5
    state = dp_state(test_pose)
    res = state.execute(None)
    rospy.loginfo(str(res))

if __name__ == "__main__":
    rospy.init_node("pooltest_fsm")
    test_restoring()
