#!/usr/bin/env python
# coding: UTF-8

from math import pi

import rospy
from smach import StateMachine, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import String

from helper import create_sequence, point, pose
from common_states import GoToState, dp_state, los_state, vel_state


def visit_waypoints():

    sm = create_sequence(
        [
            dp_state(pose(0, 0, -0.5, 0, 0, 0)),
            los_state(point(2, 0, -0.5)),
            los_state(point(0, 0, -0.5)),
            dp_state(pose(0, 0, -0.5, 0, 0, 0)),
        ]
    )
    introspection_server = IntrospectionServer(str(rospy.get_name()), sm, '/sm_root')

    introspection_server.start()
    sm.execute()


if __name__ == "__main__":
    rospy.init_node("pooltest_fsm")
    visit_waypoints()
    rospy.spin()
