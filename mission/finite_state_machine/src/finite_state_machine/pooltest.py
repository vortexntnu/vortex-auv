#!/usr/bin/env python
# coding: UTF-8

from math import pi

import rospy
from smach import StateMachine, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from helper import create_sequence, point, pose
from common_states import GoToState, dp_state, los_state, vel_state


def visit_waypoints():

    test_pose = Pose()
    test_pose.position.z = -0.5

    sm = create_sequence(
        [
            los_state(point(2, 0, -0.5)),
            los_state(point(0, 0, -0.5)),
            dp_state(test_pose),
        ]
    )
    introspection_server = IntrospectionServer(str(rospy.get_name()), sm, '/sm_root')

    introspection_server.start()
    sm.execute()


if __name__ == "__main__":
    rospy.init_node("pooltest_fsm")
    visit_waypoints()
    rospy.spin()
