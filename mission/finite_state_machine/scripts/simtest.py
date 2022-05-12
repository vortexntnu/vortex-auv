#!/usr/bin/python3
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence
from std_msgs.msg import String
from math import pi
from fsm_helper import dp_move, los_move, patrol_sequence
from smach_ros import IntrospectionServer


rospy.init_node("simtest_fsm")

simtest_sm = patrol_sequence(
    [dp_move(0, 0), los_move(4, 0), los_move(1, 0), dp_move(0, 0, yaw_rad=pi)]
)

intro_server = IntrospectionServer(str(rospy.get_name()), simtest_sm, "/SM_ROOT")
intro_server.start()


try:
    simtest_sm.execute()
    intro_server.stop()

except Exception as e:
    rospy.loginfo("Pooltest failed: %s" % e)
