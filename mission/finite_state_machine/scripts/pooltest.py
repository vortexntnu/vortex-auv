#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence
from std_msgs.msg import String
from math import pi
from fsm_helper import dp_move, los_move, patrol_sequence


rospy.init_node('pooltest_fsm')
thruster_armer = rospy.Publisher('/mcu_arm', String, queue_size=10)

patrol_sm = patrol_sequence([
    dp_move(0, 0, yaw_rad=pi),
    los_move(4, 0),
    los_move(1, 0),
    dp_move(0, 0, yaw_rad=pi)
])

try:
    thruster_armer.publish("data: 'arm'")       # arm thrusters
    rospy.sleep(3)
    patrol_sm.execute()

except Exception as e:
    rospy.loginfo("Pooltest is stoppted: %s" % e) 

finally: 
    thruster_armer.publish("data: 'disarm'")    # disarm thrusters
