#!/usr/bin/env python
# coding: UTF-8

import rospy
from smach import StateMachine, Sequence
from std_msgs.msg import String
from math import pi
from helper import dp_move, los_move

rospy.init_node('the_great_testing_node')

sm = Sequence(outcomes=['preempted', 'succeeded', 'aborted'], connector_outcome='succeeded')

with sm:

    # Add desired waypoints (including controller choice) here
    Sequence.add('ONE', dp_move(0, 0, yaw_rad=pi))
    Sequence.add('TWO', los_move(4, 0))
    Sequence.add('THREE', los_move(1, 0))
    Sequence.add('FOUR', dp_move(0, 0, yaw_rad=pi))


arm_pub = rospy.Publisher('/mcu_arm', String)

arm_pub.publish("data: 'arm'")      # thrusters must be armed before use
sm.execute()
arm_pub.publish("data: 'disarm'")   # and disarmed