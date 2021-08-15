#!/usr/bin/python3

from math import pi

import rospy
from smach import StateMachine, Sequence
from smach_ros import IntrospectionServer
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from helper import create_sequence, point, pose, ControlModeEnum
from common_states import GoToState


if __name__ == "__main__":
    rospy.init_node("go_home_fsm")
    state = GoToState(pose(0, 0, 0.7, 0, 0, 0))
    state.execute(None)
