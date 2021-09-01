#!/usr/bin/env python

import rospy
from smach import Sequence
from fsm_helper import dp_move, los_move
from smach_ros import SimpleActionState
from vortex_msgs.msg import MoveAction

def main():
    """
    A simple state machine which moves the AUV in a rectangle pattern using dp_move() and ls_move.
    This state machine is inteded for testing/data collection.
    """
    rospy.init_node('four_corner_sm')

    move_action_server = 'guidance/move'

    rectangle_length = 4

    four_corner_sm = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')

    with four_corner_sm:

        Sequence.add('1ST_CORNER', dp_move(0,0))
        Sequence.add('2ND_CORNER', los_move(rectangle_length,0))
        Sequence.add('3RD_CORNER', los_move(rectangle_length,rectangle_length))
        Sequence.add('4TH_CORNER', los_move(0,rectangle_length))
        Sequence.add('ORIGIN_RETURN',dp_move(0,0))        

    try:
        four_corner_sm.execute()
        

    except Exception as e:
        rospy.loginfo("Pooltest failed: %s" % e)



if __name__ == '__main__':
    main()
