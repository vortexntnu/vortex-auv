#!/usr/bin/env python
# coding: UTF-8

"""
"""


import rospy
from smach import StateMachine, Concurrence, CBState, cb_interface
from smach_ros import SimpleActionState

from helper import allign_with_target



@cb_interface(outcomes=[])
def check_side():
    pass


@cb_interface(outcomes=[])
def switch_side():
    pass


@cb_interface(outcomes=[])
def allign_with_gman():
    pass


@cb_interface(outcomes=[])
def go_through_gate():
    pass 


@cb_interface(outcomes=[])
def do_360():
    pass



gate_sm = StateMachine(
    outcomes=['succeeded', 'failed', 'preempted'],
    input_keys=[],
    output_keys=[]
)

with gate_sm:

    StateMachine.add(
        'ALLIGN_WITH_GATE', 
        allign_with_target('GATE'),
        transitions={
            'succeeded': 'CHECK_SIDE'
        }
    )
    StateMachine.add(
        'CHECK_SIDE', 
        CBState(check_side),
        transitions={
            'correct': 'ALLIGN_WITH_GMAN',
            'wrong': 'SWITCH_SIDE'
        }
    )
    StateMachine.add(
        'SWITCH_SIDE', 
        CBState(switch_side),
        transitions={
            'succeeded': 'ALLIGN_WITH_GATE'
        }
    )
    StateMachine.add(
        'ALLIGN_WITH_GMAN', 
        CBState(allign_with_gman),
        transitions={
            'succeeded': 'GO_THROUGH_GATE'
        }
    )
    StateMachine.add(
        'GO_THROUGH_GATE', 
        CBState(go_through_gate),
        transitions={
            'succeeded': 'DO_360'
        }
    )
    StateMachine.add(
        'DO_360', 
        CBState(do_360),
        transitions={
            'succeeded': 'succeeded'
        }
    )




