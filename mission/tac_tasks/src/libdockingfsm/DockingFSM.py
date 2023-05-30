#!/usr/bin/python3

import rospy

from smach import StateMachine

from libdockingfsm.DockingConverge import DockingConverge
from libdockingfsm.DockingExecute import DockingExecute
from libdockingfsm.DockingStandby import DockingStandby

class DockingFSM():

    def __init__(self):
        rospy.init_node("docking_fsm")

    def main(self):
        docking_sm = StateMachine(outcomes=['done'])
        with docking_sm:
            StateMachine.add("DOCKING_CONVERGE_1",
                             DockingConverge(1.0, 0.5),
                             transitions={
                                 'succeeded': 'DOCKING_CONVERGE_2',
                                 'preempted': 'done'
                             })
            StateMachine.add("DOCKING_CONVERGE_2",
                             DockingConverge(0.15, 0.05, True),
                             transitions={
                                 'succeeded': 'DOCKING_EXECUTE',
                                 'preempted': 'done'
                             })
            StateMachine.add("DOCKING_EXECUTE",
                             DockingExecute(True),
                             transitions={
                                 'succeeded': 'DOCKING_CONVERGE_3',
                                 'preempted': 'done'
                             })
            StateMachine.add("DOCKING_CONVERGE_3",
                             DockingConverge(1.0, 0.1, True),
                             transitions={
                                 'succeeded': 'done',
                                 'preempted': 'done'
                             })
        try:
            docking_sm.execute()

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)