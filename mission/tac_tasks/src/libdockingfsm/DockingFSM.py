#!/usr/bin/python3

import rospy

from smach import StateMachine

from libdockingfsm.DockingExecute import DockingExecute
from libdockingfsm.DockingStandby import DockingStandby

from task_manager_defines import defines


class DockingFSM():

    def __init__(self):
        rospy.init_node("docking_fsm")

    def main(self):
        docking_sm = StateMachine(outcomes=['done'])
        with docking_sm:
            StateMachine.add("DOCKING_EXECUTE",
                             DockingExecute(),
                             transitions={
                                 'succeeded': 'DOCKING_STANDBY',
                                 'preempted': 'done'
                             })

            StateMachine.add("DOCKING_STANDBY",
                             DockingStandby(),
                             transitions={'succeeded': 'done'})
        try:
            docking_sm.execute()

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)
