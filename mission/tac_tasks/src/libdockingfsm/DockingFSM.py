#!/usr/bin/python3

import rospy

from smach import StateMachine

from libdockingfsm.DockingExecute import DockingExecute
from libdockingfsm.DockingStandby import DockingStandby

from task_manager_defines import defines

from dp_client_py.DPClient import DPClient


class DockingFSM():

    def __init__(self):
        rospy.init_node("docking_fsm")
        self.dp_client = DPClient()

    def main(self):
        docking_sm = StateMachine(outcomes=['done'])
        with docking_sm:
            StateMachine.add("DOCKING_EXECUTE",
                             DockingExecute(self.dp_client),
                             transitions={
                                 'succeeded': 'DOCKING_STANDBY',
                                 'preempted': 'done'
                             })

            StateMachine.add("DOCKING_STANDBY",
                             DockingStandby(self.dp_client),
                             transitions={'succeeded': 'done'})
        try:
            docking_sm.execute()

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)
