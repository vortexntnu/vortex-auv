#!/usr/bin/python3

import rospy

from smach import StateMachine

from mission.TAC_tasks.scripts.docking.DockingExecute import DockingExecute
from mission.TAC_tasks.scripts.docking.DockingStandby import DockingStandby

from task_manager_defines import defines

import dynamic_reconfigure.client


class Docking():
    def __init__(self):
        rospy.init_node("docking_fsm")

        self.isEnabled = False
        self.task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=3,
            config_callback=self.task_manager_cb)

    def main(self):
        # task manager
        if self.isEnabled == False:
            return

        rospy.loginfo('STARTING DOCKING')

        docking_sm = StateMachine(outcomes=['done'],
                                  input_keys=['isEnabled'],
                                  output_keys=['isEnabled'])
        with docking_sm:

            StateMachine.add("DOCKING_EXECUTE",
                             DockingExecute(),
                             transitions={
                                 'succeeded': 'DOCKING_STANDBY',
                                 'preempted': 'done'
                             },
                             remapping={'isEnabled': 'isEnabled'})

            StateMachine.add("DOCKING_STANDBY",
                             DockingStandby(),
                             transitions={'succeeded': 'done'},
                             remapping={'isEnabled': 'isEnabled'})

        try:
            docking_sm.execute()

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)

    def task_manager_cb(self, config):
        activated_task_id = config["Tac_states"]

        if defines.Tasks.docking.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        rospy.logwarn(f"Docking Enabled: {self.isEnabled} ")

        return config