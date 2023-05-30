#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
import dynamic_reconfigure.client

from libpipelinefsm.PipelineExecute import PipelineExecute
from libpipelinefsm.PipelineStandby import PipelineStandby

from task_manager_defines import defines


class PipelineFollowingFSM():
    def __init__(self):
        rospy.init_node("tac_pipeline_fsm")

        # initializing task manager client
        self.isEnabled = False
        self.task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=self.task_manager_cb)

    def task_manager_cb(self, config):
        rospy.loginfo(
            """Client: state change request: {Tac_states}""".format(**config))
        activated_task_id = config["Tac_states"]

        if defines.Tasks.pipeline_inspection.id == activated_task_id:
            self.isEnabled = True
        else:
            self.isEnabled = False
        print(f"isEnabled: {self.isEnabled} ")

        return config

    def main(self):
        # task manager
        if self.isEnabled == False:
            return

        rospy.loginfo('STARTING PIPELINE FOLLOWING')

        pipeline_following_sm = StateMachine(outcomes=["done"],
                                             input_keys=['isEnabled'],
                                             output_keys=['isEnabled'])

        userdata = pipeline_following_sm.userdata
        userdata.isEnabled = self.isEnabled

        with pipeline_following_sm:

            StateMachine.add("PIPELINE_STANDBY",
                             PipelineStandby(userdata),
                             transitions={
                                 "aborted": "done",
                                 "succeeded": "PIPELINE_EXECUTE"
                             },
                             remapping={'isEnabled': 'isEnabled'})

            StateMachine.add("PIPELINE_EXECUTE",
                             PipelineExecute(userdata),
                             transitions={"aborted": "PIPELINE_STANDBY"},
                             remapping={'isEnabled': 'isEnabled'})

        try:
            #Execute SMACH plan
            pipeline_following_sm.execute()
            rospy.loginfo("Exiting Pipeline Following")

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)
