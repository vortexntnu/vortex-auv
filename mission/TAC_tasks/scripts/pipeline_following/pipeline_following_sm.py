#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from pipeline_following import PipelineExecute, PipelineStandby
from task_manager_defines import defines
import dynamic_reconfigure.client


class PipelineFollowing():

    def __init__(self):
        rospy.init_node("tac_pipeline_fsm")

        # initializing task manager client
        self.isEnabled = False
        task_manager_client = dynamic_reconfigure.client.Client(
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

        pipeline_following_sm = StateMachine(outcomes=["done"])
        with pipeline_following_sm:

            StateMachine.add("PIPELINE_STANDBY",
                             PipelineStandby(),
                             transitions={
                                 "aborted": "done",
                                 "succeeded": "PIPELINE_EXECUTE"
                             }),

            StateMachine.add("PIPELINE_EXECUTE",
                             PipelineExecute(),
                             transitions={"aborted": "PIPELINE_STANDBY"})

        try:
            #Execute SMACH plan
            pipeline_following_sm.execute()
            rospy.loginfo("Exiting Pipeline Following")

        except Exception as e:
            rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    execute = PipelineFollowing()
    execute.main()
    rospy.loginfo('PIPELINE FOLLOWING ENDED')
    rospy.spin()
