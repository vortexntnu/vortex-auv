#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from pipeline_following_DP import PipelineExecute, PipelineStandby
import dynamic_reconfigure.client
from task_manager_defines import defines


def main():
    rospy.init_node("tac_pipeline_fsm")
    # rospy.wait_for_message()
    # rospy.wait_for_service()

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

    #intro_server = IntrospectionServer(
    #    str(rospy.get_name()), pipeline_following_sm, "/SM_ROOT"
    #)
    #intro_server.start()

    try:
        #Execute SMACH plan
        pipeline_following_sm.execute()
        rospy.loginfo("Exiting Pipeline Following")
        #intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


def callback(self, config):
    rospy.loginfo(
        """Client: state change request: {Tac_states}""".format(**config))
    activated_task_id = config["Tac_states"]

    if defines.Tasks.valve_vertical.id == activated_task_id:
        self.isEnabled = True
    else:
        self.isEnabled = False
    print(f"isEnabled: {self.isEnabled} ")

    return config


if __name__ == "__main__":
    while not rospy.is_shutdown():
        enabled = True  #rospy.get_param("/tasks/pipeline_inspection")
        if enabled:
            rospy.loginfo('STARTING PIPELINE FOLLOWING')
            main()
            rospy.loginfo('PIPELINE FOLLOWING ENDED')
