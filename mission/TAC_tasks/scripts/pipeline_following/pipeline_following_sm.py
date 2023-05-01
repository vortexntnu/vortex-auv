#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from pipeline_following import PipelineExecute, PipelineStandby
from task_manager_defines import defines
import dynamic_reconfigure.client


def task_manager_cb(config):
    rospy.loginfo(
        """Client: state change request: {Tac_states}""".format(**config))
    activated_task_id = config["Tac_states"]

    if defines.Tasks.valve_vertical.id == activated_task_id:
        isEnabled = True
    else:
        isEnabled = False
    print(f"isEnabled: {isEnabled} ")


def main():
    rospy.init_node("tac_pipeline_fsm")

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
    while not rospy.is_shutdown():
        # task manager
        isEnabled = False
        task_manager_client = dynamic_reconfigure.client.Client(
            "task_manager/task_manager_server",
            timeout=5,
            config_callback=task_manager_cb)
        while not rospy.is_shutdown() and isEnabled:
            rospy.loginfo('STARTING PIPELINE FOLLOWING')
            main()
            rospy.loginfo('PIPELINE FOLLOWING ENDED')
