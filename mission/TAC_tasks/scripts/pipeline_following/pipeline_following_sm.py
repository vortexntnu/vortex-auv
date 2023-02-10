#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from pipeline_following import PipelineConverge, PipelineExecute

def main():
    rospy.init_node("tac_fsm")
    # rospy.wait_for_message()
    # rospy.wait_for_service()

    pipeline_following_sm = StateMachine(outcomes=["succeeded", "aborted"])
    with pipeline_following_sm:
        
        StateMachine.add(
            "PIPELINE_CONVERGE",
            PipelineConverge(),
            transitions = {"succeeded": "PIPELINE_EXECUTE"}
        )

        StateMachine.add(
            "PIPELINE_EXECUTE",
            PipelineExecute(),
            transitions = {"aborted": "PIPELINE_CONVERGE"}
        )

    #intro_server = IntrospectionServer(
    #    str(rospy.get_name()), pipeline_following_sm, "/SM_ROOT"
    #)
    #intro_server.start()

    try:
        #Execute SMACH plan
        pipeline_following_sm.execute()
        rospy.loginfo("hello world %s")
        #intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    main()