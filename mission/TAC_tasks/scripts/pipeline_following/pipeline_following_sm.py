#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from pipeline_following import PipelineConverge, PipelineExecute

def main():
    rospy.init_node("tac_fsm.py")

    # rospy.wait_for_message()
    # rospy.wait_for_service()

    pipeline_following_sm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])
    with pipeline_following_sm:
        
        StateMachine.add(
            "PIPELINE_CONVERGE",
            PipelineConverge(),
            transitions = {"succeeded": "PIPELINE_EXECUTE", "aborted": "MANUAL_MODE"}
        )

        StateMachine.add(
            "PIPELINE_EXECUTE",
            PipelineExecute(),
            transitions = {"succeeded": "MANUAL_MODE" , "aborted": "PIPELINE_CONVERGE"}
        )


    StateMachine.add("VALVE_H_SM", pipeline_following_sm, transitions={"succeded": "MANUAL_MODE"})


    intro_server = IntrospectionServer(
        str(rospy.get_name()), pipeline_following_sm, "/SM_ROOT"
    )
    intro_server.start()

    try:
        #Execute SMACH plan
        pipeline_following_sm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    main()