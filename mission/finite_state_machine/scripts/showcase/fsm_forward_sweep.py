#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from reach_depth import ReachDepth
from search.forward_sweep import ForwardSweepSearch


def main():
    rospy.init_node("nortek_fsm_forward_sweep")

    rospy.sleep(rospy.get_param("/fsm/time_to_launch"))

    #rospy.wait_for_service(
    #    "send_positions"
    #) 

    nortek_fsm = StateMachine(
        outcomes=["preempted", "succeeded", "aborted"])


    with nortek_fsm:

        StateMachine.add(
            "REACH_DEPTH", ReachDepth(), transitions={"succeeded": "SEARCH"}
        )

        StateMachine.add("SEARCH", ForwardSweepSearch())


    intro_server = IntrospectionServer(
        str(rospy.get_name()), nortek_fsm, "/SM_ROOT"
    )
    intro_server.start()

    try:
        nortek_fsm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Prequalification test failed: %s" % e)




if __name__ == "__main__":
    main()












