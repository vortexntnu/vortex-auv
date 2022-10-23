#!/usr/bin/python3

from mission.finite_state_machine.scripts.showcases.circle import Circle
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from mission.finite_state_machine.scripts.tasks.reach_depth import ReachDepth
#from mission.finite_state_machine.scripts.tasks.pole import PoleExecute
from . import Circle


def main():
    rospy.init_node("nortek_fsm")

    rospy.sleep(rospy.get_param("/fsm/time_to_launch"))

    #rospy.wait_for_service(
    #    "send_positions"
    #) 

    nortek_fsm = StateMachine(
        outcomes=["preempted", "succeded", "aborted"])


    with nortek_fsm:

        StateMachine.add(
            "REACH_DEPTH", ReachDepth(), transitions={"succeeded": "CIRCLE"}
        )

        StateMachine.add("CIRCLE", Circle())


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












