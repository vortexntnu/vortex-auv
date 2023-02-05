#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from docking import DockingSearch, DockingConverge, DockingExecute 


def main():
    rospy.init_node("tac_docking_fsm")

    docking_sm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])
    with docking_sm:

        StateMachine.add(
            "DOCKING_SEARCH",
            DockingSearch(),
            transitions = {"succeeded": "DOCKING_CONVERGE", "aborted": "MANUAL_MODE"}
        )

        StateMachine.add(
            "DOCKING_CONVERGE",
            DockingConverge(),
            transitions = {"succeeded": "DOCKING_EXECUTE", "aborted": "DOCKING_SEARCH"}
        )

        StateMachine.add(
            "DOCKING_EXECUTE",
            DockingExecute(),
            transitions = {"succeeded": "MANUAL_MODE" , "aborted": "VALVE_CONVERGE"}
        )


    StateMachine.add("DOCKING_FSM", docking_sm, transitions={"succeded": "MANUAL_MODE"})


    intro_server = IntrospectionServer(
        str(rospy.get_name()), docking_sm, "/SM_ROOT"
    )
    intro_server.start()

    try:
        #Execute SMACH plan
        docking_sm.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        enabled = rospy.get_param("/tasks/docking")
        if enabled == True:
            main()
