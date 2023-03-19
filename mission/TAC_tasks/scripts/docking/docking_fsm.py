#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from docking import DockingSearch, DockingExecute, DockingStandby


def main():
    rospy.init_node("tac_docking_fsm")

    docking_sm = StateMachine(outcomes=['done'])
    with docking_sm:

        StateMachine.add(
            "DOCKING_SEARCH",
            DockingSearch(),
            transitions={'succeeded': 'DOCKING_EXECUTE', 'preempted': 'done'}
        )

        StateMachine.add(
            "DOCKING_EXECUTE",
            DockingExecute(),
            transitions = {'succeeded': 'DOCKING_STANDBY', 'aborted': 'DOCKING_SEARCH', 'preempted': 'done'} 
        )

        StateMachine.add(
            "DOCKING_STANDBY",
            DockingStandby(),
            transitions = {'succeeded': 'done'}
        )


    # intro_server = IntrospectionServer(
    #     str(rospy.get_name()), docking_sm, "/SM_ROOT"
    # )
    # intro_server.start()

    try:
        #Execute SMACH plan
        docking_sm.execute()
        # intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        # enabled = rospy.get_param("/tasks/docking")
        # if enabled == True:
        main()
