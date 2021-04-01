#!/usr/bin/env python
import rospy
import smach
import smach_ros
import sys

from si_sm_classes import Idle, ReachStartPosition


def main(world_name):
    rospy.init_node("si_state_machine_node")

    sm = smach.StateMachine(outcomes=["Done"])

    sis = smach_ros.IntrospectionServer("si_state_machine", sm, "/SM_ROOT")
    sis.start()

    with sm:
        smach.StateMachine.add(
            "Idle",
            Idle(),
            transitions={
                "doing": "ReachStartPosition",
                "waiting": "Idle",
            },
        )
        smach.StateMachine.add(
            "ReachStartPosition",
            ReachStartPosition(),
            transitions={
                "continue": "Todo",
                "waiting": "ReachStartPosition",
            },
        )

    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()