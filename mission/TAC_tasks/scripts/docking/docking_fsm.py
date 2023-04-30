#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from docking import DockingExecute, DockingStandby


def main():
    rospy.init_node("tac_docking_fsm")

    docking_sm = StateMachine(outcomes=['done'])
    with docking_sm:

        StateMachine.add("DOCKING_EXECUTE",
                         DockingExecute(),
                         transitions={
                             'succeeded': 'DOCKING_STANDBY',
                             'preempted': 'done'
                         })

        StateMachine.add("DOCKING_STANDBY",
                         DockingStandby(),
                         transitions={'succeeded': 'done'})

    try:
        docking_sm.execute()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        enabled = rospy.get_param("/tasks/docking")
        if enabled:
            main()
