#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

def main():
    rospy.init_node("tac_fsm.py")

    # rospy.wait_for_message()
    # rospy.wait_for_service()

    tac_state_machine = StateMachine(outcomes=["preempted", "succeeded", "aborted"])

    with tac_state_machine:
        ##PREPARATION
        #StateMAchine.add(...)

        ##Manual_Mode

        ##Gripper_Horisontal

        ##Gripper_Vertical

        ##State Machine ...

        ##State Machine ...
        
        pass


    intro_server = IntrospectionServer(
        str(rospy.get_name()), tac_state_machine, "/SM_ROOT"
    )
    intro_server.start()

    try:
        tac_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    main()

