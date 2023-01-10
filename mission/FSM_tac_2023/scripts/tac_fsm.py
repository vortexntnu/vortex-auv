#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from tasks.manual_mode import ManualMode
def main():
    rospy.init_node("tac_fsm.py")

    # rospy.wait_for_message()
    # rospy.wait_for_service()

    #Create a SMACH state machine
    tac_state_machine = StateMachine(outcomes=["preempted", "succeeded", "aborted"])

    #Open the container
    with tac_state_machine:
        ##PREPARATION
        #StateMAchine.add(...)

        ##Manual_Mode
        manual_mode = StateMachine(outcomes=["GripperHorisontal", "GripperVertical"])
        with manual_mode:
            StateMachine.add(
                "ManualMode",
                ManualMode(),
                transitions = {"gripper_horisontal": "GripperHorisontal"},
                transitions = {"gripper_vertical": "GripperVertical"}
                )
            

        # ##Gripper_Horisontal
        # gripper_horisontal = StateMachine(outcomes=["preempted", "succeeded", "aborted"])
        # with manual_mode:
        #     StateMachine.add(
        #         "GripperHorisontal",
        #         GripperHorisontal(),
        #         transitions = {"preempted"},
        #         transitions = {"succeeded"},
        #         transitions = {"aborted"},
        #         )

        # ##Gripper_Vertical
        # gripper_vertical = StateMachine(outcomes=["gripper_horisontal", "gripper_vertical"])
        # with manual_mode:
        #     StateMachine.add(
        #         "GripperVertical",
        #         GripperVertical(),
        #         transitions = {"preempted"},
        #         transitions = {"succeeded"},
        #         transitions = {"aborted"},
        #         )

        ##State Machine ...

        ##State Machine ...
        
        pass


    intro_server = IntrospectionServer(
        str(rospy.get_name()), tac_state_machine, "/SM_ROOT"
    )
    intro_server.start()

    try:
        #Execute SMACH plan
        tac_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("State machine failed: %s" % e)


if __name__ == "__main__":
    main()

