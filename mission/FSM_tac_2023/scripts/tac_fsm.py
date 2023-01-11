#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from tasks.manual_mode import ManualMode
from tasks.valve_horisontal import ValveSearch, ValveConverge, ValveExecute

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
        manual_mode = StateMachine(outcomes=["valve_h", "valve_v"])
        with manual_mode:
            StateMachine.add(
                "MANUAL_MODE",
                ManualMode(),
                transitions = {"valve_h": "VALVE_H_SM","valve_v": "VALVE_V_SM"}
            )
            

        ##Valve_Manipulation_Horisontal
        valve_h_sm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])
        with valve_h_sm:
            StateMachine.add(
                "VALVE_SEARCH",
                ValveSearch(),
                transitions = {"succeeded": "VALVE_CONVERGE", "aborted": "MANUAL_MODE"}
            )
            
            StateMachine.add(
                "VALVE_CONVERGE",
                ValveConverge(),
                transitions = {"succeeded": "VALVE_EXECUTE", "aborted": "VALVE_SEARCH"}
            )

            StateMachine.add(
                "VALVE_EXECUTE",
                ValveExecute(),
                transitions = {"succeeded": "MANUAL_MODE" , "aborted": "VALVE_CONVERGE"}
            )


        StateMachine.add("VALVE_H_SM", valve_h_sm, transitions={"succeded": "MANUAL_MODE"})

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

