#!/usr/bin/python3

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer

from mission.finite_state_machine.scripts.tasks.gate import (
    GateSearch,
    GateConverge,
    GateExecute,
)
from mission.finite_state_machine.scripts.tasks.pole import (
    PoleSearch,
    PoleConverge,
    PoleExecute,
)
from mission.finite_state_machine.scripts.tasks.path import (
    PathSearch,
    PathConverge,
    PathExecute,
)
from mission.finite_state_machine.scripts.tasks.reach_depth import ReachDepth


def main():
    rospy.init_node("prequalification_fsm")

    rospy.sleep(rospy.get_param("/fsm/time_to_launch"))

    rospy.wait_for_service(
        "send_positions")  # consider moving into individual state functions

    prequalification_state_machine = StateMachine(
        outcomes=["preempted", "succeeded", "aborted"])

    with prequalification_state_machine:
        StateMachine.add("PREQUAL_PREPARE",
                         ReachDepth(),
                         transitions={"succeeded": "GATE_SM"})

        gate_sm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])

        with gate_sm:
            StateMachine.add("GATE_SEARCH",
                             GateSearch(),
                             transitions={"succeeded": "GATE_CONVERGE"})

            StateMachine.add(
                "GATE_CONVERGE",
                GateConverge(),
                transitions={
                    "succeeded": "GATE_EXECUTE",
                    "aborted": "GATE_SEARCH"
                },
                remapping={"gate_converge_output": "gate"},
            )

            StateMachine.add("GATE_EXECUTE", GateExecute())

        StateMachine.add("GATE_SM",
                         gate_sm,
                         transitions={"succeeded": "POLE_SM"})

        pole_sm = StateMachine(outcomes=["preempted", "succeeded", "aborted"])

        with pole_sm:
            StateMachine.add("POLE_SEARCH",
                             PoleSearch(),
                             transitions={"succeeded": "POLE_CONVERGE"})

            StateMachine.add(
                "POLE_CONVERGE",
                PoleConverge(),
                transitions={
                    "succeeded": "POLE_EXECUTE",
                    "aborted": "POLE_SEARCH"
                },
                remapping={"pole_converge_output": "pole"},
            )

            StateMachine.add("POLE_EXECUTE", PoleExecute())

        StateMachine.add("POLE_SM",
                         pole_sm,
                         transitions={"succeeded": "GATE_SM_BACK"})

        gate_sm_back = StateMachine(
            outcomes=["preempted", "succeeded", "aborted"])

        with gate_sm_back:
            StateMachine.add("GATE_SEARCH",
                             GateSearch(),
                             transitions={"succeeded": "GATE_CONVERGE"})

            StateMachine.add(
                "GATE_CONVERGE",
                GateConverge(),
                transitions={
                    "succeeded": "GATE_EXECUTE",
                    "aborted": "GATE_SEARCH"
                },
                remapping={"gate_converge_output": "gate"},
            )

            StateMachine.add("GATE_EXECUTE", GateExecute())

        StateMachine.add("GATE_SM_BACK", gate_sm_back)

    intro_server = IntrospectionServer(str(rospy.get_name()),
                                       prequalification_state_machine,
                                       "/SM_ROOT")
    intro_server.start()

    try:
        prequalification_state_machine.execute()
        intro_server.stop()

    except Exception as e:
        rospy.loginfo("Prequalification test failed: %s" % e)


if __name__ == "__main__":
    main()
