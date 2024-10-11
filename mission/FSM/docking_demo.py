import rclpy

# from action_tutorials_interfaces.action import Fibonacci
from yasmin import Blackboard, CbState, StateMachine

# from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_viewer import YasminViewerPub


def go_to_dock(blackboard: Blackboard) -> str:
    """
    State for going to dock. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Moving to dock ...")
    return SUCCEED


def dock(blackboard: Blackboard) -> str:
    """
    State for docking. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Docking...")
    return SUCCEED


def idle(blackboard: Blackboard) -> str:
    """
    State for being idle. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Idle...")
    return SUCCEED


def search_for_dock(blackboard: Blackboard) -> str:
    """
    State for searching for the dock. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Searching_for_dock...")
    return SUCCEED


def set_up(blackboard: Blackboard) -> str:
    """
    State for setting up the auv. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Setting up")
    return SUCCEED


def return_home(blackboard: Blackboard) -> str:
    """
    State for returning home. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Returning home")
    return SUCCEED


def error(blackboard: Blackboard) -> str:
    """
    State for error. Input: blackboard, returns: (str) the result of the node. ABORT
    """
    print("Error occured")
    return ABORT


def abort_mission(blackboard: Blackboard) -> str:
    """
    State for aborting. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Aborting mission")
    return SUCCEED


def docked(blackboard: Blackboard) -> str:
    """
    State for being docked. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Docked")
    return SUCCEED


def main() -> None:
    """
    main function of the state machine.
    """
    print("yasmin_docking_fsm_demo")

    rclpy.init()

    sm = StateMachine(outcomes=["outcome4"])

    sm.add_state("go_to_dock", CbState([SUCCEED], go_to_dock), transitions={SUCCEED: "set_up"})
    sm.add_state("dock", CbState([SUCCEED], dock), transitions={SUCCEED: "docked"})
    sm.add_state("Idle", CbState([SUCCEED], idle), transitions={SUCCEED: "find_dock"})
    sm.add_state("find_dock", CbState([SUCCEED], search_for_dock), transitions={SUCCEED: "go_to_dock"})
    sm.add_state("set_up", CbState([SUCCEED], set_up), transitions={SUCCEED: "dock"})
    sm.add_state("return_home", CbState([SUCCEED], return_home), transitions={SUCCEED: "Idle"})
    sm.add_state("Error_state", CbState([ABORT], error))
    sm.add_state("Abort_mission", CbState([SUCCEED], abort_mission), transitions={SUCCEED: "return_home", ABORT: "outcome4"})
    sm.add_state("docked", CbState([SUCCEED], docked), transitions={ABORT: "return_home", SUCCEED: "Idle"})

    YasminViewerPub("Docking State Machine", sm)

    blackboard = Blackboard()
    blackboard.n = 10

    outcome = sm(blackboard)
    print(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
