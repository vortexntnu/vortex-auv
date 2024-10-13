import time

import rclpy

# from action_tutorials_interfaces.action import Fibonacci
import rclpy.publisher
from yasmin import Blackboard, CbState, StateMachine

# from waypoint_action.action import Waypoint
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_viewer import YasminViewerPub

"""
class GoToWaypointState(ActionState):
    def __init__(self) -> None:
        super.__init__(Waypoint, "/waypoint", self._create_goal_handler, None, self.response_handler, self.print_feedback)

    def create_goal_handler(self, blackboard: Blackboard) ->Waypoint.Goal:
        goal = Waypoint.Goal()
        goal.order = blackboard["waypoint"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: Waypoint.Result) -> str:
        blackboard["waypoint_res"] = response.sequence
        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: Waypoint.Feedback) -> None:
        print(f"Received feedback: {list(feedback.partial_sequence)}")
"""


def go_to_dock(blackboard: Blackboard) -> str:
    """
    State for going to dock. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Moving to dock ...")
    time.sleep(0.5)
    return SUCCEED


def dock(blackboard: Blackboard) -> str:
    """
    State for docking. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Docking...")
    time.sleep(0.5)
    return SUCCEED


def idle(blackboard: Blackboard) -> str:
    """
    State for being idle. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    blackboard.distance = 10
    print("Idle...")
    time.sleep(0.5)
    return SUCCEED


def search_for_dock(blackboard: Blackboard) -> str:
    """
    State for searching for the dock. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Searching_for_dock...")
    time.sleep(0.5)
    if blackboard.distance > 8:
        print("Dock not found. Searching...")
        blackboard.distance -= 1
        return ABORT
    else:
        return SUCCEED


def set_up(blackboard: Blackboard) -> str:
    """
    State for setting up the auv. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    time.sleep(0.5)
    print("Setting up")
    return SUCCEED


def return_home(blackboard: Blackboard) -> str:
    """
    State for returning home. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    print("Returning home")
    time.sleep(0.5)
    return SUCCEED


def error(blackboard: Blackboard) -> str:
    """
    State for error. Input: blackboard, returns: (str) the result of the node. ABORT
    """
    print("Error occured")

    time.sleep(0.5)
    return ABORT


def abort_mission(blackboard: Blackboard) -> str:
    """
    State for aborting. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    time.sleep(0.5)
    print("Aborting mission")
    return ABORT


def docked(blackboard: Blackboard) -> str:
    """
    State for being docked. Input: blackboard, returns: (str) the result of the node. SUCCESS
    """
    time.sleep(0.5)
    print("Docked")
    if blackboard.distance <= 3:
        return ABORT
    return SUCCEED


def main() -> None:
    """
    main function of the state machine.
    """
    print("yasmin_docking_fsm_demo")

    rclpy.init()

    sm = StateMachine(outcomes=["error", "finished", "aborted"])

    sm.add_state("go_to_dock", CbState([SUCCEED], go_to_dock), transitions={SUCCEED: "set_up"})
    sm.add_state("dock", CbState([SUCCEED], dock), transitions={SUCCEED: "docked"})
    sm.add_state("Idle", CbState([SUCCEED], idle), transitions={SUCCEED: "find_dock"})
    sm.add_state("find_dock", CbState([SUCCEED, ABORT], search_for_dock), transitions={SUCCEED: "go_to_dock", ABORT: "find_dock"})
    sm.add_state("set_up", CbState([SUCCEED], set_up), transitions={SUCCEED: "dock"})
    sm.add_state("return_home", CbState([SUCCEED], return_home), transitions={SUCCEED: "finished"})
    sm.add_state("Error_state", CbState([ABORT], error), transitions={ABORT: "error"})
    sm.add_state("Abort_mission", CbState([SUCCEED, ABORT], abort_mission), transitions={SUCCEED: "return_home", ABORT: "aborted"})
    sm.add_state("docked", CbState([SUCCEED, ABORT], docked), transitions={ABORT: "Abort_mission", SUCCEED: "Idle"})

    YasminViewerPub("Docking State Machine", sm)

    blackboard = Blackboard()
    blackboard.distance = 10

    outcome = sm(blackboard)
    print("outcome: ", outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
