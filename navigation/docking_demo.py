import rclpy
from action_tutorials_interfaces.action import Fibonacci
from yasmin import Blackboard, CbState, StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import ABORT, CANCEL, SUCCEED
from yasmin_viewer import YasminViewerPub

"""
class DockingState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            Fibonacci, # action type
            "/fibonacci", # action name
            self.create_goal_handler, #cb to create goal
            None, #outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler, #cb to handle response
            self.print_feedback #cb to print feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Fibonacci.Goal:
        goal = Fibonacci.Goal()
        goal.order = blackboard.n
        return goal

    def response_handler(
            self,
            blackboard: Blackboard,
            response: Fibonacci.Result
    ) -> str:
        blackboard.dock_res = response.sequence

        return SUCCEED

    def print_feedback(self, blackboard: Blackboard, feedback: Fibonacci.Feedback) -> None:
        print(f"Received feedback {list(feedback.partial_sequence)}")
"""


def go_to_dock(blackboard: Blackboard) -> str:
    print("Moving to dock ...")
    return SUCCEED


def dock(blackboard: Blackboard) -> str:
    print("Docking...")
    return SUCCEED


def idle(blackboard: Blackboard) -> str:
    print("Idle...")
    return SUCCEED


def search_for_dock(blackboard: Blackboard) -> str:
    print("Searching_for_dock...")
    return SUCCEED


def set_up(blackboard: Blackboard) -> str:
    print("Setting up")
    return SUCCEED


def return_home(blackboard: Blackboard) -> str:
    print("Returning home")
    return SUCCEED


def error(blackboard: Blackboard) -> str:
    print("Error occured")
    return ABORT


def abort_mission(blackboard: Blackboard) -> str:
    print("Aborting mission")
    return SUCCEED


def docked(blackboard: Blackboard) -> str:
    print("Docked")
    return SUCCEED


def main():
    print("yasmin_action_client_demo (docking)")

    rclpy.init()

    sm = StateMachine(outcomes=["outcome4"])

    sm.add_state("go_to_dock", CbState([SUCCEED], go_to_dock), transitions={SUCCEED: "set_up"})
    sm.add_state("dock", CbState([SUCCEED], dock), transitions={SUCCEED: "docked"})
    sm.add_state("Idle", CbState([SUCCEED], idle), transitions={SUCCEED: "find_dock"})
    sm.add_state("find_dock", CbState([SUCCEED], search_for_dock), transitions={SUCCEED: "go_to_dock"})
    sm.add_state("set_up", CbState([SUCCEED], set_up), transitions={SUCCEED: "dock"})
    sm.add_state("return_home", CbState([SUCCEED], return_home), transitions={SUCCEED: "Idle"})
    sm.add_state("Error_state", CbState([ABORT], error), transitions={ABORT: "Abort_mission", SUCCEED: "find_dock"})
    sm.add_state("Abort_mission", CbState([SUCCEED], abort_mission), transitions={SUCCEED: "return_home", ABORT: "outcome4"})
    sm.add_state("docked", CbState([SUCCEED], docked), transitions={ABORT: "return_home", SUCCEED: "Idle"})

    YasminViewerPub("Stateså", sm)

    blackboard = Blackboard()
    blackboard.n = 10

    outcome = sm(blackboard)
    print(outcome)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
