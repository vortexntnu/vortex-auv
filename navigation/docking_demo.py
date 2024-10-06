import rclpy
from action_tutorials_interfaces.action import Fibonacci
from yasmin import CbState
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

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
    
def print_result(blackboard: Blackboard) -> str:
    print(f"Result: {blackboard.dock_res}")
    return SUCCEED

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

def main():
    print("yasmin_action_client_demo (docking)")

    rclpy.init()

    sm = StateMachine(outcomes=["outcome4"])

    sm.add_state("calling_the_docker",
                 DockingState(),
                 transitions={
                     SUCCEED: "go_to_dock",
                     CANCEL: "outcome4",
                     ABORT: "outcome4"
                 })
    sm.add_state(
                "Printing_the_results",
                CbState([SUCCEED], print_result),
                transitions={
                    SUCCEED: "outcome4"
                })

    sm.add_state(
                "go_to_dock", CbState([SUCCEED], go_to_dock),
                transitions={
                    SUCCEED: "dock"
                }
    )
    sm.add_state(
        "dock", CbState([SUCCEED], dock), transitions={
            SUCCEED: "Printing_the_results"
        }
    )
    sm.add_state(
        "Idle", CbState([SUCCEED], idle), transitions={
            SUCCEED: "find_dock"
        }
    )
    sm.add_state(
        "find_dock", CbState([SUCCEED], search_for_dock), transitions={
            SUCCEED: "go_to_dock"
        }
    )


    YasminViewerPub("Stateså", sm)

    blackboard = Blackboard()
    blackboard.n = 10

    outcome = sm(blackboard)
    print(outcome)

    rclpy.shutdown()

if __name__ == "__main__":
    main()