#!/usr/bin/env python3
import argparse
import os
import sys

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from vortex_msgs.action import GripperReferenceFilterWaypoint


def goal_status_to_text(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(status, f"UNRECOGNIZED({status})")


def write_yaml(path: str, payload: dict) -> None:
    with open(path, "w", encoding="utf-8") as file_handle:
        yaml.safe_dump(payload, file_handle, sort_keys=False)


class GripperGoalClient(Node):
    def __init__(self, action_name: str):
        super().__init__("gripper_reference_filter_waypoint_client")
        self.action_client = ActionClient(
            self,
            GripperReferenceFilterWaypoint,
            action_name,
        )
        self.latest_feedback = None

    def feedback_callback(self, feedback_msg):
        reference = feedback_msg.feedback.reference
        self.latest_feedback = {
            "roll": float(reference.roll),
            "pinch": float(reference.pinch),
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send gripper reference filter goal and persist result for check_goal.py"
    )
    parser.add_argument("--mode", type=int, default=0, help="0=ROLL_AND_PINCH, 1=ONLY_ROLL, 2=ONLY_PINCH")
    parser.add_argument("--roll", type=float, default=1.57)
    parser.add_argument("--pinch", type=float, default=-0.10)
    parser.add_argument("--convergence-threshold", type=float, default=0.05)
    parser.add_argument("--action-name", default="/vortex/gripper/reference_filter")
    parser.add_argument("--output-dir", default=os.path.dirname(os.path.abspath(__file__)))
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.mode not in (0, 1, 2):
        print("Invalid mode. Valid values: 0 (ROLL_AND_PINCH), 1 (ONLY_ROLL), 2 (ONLY_PINCH)")
        return 1

    if not (-0.333 <= args.pinch <= 0.0):
        print("Invalid pinch target. Expected range is [-0.333, 0.0].")
        return 1

    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    goal_file = os.path.join(output_dir, "gripper_goal.yaml")
    result_file = os.path.join(output_dir, "gripper_result.yaml")

    goal_payload = {
        "action_name": args.action_name,
        "mode": int(args.mode),
        "roll": float(args.roll),
        "pinch": float(args.pinch),
        "convergence_threshold": float(args.convergence_threshold),
    }
    write_yaml(goal_file, goal_payload)

    result_payload = {
        "action_success": False,
        "goal_status": int(GoalStatus.STATUS_UNKNOWN),
        "goal_status_text": goal_status_to_text(int(GoalStatus.STATUS_UNKNOWN)),
        "last_feedback": None,
    }

    rclpy.init()
    node = GripperGoalClient(args.action_name)

    try:
        if not node.action_client.wait_for_server(timeout_sec=20.0):
            print(f"Timed out waiting for action server: {args.action_name}")
            write_yaml(result_file, result_payload)
            return 1

        goal_msg = GripperReferenceFilterWaypoint.Goal()
        goal_msg.waypoint.roll.roll = float(args.roll)
        goal_msg.waypoint.pinch.pinch = float(args.pinch)
        goal_msg.waypoint.mode = int(args.mode)
        goal_msg.convergence_threshold = float(args.convergence_threshold)

        print(
            f"Sending goal: mode={args.mode}, roll={args.roll}, pinch={args.pinch}, "
            f"convergence_threshold={args.convergence_threshold}"
        )

        send_goal_future = node.action_client.send_goal_async(
            goal_msg,
            feedback_callback=node.feedback_callback,
        )

        while rclpy.ok() and not send_goal_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Goal rejected")
            write_yaml(result_file, result_payload)
            return 1

        print("Goal accepted")
        get_result_future = goal_handle.get_result_async()

        while rclpy.ok() and not get_result_future.done():
            rclpy.spin_once(node, timeout_sec=0.1)

        wrapped_result = get_result_future.result()
        if wrapped_result is None:
            print("Failed to retrieve action result")
            write_yaml(result_file, result_payload)
            return 1

        goal_status = int(wrapped_result.status)
        action_success = bool(wrapped_result.result.success)

        result_payload = {
            "action_success": action_success,
            "goal_status": goal_status,
            "goal_status_text": goal_status_to_text(goal_status),
            "last_feedback": node.latest_feedback,
        }
        write_yaml(result_file, result_payload)

        print(
            f"Action finished with status={result_payload['goal_status_text']} "
            f"and success={action_success}"
        )

        if action_success and goal_status == GoalStatus.STATUS_SUCCEEDED:
            return 0

        return 1

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
