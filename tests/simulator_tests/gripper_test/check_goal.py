#!/usr/bin/env python3
import argparse
import math
import os
import sys

import yaml
from action_msgs.msg import GoalStatus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate gripper action result and feedback convergence"
    )
    parser.add_argument(
        "--output-dir", default=os.path.dirname(os.path.abspath(__file__))
    )
    parser.add_argument("--margin", type=float, default=0.02)
    return parser.parse_args()


def read_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as file_handle:
        return yaml.safe_load(file_handle)


def main() -> int:
    args = parse_args()

    output_dir = os.path.abspath(args.output_dir)
    goal_file = os.path.join(output_dir, "gripper_goal.yaml")
    result_file = os.path.join(output_dir, "gripper_result.yaml")

    if not os.path.exists(goal_file):
        print(f"Missing goal file: {goal_file}")
        return 1

    if not os.path.exists(result_file):
        print(f"Missing result file: {result_file}")
        return 1

    goal = read_yaml(goal_file)
    result = read_yaml(result_file)

    action_success = bool(result.get("action_success", False))
    goal_status = int(result.get("goal_status", GoalStatus.STATUS_UNKNOWN))
    goal_status_text = result.get("goal_status_text", "UNKNOWN")

    if not action_success:
        print(f"Action result reported success=false (status={goal_status_text})")
        return 1

    if goal_status != GoalStatus.STATUS_SUCCEEDED:
        print(f"Action status is not SUCCEEDED: {goal_status_text}")
        return 1

    feedback = result.get("last_feedback")
    if not isinstance(feedback, dict):
        print(
            "No feedback captured from action execution; relying on action success/status only"
        )
        print("Goal check passed")
        return 0

    mode = int(goal["mode"])
    target_roll = float(goal["roll"])
    target_pinch = float(goal["pinch"])
    threshold = float(goal["convergence_threshold"])

    feedback_roll = float(feedback["roll"])
    feedback_pinch = float(feedback["pinch"])

    margin = float(args.margin)
    tolerance = threshold + margin

    if mode == 0:
        error = math.hypot(feedback_roll - target_roll, feedback_pinch - target_pinch)
        mode_name = "ROLL_AND_PINCH"
    elif mode == 1:
        error = abs(feedback_roll - target_roll)
        mode_name = "ONLY_ROLL"
    elif mode == 2:
        error = abs(feedback_pinch - target_pinch)
        mode_name = "ONLY_PINCH"
    else:
        print(f"Unsupported mode in goal file: {mode}")
        return 1

    print(
        "Goal check:\n"
        f"  mode={mode} ({mode_name})\n"
        f"  target_roll={target_roll:.6f}, target_pinch={target_pinch:.6f}\n"
        f"  feedback_roll={feedback_roll:.6f}, feedback_pinch={feedback_pinch:.6f}\n"
        f"  threshold={threshold:.6f}, margin={margin:.6f}, tolerance={tolerance:.6f}\n"
        f"  error={error:.6f}"
    )

    if error > tolerance:
        print("Goal check failed: feedback did not converge within tolerance")
        return 1

    print("Goal check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
