#!/bin/bash
set -e
set -o pipefail

# Goal mode argument (GripperWaypoint mode constraints)
#   0 = ROLL_AND_PINCH
#   1 = ONLY_ROLL
#   2 = ONLY_PINCH
MODE_ARG=${1:-0}

# Goal values can be overridden from env without editing the file.
ROLL_TARGET=${ROLL_TARGET:-1.57}
PINCH_TARGET=${PINCH_TARGET:--0.10}
CONVERGENCE_THRESHOLD=${CONVERGENCE_THRESHOLD:-0.05}

ACTION_NAME="/vortex/gripper/reference_filter"
ACTION_TYPE="vortex_msgs/action/GripperReferenceFilterWaypoint"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR=""

MODE_NAME=""
case "$MODE_ARG" in
	0) MODE_NAME="ROLL_AND_PINCH" ;;
	1) MODE_NAME="ONLY_ROLL" ;;
	2) MODE_NAME="ONLY_PINCH" ;;
	*)
		echo "Invalid mode '$MODE_ARG'. Valid values: 0 (ROLL_AND_PINCH), 1 (ONLY_ROLL), 2 (ONLY_PINCH)"
		exit 1
		;;
esac

# Valid pinch range is [-0.333, 0.0].
if ! awk "BEGIN {exit !($PINCH_TARGET >= -0.333 && $PINCH_TARGET <= 0.0)}"; then
	echo "Invalid pinch target '$PINCH_TARGET'. Expected range is [-0.333, 0.0]."
	exit 1
fi

GOAL_PAYLOAD="{waypoint: {roll: {roll: $ROLL_TARGET}, pinch: {pinch: $PINCH_TARGET}, mode: $MODE_ARG}, convergence_threshold: $CONVERGENCE_THRESHOLD}"

FILTER_PID=""
ACTION_LOG_FILE=""
ACTION_PID=""

find_workspace_dir() {
	local start_dir=""
	local dir=""
	local -a start_points=()

	[[ -n "${WORKSPACE:-}" ]] && start_points+=("$WORKSPACE")
	start_points+=("$PWD" "$SCRIPT_DIR")

	for start_dir in "${start_points[@]}"; do
		dir="$start_dir"
		while true; do
			if [[ -f "$dir/install/setup.bash" ]]; then
				echo "$dir"
				return 0
			fi
			[[ "$dir" == "/" ]] && break
			dir="$(dirname "$dir")"
		done
	done

	return 1
}

cleanup() {
	echo "Error detected. Cleaning up..."
	[[ -n "$ACTION_PID" ]] && kill -TERM "$ACTION_PID" 2>/dev/null || true
	kill -TERM -"$FILTER_PID" || true
	[[ -n "$ACTION_LOG_FILE" && -f "$ACTION_LOG_FILE" ]] && rm -f "$ACTION_LOG_FILE"
	exit 1
}
trap cleanup ERR

# Load ROS 2 environment
echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
WORKSPACE_DIR="$(find_workspace_dir)" || {
	echo "Unable to locate workspace root containing install/setup.bash."
	echo "Set WORKSPACE to your ROS 2 workspace path and re-run."
	exit 1
}
. "$WORKSPACE_DIR/install/setup.bash"
echo "Detected workspace: $WORKSPACE_DIR"

echo "Using goal config: mode=$MODE_ARG ($MODE_NAME), roll=$ROLL_TARGET, pinch=$PINCH_TARGET, convergence_threshold=$CONVERGENCE_THRESHOLD"

# Launch gripper reference filter node
echo "Launching gripper reference filter..."
setsid ros2 launch gripper_reference_filter gripper_reference_filter.launch.py &
FILTER_PID=$!
echo "Launched gripper reference filter with PID: $FILTER_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
	echo "Error detected in ROS logs. Exiting..."
	exit 1
fi

# Seed current gripper state so the filter has a valid initial state.
echo "Publishing initial gripper state..."
ros2 topic pub /vortex/gripper/state vortex_msgs/msg/GripperState "{roll: 0.0, pinch: -0.333}" --once >/dev/null

ACTION_LOG_FILE=$(mktemp)

echo "Sending gripper goal with feedback..."
ros2 action send_goal "$ACTION_NAME" "$ACTION_TYPE" "$GOAL_PAYLOAD" --feedback >"$ACTION_LOG_FILE" 2>&1 &
ACTION_PID=$!

# Check if node correctly publishes guidance
echo "Waiting for guidance data..."
timeout 15s ros2 topic echo /vortex/gripper/guidance --once --qos-reliability best_effort
echo "Got guidance data"

wait "$ACTION_PID"

echo "Action output:"
cat "$ACTION_LOG_FILE"

if ! grep -Eiq "Goal finished with status: SUCCEEDED|Goal succeeded" "$ACTION_LOG_FILE"; then
	echo "Goal did not report SUCCEEDED status."
	exit 1
fi

if ! grep -Eiq "success:[[:space:]]*(true|True)" "$ACTION_LOG_FILE"; then
	echo "Action result did not report success=true."
	exit 1
fi

# Terminate process
kill -TERM -"$FILTER_PID"
rm -f "$ACTION_LOG_FILE"

echo "Test completed successfully."
