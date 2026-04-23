#!/bin/bash
set -e
set -o pipefail

DRONE="${1:-nautilus}"
MODE_ARG="${2:-0}"

# Match manual behavior by default when a display exists; allow CI/headless override.
if [[ -z "${RENDERING:-}" ]]; then
    if [[ -n "${DISPLAY:-}" ]]; then
        RENDERING="true"
    else
        RENDERING="false"
    fi
fi

# Goal values can be overridden from env without editing the script.
ROLL_TARGET="${ROLL_TARGET:-1.57}"
PINCH_TARGET="${PINCH_TARGET:--0.10}"
CONVERGENCE_THRESHOLD="${CONVERGENCE_THRESHOLD:-0.05}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR=""

SIM_PID=""
GRIPPER_CONTROLLER_PID=""
GRIPPER_REF_FILTER_PID=""
GRIPPER_SIM_BRIDGE_PID=""
GUIDANCE_WATCH_PID=""
CONTROL_WATCH_PID=""

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

wait_for_topic() {
    local topic_name="$1"
    local timeout_s="$2"

    echo "Waiting for topic: $topic_name"
    timeout "${timeout_s}s" bash -c 'until ros2 topic list | grep -Fxq "$1"; do sleep 1; done' _ "$topic_name"
}

wait_for_service() {
    local service_name="$1"
    local timeout_s="$2"

    echo "Waiting for service: $service_name"
    timeout "${timeout_s}s" bash -c 'until ros2 service list | grep -Fxq "$1"; do sleep 1; done' _ "$service_name"
}

run_in_workspace_shell() {
    local cmd="$1"
    bash -lc "cd \"$WORKSPACE_DIR\" && source /opt/ros/humble/setup.sh && source \"$WORKSPACE_DIR/install/setup.bash\" && $cmd"
}

cleanup() {
    echo "Error detected. Cleaning up..."

    [[ -n "$GUIDANCE_WATCH_PID" ]] && kill -TERM "$GUIDANCE_WATCH_PID" 2>/dev/null || true
    [[ -n "$CONTROL_WATCH_PID" ]] && kill -TERM "$CONTROL_WATCH_PID" 2>/dev/null || true

    [[ -n "$GRIPPER_SIM_BRIDGE_PID" ]] && kill -TERM -"$GRIPPER_SIM_BRIDGE_PID" 2>/dev/null || true
    [[ -n "$GRIPPER_REF_FILTER_PID" ]] && kill -TERM -"$GRIPPER_REF_FILTER_PID" 2>/dev/null || true
    [[ -n "$GRIPPER_CONTROLLER_PID" ]] && kill -TERM -"$GRIPPER_CONTROLLER_PID" 2>/dev/null || true
    [[ -n "$SIM_PID" ]] && kill -TERM -"$SIM_PID" 2>/dev/null || true

    rm -f "$SCRIPT_DIR/gripper_goal.yaml" "$SCRIPT_DIR/gripper_result.yaml"

    exit 1
}
trap cleanup ERR

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

if [[ "$DRONE" != "nautilus" ]]; then
    echo "gripper_sim_interface currently bridges nautilus topics only."
    echo "Use DRONE=nautilus for this simulator test."
    exit 1
fi

echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
WORKSPACE_DIR="$(find_workspace_dir)" || {
    echo "Unable to locate workspace root containing install/setup.bash."
    echo "Set WORKSPACE to your ROS 2 workspace path and re-run."
    exit 1
}
. "$WORKSPACE_DIR/install/setup.bash"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}/usr/local/lib"

echo "Detected workspace: $WORKSPACE_DIR"
echo "Using goal config: drone=$DRONE mode=$MODE_ARG ($MODE_NAME), roll=$ROLL_TARGET, pinch=$PINCH_TARGET, convergence_threshold=$CONVERGENCE_THRESHOLD, rendering=$RENDERING"

# Match launch_drone.sh stack; rendering can be toggled with RENDERING=true/false.
echo "Launching simulator..."
setsid ros2 launch stonefish_sim vortex_sim_launch.py drone:=${DRONE} rendering:=${RENDERING} &
SIM_PID=$!
echo "Launched simulator with PID: $SIM_PID"

echo "Launching gripper controller..."
setsid ros2 launch gripper_controller gripper_controller.launch.py &
GRIPPER_CONTROLLER_PID=$!
echo "Launched gripper controller with PID: $GRIPPER_CONTROLLER_PID"

echo "Launching gripper reference filter..."
setsid ros2 launch gripper_reference_filter gripper_reference_filter.launch.py &
GRIPPER_REF_FILTER_PID=$!
echo "Launched gripper reference filter with PID: $GRIPPER_REF_FILTER_PID"

echo "Launching gripper sim interface..."
setsid ros2 launch gripper_sim_interface gripper_sim_interface.launch.py &
GRIPPER_SIM_BRIDGE_PID=$!
echo "Launched gripper sim interface with PID: $GRIPPER_SIM_BRIDGE_PID"

echo "Waiting for simulator to start..."
wait_for_topic "/${DRONE}/odom" 60
timeout 15s ros2 topic echo "/${DRONE}/odom" --once >/dev/null
echo "Simulator online"

echo "Waiting for gripper pipeline readiness..."
# Bridge input from stonefish sim
wait_for_topic "/${DRONE}/servo_state" 60
timeout 20s ros2 topic echo "/${DRONE}/servo_state" --once >/dev/null
# Bridge output from gripper_sim_interface
timeout 30s ros2 topic echo /vortex/gripper/state --once >/dev/null

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Manual workflow parity:
# - Wait ~5 seconds after sim appears online.
# - Emulate keyboard '2' then '3' through service calls.
echo "Sleeping 5 seconds for sim spool-up..."
sleep 5

echo "Emulating keyboard input: key '2' (killswitch false)"
wait_for_service "/${DRONE}/set_killswitch" 30
run_in_workspace_shell "timeout 20s ros2 service call /${DRONE}/set_killswitch vortex_msgs/srv/SetKillswitch '{killswitch_on: false}'" >/dev/null

echo "Emulating keyboard input: key '3' (autonomous mode)"
wait_for_service "/${DRONE}/set_operation_mode" 30
run_in_workspace_shell "timeout 20s ros2 service call /${DRONE}/set_operation_mode vortex_msgs/srv/SetOperationMode '{requested_operation_mode: {operation_mode: 1}}'" >/dev/null

# Let operation mode propagate before sending action goal.
sleep 2

echo "Waiting for guidance output from reference filter..."
timeout 40s ros2 topic echo /vortex/gripper/guidance --once --qos-reliability best_effort >/dev/null &
GUIDANCE_WATCH_PID=$!

echo "Waiting for controller output command..."
timeout 40s ros2 topic echo /vortex/gripper/control --once --qos-reliability best_effort >/dev/null &
CONTROL_WATCH_PID=$!

echo "Sending goal (terminal-like subshell)"
run_in_workspace_shell "python3 \"$SCRIPT_DIR/send_goal.py\" \
    --mode "$MODE_ARG" \
    --roll "$ROLL_TARGET" \
    --pinch "$PINCH_TARGET" \
    --convergence-threshold "$CONVERGENCE_THRESHOLD" \
    --output-dir \"$SCRIPT_DIR\""

wait "$GUIDANCE_WATCH_PID"
echo "Got guidance data"

wait "$CONTROL_WATCH_PID"
echo "Got controller output"

echo "Checking if goal reached"
if ! run_in_workspace_shell "python3 \"$SCRIPT_DIR/check_goal.py\" --output-dir \"$SCRIPT_DIR\""; then
    echo "Test failed: Gripper did not reach goal."
    exit 1
fi
echo "Test passed: Gripper reached goal."

echo "Terminating launched processes..."
kill -TERM -"$GRIPPER_SIM_BRIDGE_PID" -"$GRIPPER_REF_FILTER_PID" -"$GRIPPER_CONTROLLER_PID" -"$SIM_PID" || true

rm -f "$SCRIPT_DIR/gripper_goal.yaml" "$SCRIPT_DIR/gripper_result.yaml"

echo "Gripper simulator test completed successfully."
