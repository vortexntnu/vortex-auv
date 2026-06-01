#!/bin/bash
# Launch drone simulation stack in a tmux session
# Usage: ./launch_drone_sim.sh [--scenario <name>] [--domain-id <id>]
#   --scenario  Stonefish scenario to load (default: default)
#               GPU scenarios:    default, docking, pipeline, structure,
#                                 orca_demo, freya_demo, orca_freya_demo, tacc
#               No-GPU scenarios: nautilus_no_gpu, orca_no_gpu, freya_no_gpu
#   --domain-id ROS_DOMAIN_ID to use (default: 0)

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --scenario <name>    Stonefish scenario to load (default: default)
                         GPU:    default, docking, pipeline, structure,
                                 orca_demo, freya_demo, orca_freya_demo, tacc
                         No-GPU: nautilus_no_gpu, orca_no_gpu, freya_no_gpu
  --domain-id <id>     ROS_DOMAIN_ID to use (default: 0)
  --keyboard-joy <bool> Enable keyboard joystick control (default: true)
  -h, --help           Show this help message
EOF
}

SCENARIO="default"
DOMAIN_ID="0"
KEYBOARD_JOY="true"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario)     SCENARIO="$2";     shift 2 ;;
        --domain-id)    DOMAIN_ID="$2";    shift 2 ;;
        --keyboard-joy) KEYBOARD_JOY="$2"; shift 2 ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

SESSION="drone_launch"
S="source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# Launch Foxglove Studio only if not already running
if ! pgrep -f foxglove-studio &>/dev/null; then
  foxglove-studio &>/dev/null &
fi

# =============================================
# Window 1: sim (4 equal panes)
# =============================================
tmux new-session -d -s "$SESSION" -n "sim"

PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')
tmux send-keys -t "$PANE_SIM" "clear && $S && ros2 launch stonefish_sim vortex_sim_launch.py keyboard_joy:=$KEYBOARD_JOY scenario:=$SCENARIO" Enter

PANE_P2=$(tmux split-window -h -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P2" "clear && $S && ros2 launch auv_setup dp_quat.launch.py" Enter

PANE_P3=$(tmux split-window -v -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P3" "clear && $S" Enter

PANE_P4=$(tmux split-window -v -t "$PANE_P2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P4" "clear && $S" Enter

tmux select-layout -t "$SESSION:sim" tiled

# =============================================
# Window 2: tools (2 panes)
# =============================================
tmux new-window -t "$SESSION" -n "tools"

PANE_FOX=$(tmux list-panes -t "$SESSION:tools" -F '#{pane_id}')
tmux send-keys -t "$PANE_FOX" "clear && $S && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

PANE_MSG=$(tmux split-window -v -t "$PANE_FOX" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_MSG" "clear && $S && ros2 launch vortex_utility_nodes rpy_publisher.launch.py " Enter

# =============================================
# Focus first window and attach
# =============================================
tmux select-window -t "$SESSION:sim"
tmux attach-session -t "$SESSION"
