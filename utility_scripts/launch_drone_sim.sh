#!/bin/bash
# Launch drone simulation stack in a tmux session
# Usage: ./launch_drone_sim.sh [--scenario <name>] [--domain-id <id>]
#   --scenario  Stonefish scenario to load (default: default)
#               GPU scenarios:    default, docking, pipeline, structure,
#                                 orca_demo, freya_demo, orca_freya_demo, tacc
#               No-GPU scenarios: nautilus_no_gpu, orca_no_gpu, freya_no_gpu
#   --domain-id ROS_DOMAIN_ID to use (default: 0)
# Perception and mission (landmark_server, waypoint_manager, dummy perception)
# are started separately, e.g. vortex-cv perception_setup/scripts/tmux_robosub_sim.sh.

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
  --seed <n>           Seed for the RoboSub role images (default: 7). Use the
                       same seed for the dummy perception so they agree
  --headless           No rendering and no course geometry (scenario
                       nautilus_no_gpu, keyboard joystick off)
  --low-res            Render at 960x540, low quality (the RoboSub course at
                       full resolution can run out of memory)
  --mem-limit <GB>     Memory limit for the simulator (default: none)
  --detach             Start the session without attaching to it
  -h, --help           Show this help message
EOF
}

SCENARIO="default"
DOMAIN_ID="0"
KEYBOARD_JOY="true"
SEED="7"
HEADLESS="false"
LOW_RES="false"
MEM_LIMIT="0"
DETACH="false"
while [[ $# -gt 0 ]]; do
    case "$1" in
        --scenario)     SCENARIO="$2";     shift 2 ;;
        --domain-id)    DOMAIN_ID="$2";    shift 2 ;;
        --keyboard-joy) KEYBOARD_JOY="$2"; shift 2 ;;
        --seed)         SEED="$2";         shift 2 ;;
        --headless)     HEADLESS="true";   shift ;;
        --low-res)      LOW_RES="true";    shift ;;
        --mem-limit)    MEM_LIMIT="$2";    shift 2 ;;
        --detach)       DETACH="true";     shift ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "Unknown argument: $1"; usage; exit 1 ;;
    esac
done

if [[ "$HEADLESS" == "true" ]]; then
    SIM_ARGS="keyboard_joy:=false rendering:=false scenario:=nautilus_no_gpu"
else
    SIM_ARGS="keyboard_joy:=$KEYBOARD_JOY scenario:=$SCENARIO robosub_icon_seed:=$SEED"
    if [[ "$LOW_RES" == "true" ]]; then
        SIM_ARGS="$SIM_ARGS window_res_x:=960 window_res_y:=540 rendering_quality:=low"
    fi
fi
SIM_CMD="ros2 launch stonefish_sim vortex_sim_launch.py $SIM_ARGS"
if [[ "$MEM_LIMIT" != "0" ]] && command -v systemd-run &>/dev/null; then
    SIM_CMD="systemd-run --user --scope -p MemoryMax=${MEM_LIMIT}G -p MemorySwapMax=0 $SIM_CMD"
fi

SESSION="drone_launch"
S="source install/setup.bash && export ROS_DOMAIN_ID=$DOMAIN_ID"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# Launch Foxglove Studio only if not already running
if ! pgrep -f foxglove-studio &>/dev/null && command -v foxglove-studio &>/dev/null; then
  foxglove-studio &>/dev/null &
fi

# =============================================
# Window 1: sim (4 equal panes)
# =============================================
tmux new-session -d -s "$SESSION" -n "sim"

PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')
tmux send-keys -t "$PANE_SIM" "clear && $S && $SIM_CMD" Enter

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
if [[ "$DETACH" == "true" ]]; then
    echo "Session $SESSION started; attach with: tmux attach -t $SESSION"
elif [[ -n "$TMUX" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach-session -t "$SESSION"
fi
