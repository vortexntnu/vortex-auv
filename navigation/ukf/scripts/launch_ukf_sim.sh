#!/bin/bash
# Launch the drone simulation stack with the UKF and its NEES monitor in a tmux session
# Usage: ./launch_ukf_sim.sh [--scenario NAME] [--gui] [--pattern square|figure8]
#                            [--calibrated] [--no_foxglove] [--no_pattern] [--side M] [--depth M]
#   --scenario, -s NAME  Stonefish scenario to load (default: nautilus_no_gpu, headless)
#                        GPU:    default, docking, pipeline, structure, orca_demo,
#                                freya_demo, orca_freya_demo, tacc, robosub
#                        No-GPU: nautilus_no_gpu, orca_no_gpu, freya_no_gpu
#                        GPU scenarios render at low quality.
#   --gui                shorthand for --scenario default
#   --pattern TYPE       square: four corner test, figure8: figure 8 (default: square)
#   --calibrated         use the filter params that exactly match the simulator noise
#                        (ukf_params_calibrated_sim.yaml), default is the tuned ukf_params.yaml
#   --no_foxglove        do not start Foxglove Studio and the Foxglove bridge
#   --no_pattern         do not fly the test pattern (--no_square is kept as an alias)
#   --side M             side of the square, or half width of the figure 8, in meters (default 5)
#   --depth M            depth of the pattern in meters (default 2)
#
# Unless --no_pattern is given, waypoint_sender.py switches the drone to
# autonomous mode, turns the killswitch off and flies the pattern forever.
#
# The drone is controlled with the quaternion stack: dp_quat.launch.py starts the
# quaternion reference filter together with the quaternion adaptive controller,
# and the joystick interface publishes quaternion references to match.

SESSION="ukf_launch"

# VXO_ROS_WS is normally already exported by vortex-onboarding's env.sh via
# ~/.bashrc; source it directly too so this still works when the script is run
# outside a login shell.
if [[ -z "${VXO_ROS_WS:-}" ]] && [[ -f "$HOME/.config/vortex-onboarding/env.sh" ]]; then
  # shellcheck disable=SC1091
  source "$HOME/.config/vortex-onboarding/env.sh"
fi
S="source \"${VXO_ROS_WS:-$HOME/code/ros2_ws}/install/setup.bash\""

# =============================================
# Parse arguments
# =============================================
FOXGLOVE=true
SQUARE=true
PATTERN="square"
CALIBRATED=false
SCENARIO="nautilus_no_gpu"
SIDE=5
DEPTH=2
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USAGE="Usage: $0 [--scenario NAME] [--gui] [--pattern square|figure8] [--calibrated] [--no_foxglove] [--no_pattern] [--side M] [--depth M]"

while [[ $# -gt 0 ]]; do
  case "$1" in
  --scenario | -s)
    SCENARIO="$2"
    shift 2
    ;;
  --gui)
    SCENARIO="default"
    shift
    ;;
  --pattern)
    PATTERN="$2"
    shift 2
    ;;
  --calibrated)
    CALIBRATED=true
    shift
    ;;
  --no_foxglove)
    FOXGLOVE=false
    shift
    ;;
  --no_pattern | --no_square)
    SQUARE=false
    shift
    ;;
  --side)
    SIDE="$2"
    shift 2
    ;;
  --depth)
    DEPTH="$2"
    shift 2
    ;;
  *)
    echo "Unknown argument: $1"
    echo "$USAGE"
    exit 1
    ;;
  esac
done

if [[ "$PATTERN" != "square" && "$PATTERN" != "figure8" ]]; then
  echo "Error: pattern must be 'square' or 'figure8' (got: $PATTERN)"
  exit 1
fi

if [[ ! -f "${VXO_ROS_WS:-$HOME/code/ros2_ws}/src/vortex-stonefish-sim/stonefish_sim/scenarios/$SCENARIO.scn" ]]; then
  echo "Error: unknown scenario '$SCENARIO', see stonefish_sim/scenarios/"
  exit 1
fi

# The no-GPU scenarios cannot render
SIM_ARGS="rendering_quality:=low"
if [[ "$SCENARIO" == *_no_gpu ]]; then
  SIM_ARGS="rendering:=false"
fi

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# Launch Foxglove Studio only if not already running
if [[ "$FOXGLOVE" == true ]] && ! pgrep -f foxglove-studio &>/dev/null; then
  foxglove-studio &>/dev/null &
fi

# =============================================
# Window 1: sim (4 equal panes)
# The drone sim launch also brings up the drone description (sensor TFs)
# =============================================
tmux new-session -d -s "$SESSION" -n "sim"

# Grab the initial pane ID
PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')

# Stonefish simulation (top-left)
tmux send-keys -t "$PANE_SIM" "clear && $S && ros2 launch stonefish_sim vortex_sim_launch.py scenario:=$SCENARIO drone:=nautilus $SIM_ARGS" Enter

# Split right -> Keyboard joy (top-right)
PANE_JOY=$(tmux split-window -h -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_JOY" "clear && $S && ros2 launch keyboard_joy keyboard_joy_node.launch.py" Enter

# Split sim pane down -> Quaternion reference filter + controller (bottom-left)
PANE_DP=$(tmux split-window -v -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_DP" "clear && $S && ros2 launch auv_setup dp_quat.launch.py" Enter

# Split joy pane down -> Drone sim (bottom-right)
PANE_DRONE=$(tmux split-window -v -t "$PANE_JOY" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_DRONE" "clear && $S && ros2 launch stonefish_sim drone_sim.launch.py orientation_mode:=quat" Enter

# Force equal pane sizes
tmux select-layout -t "$SESSION:sim" tiled

# =============================================
# Window 2: ukf (UKF, NEES monitor, foxglove bridge)
# The UKF retries its TF lookups until the drone description is up
# =============================================
tmux new-window -t "$SESSION" -n "ukf"

PANE_UKF=$(tmux list-panes -t "$SESSION:ukf" -F '#{pane_id}')
tmux send-keys -t "$PANE_UKF" "clear && $S && ros2 launch ukf ukf.launch.py drone:=nautilus use_sim:=true calibrated:=$CALIBRATED" Enter

# Split right -> NEES monitor
PANE_NEES=$(tmux split-window -h -t "$PANE_UKF" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_NEES" "clear && $S && ros2 launch ukf nees.launch.py drone:=nautilus" Enter

if [[ "$FOXGLOVE" == true ]]; then
  # Split UKF pane down -> Foxglove bridge
  PANE_FOX=$(tmux split-window -v -t "$PANE_UKF" -P -F '#{pane_id}')
  tmux send-keys -t "$PANE_FOX" "clear && $S && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter
fi

# =============================================
# Window 3: square test (waits for the controller, then flies the square)
# =============================================
if [[ "$SQUARE" == true ]]; then
  tmux new-window -t "$SESSION" -n "square"
  PANE_SQUARE=$(tmux list-panes -t "$SESSION:square" -F '#{pane_id}')
  tmux send-keys -t "$PANE_SQUARE" "clear && $S && python3 \"$SCRIPT_DIR/waypoint_sender.py\" --$PATTERN $SIDE --depth $DEPTH" Enter
fi

# =============================================
# Focus first window and attach
# =============================================
tmux select-window -t "$SESSION:sim"
tmux attach-session -t "$SESSION"
