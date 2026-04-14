#!/bin/bash
# Launch drone vehicle stack in a tmux session
# Usage: ./launch_drone_vehicle.sh [--ori_type quat|euler] [--controller_type pid|adapt|adapt_quat]

SESSION="drone_vehicle"

# =============================================
# Parse arguments
# =============================================
ORI_TYPE="quat"
CONTROLLER_TYPE="adapt_quat"

while [[ $# -gt 0 ]]; do
  case "$1" in
  --ori_type | -o)
    ORI_TYPE="$2"
    shift 2
    ;;
  --controller_type | -c)
    CONTROLLER_TYPE="$2"
    shift 2
    ;;
  *)
    echo "Unknown argument: $1"
    echo "Usage: $0 [--ori_type quat|euler] [--controller_type pid|adapt|adapt_quat]"
    exit 1
    ;;
  esac
done

# Validate
if [[ "$ORI_TYPE" != "quat" && "$ORI_TYPE" != "euler" ]]; then
  echo "Error: ori_type must be 'quat' or 'euler' (got: $ORI_TYPE)"
  exit 1
fi

if [[ "$CONTROLLER_TYPE" != "pid" && "$CONTROLLER_TYPE" != "adapt" && "$CONTROLLER_TYPE" != "adapt_quat" ]]; then
  echo "Error: controller_type must be 'pid', 'adapt' or 'adapt_quat' (got: $CONTROLLER_TYPE)"
  exit 1
fi

# Cross-validate: pid uses quat reference filter, adaptive uses euler
if [[ "$CONTROLLER_TYPE" == "pid" && "$ORI_TYPE" != "quat" ]]; then
  echo "Warning: pid controller uses quaternion representation — consider --ori_type quat"
fi
if [[ "$CONTROLLER_TYPE" == "adapt" && "$ORI_TYPE" != "euler" ]]; then
  echo "Warning: adaptive controller uses euler representation — consider --ori_type euler"
fi
if [[ "$CONTROLLER_TYPE" == "adapt_quat" && "$ORI_TYPE" != "quat" ]]; then
  echo "Warning: adapt_quat controller uses quaternion representation — consider --ori_type quat"
fi

# Select the DP launch file
if [[ "$CONTROLLER_TYPE" == "adapt_quat" ]]; then
  DP_LAUNCH="auv_setp dp_quat.launch.py"
elif [[ "$CONTROLLER_TYPE" == "pid" ]]; then
  DP_LAUNCH="auv_setp dp.launch.py controller_type:=pid orientation_mode:=$ORI_TYPE"
else
  DP_LAUNCH="auv_setp dp.launch.py controller_type:=adaptive orientation_mode:=$ORI_TYPE"
fi

echo "[LAUNCH] ori_type=$ORI_TYPE  controller_type=$CONTROLLER_TYPE  dp_launch=$DP_LAUNCH"

# Kill existing session if it exists
tmux kill-session -t "$SESSION" 2>/dev/null

# =============================================
# Window 1: control (4 equal panes)
# =============================================
tmux new-session -d -s "$SESSION" -n "control"

PANE_C1=$(tmux list-panes -t "$SESSION:control" -F '#{pane_id}')
tmux send-keys -t "$PANE_C1" "s && ros2 launch auv_setup thruster.launch.py" Enter

PANE_C2=$(tmux split-window -h -t "$PANE_C1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C2" "s && ros2 launch operation_mode_manager operation_mode_manager.launch.py" Enter

PANE_C3=$(tmux split-window -v -t "$PANE_C1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C3" "s && ros2 launch $DP_LAUNCH" Enter

PANE_C4=$(tmux split-window -v -t "$PANE_C2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C4" "s" Enter

tmux select-layout -t "$SESSION:control" tiled

# =============================================
# Window 2: perception (4 equal panes)
# =============================================
tmux new-window -t "$SESSION" -n "perception"

PANE_P1=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_P1" "s && ros2 launch auv_setup nucleus_odom_transformer.launch.py" Enter

PANE_P2=$(tmux split-window -h -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P2" "s" Enter

PANE_P3=$(tmux split-window -v -t "$PANE_P1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P3" "s" Enter

PANE_P4=$(tmux split-window -v -t "$PANE_P2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_P4" "s" Enter

tmux select-layout -t "$SESSION:perception" tiled

# =============================================
# Window 3: misc (4 equal panes)
# =============================================
tmux new-window -t "$SESSION" -n "misc"

PANE_M1=$(tmux list-panes -t "$SESSION:misc" -F '#{pane_id}')
tmux send-keys -t "$PANE_M1" "s" Enter

PANE_M2=$(tmux split-window -h -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M2" "s" Enter

PANE_M3=$(tmux split-window -v -t "$PANE_M1" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M3" "s" Enter

PANE_M4=$(tmux split-window -v -t "$PANE_M2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_M4" "s" Enter

tmux select-layout -t "$SESSION:misc" tiled

# =============================================
# Focus control window and attach
# =============================================
tmux select-window -t "$SESSION:control"
tmux attach-session -t "$SESSION"
