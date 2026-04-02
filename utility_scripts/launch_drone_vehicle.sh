#!/bin/bash
# Launch drone vehicle stack in a tmux session

SESSION="drone_vehicle"

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
tmux send-keys -t "$PANE_C3" "s" Enter

PANE_C4=$(tmux split-window -v -t "$PANE_C2" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_C4" "s" Enter

tmux select-layout -t "$SESSION:control" tiled

# =============================================
# Window 2: perception (4 equal panes)
# =============================================
tmux new-window -t "$SESSION" -n "perception"

PANE_P1=$(tmux list-panes -t "$SESSION:perception" -F '#{pane_id}')
tmux send-keys -t "$PANE_P1" "s" Enter

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
