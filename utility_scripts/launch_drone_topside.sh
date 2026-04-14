#!/bin/bash
# Launch drone simulation stack in a tmux session

SESSION="drone_launch"
S="source ~/vscopium/ros2_ws/install/setup.bash"

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

# Grab the initial pane ID
PANE_SIM=$(tmux list-panes -t "$SESSION:sim" -F '#{pane_id}')

# Stonefish simulation (top-left)
tmux send-keys -t "$PANE_SIM" "clear && $S && ros2 launch stonefish_sim simulation.launch.py drone:=nautilus" Enter

# Split right -> Keyboard joy (top-right)
PANE_JOY=$(tmux split-window -h -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_JOY" "clear && $S && ros2 launch keyboard_joy keyboard_joy_node.launch.py" Enter

# Split sim pane down -> AUV setup (bottom-left)
PANE_DP=$(tmux split-window -v -t "$PANE_SIM" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_DP" "clear && $S && ros2 launch auv_setup dp.launch.py" Enter

# Split joy pane down -> Drone sim (bottom-right)
PANE_DRONE=$(tmux split-window -v -t "$PANE_JOY" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_DRONE" "clear && $S && ros2 launch stonefish_sim drone_sim.launch.py" Enter

# Force equal pane sizes
tmux select-layout -t "$SESSION:sim" tiled

# =============================================
# Window 2: tools (2 panes)
# =============================================
tmux new-window -t "$SESSION" -n "tools"

PANE_FOX=$(tmux list-panes -t "$SESSION:tools" -F '#{pane_id}')
tmux send-keys -t "$PANE_FOX" "clear && $S && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" Enter

# Split down -> Message publisher
PANE_MSG=$(tmux split-window -v -t "$PANE_FOX" -P -F '#{pane_id}')
tmux send-keys -t "$PANE_MSG" "clear && $S && ros2 launch vortex_utility_nodes message_publisher.launch.py" Enter

# =============================================
# Focus first window and attach
# =============================================
tmux select-window -t "$SESSION:sim"
tmux attach-session -t "$SESSION"
