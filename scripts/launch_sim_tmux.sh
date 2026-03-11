#!/bin/bash

SESSION="stonefish_sim"
WS="$HOME/ros2_ws"

# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# --- SIMULATION ---
tmux new-session -d -s $SESSION -n sim

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch stonefish_sim simulation.launch.py scenario:=docking rendering_quality:=low" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch stonefish_sim orca_sim.launch.py" C-m


# --- CONTROL ---
tmux new-window -t $SESSION -n control

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch auv_setup dp.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch keyboard_joy keyboard_joy_node.launch.py" C-m


# --- PERCEPTION ---
tmux new-window -t $SESSION -n perception

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch aruco_detector aruco_detector.launch.py" C-m


# --- TOOLS ---
tmux new-window -t $SESSION -n tools

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch waypoint_manager waypoint_manager.launch.py" C-m


# --- MISSION ---
tmux new-window -t $SESSION -n mission

tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 launch landmark_server landmark_server.launch.py" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "cd $WS && source install/setup.bash && ros2 run yasmin_viewer yasmin_viewer_node" C-m


# Nice layout
tmux select-layout tiled

# Attach to session
tmux attach-session -t $SESSION
