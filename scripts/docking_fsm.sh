#!/bin/bash

tmux new-session -d -s "docking-fsm"

tmux split-window -v -t "docking-fsm:0"
tmux split-window -h -t "docking-fsm:0.0"
tmux split-window -h -t "docking-fsm:0.2"

tmux send-keys -t "docking-fsm:0.0" "ros2 launch dock_action_servers docking_action_servers.launch.py" C-m
tmux send-keys -t "docking-fsm:0.1" "ros2 launch auv_setup dp.launch.py " C-m
tmux send-keys -t "docking-fsm:0.2" "ros2 launch pose_action_server pose_action_server.launch.py" C-m
tmux send-keys -t "docking-fsm:0.3" "ros2 launch stonefish_sim simulation_nogpu.launch.py" C-m

tmux attach-session -t "docking-fsm"
