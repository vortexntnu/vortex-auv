#!/bin/bash

tmux new-session -d -s "orca"

tmux split-window -v -t "orca:0"
tmux split-window -h -t "orca:0.0"
tmux split-window -h -t "orca:0.2"

tmux send-keys -t "orca:0.0" "ros2 launch thrust_allocator_auv thrust_allocator_auv.launch.py" C-m
tmux send-keys -t "orca:0.1" "ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py" C-m
tmux send-keys -t "orca:0.2" "ros2 launch reference_filter_dp reference_filter.launch.py" C-m

tmux attach-session -t "orca"
