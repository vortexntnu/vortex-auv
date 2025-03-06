#!/bin/bash

tmux new-session -d -s "orca"

tmux split-window -v -t "orca:0"
tmux split-window -h -t "orca:0.0"
tmux split-window -h -t "orca:0.2"

tmux send-keys -t "orca:0.0" "ros2 launch auv_setup orca.launch.py" C-m
tmux send-keys -t "orca:0.1" "ros2 launch auv_setup dp.launch.py" C-m
tmux send-keys -t "orca:0.2" "ros2 launch driver_stim300 stim300_driver.launch.py" C-m
tmux send-keys -t "orca:0.3" "ros2 launch ros_dvl_a50_driver ros_dvl_a50_driver.launch.py" C-m

tmux attach-session -t "orca"
