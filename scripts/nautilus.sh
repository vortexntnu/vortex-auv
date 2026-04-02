#!/bin/bash

tmux new-session -d -s "nautilus"

tmux split-window -v -t "nautilus:0"
tmux split-window -h -t "nautilus:0.0"
tmux split-window -h -t "nautilus:0.2"

tmux attach-session -t "nautilus"
