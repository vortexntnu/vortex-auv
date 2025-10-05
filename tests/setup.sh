#!/bin/bash
set -e

ROS_WORKSPACE="${WORKSPACE:-$HOME/ros2_ws}"
LOG_PREFIX="[$(date +%T)]"

# ----------------------------- HELPER FUNCTIONS -----------------------------
log_info() {
    echo -e "$LOG_PREFIX [INFO] $1"
}

log_error() {
    echo -e "$LOG_PREFIX [ERROR] $1" >&2
}

build_ros_workspace() {
    log_info "Setting up ROS 2 workspace..."
    cd "$ROS_WORKSPACE"

    log_info "Sourcing ROS 2 setup..."
    . /opt/ros/humble/setup.sh

    log_info "Building stonefish_ros2 first (dependency for other packages)..."
    colcon build --packages-select stonefish_ros2 --symlink-install

    log_info "Sourcing workspace..."
    . install/setup.bash

    log_info "Building remaining ROS 2 packages..."
    colcon build --packages-ignore stonefish_ros2 --symlink-install

    log_info "ROS 2 workspace build complete."
}

build_ros_workspace
