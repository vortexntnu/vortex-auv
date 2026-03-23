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

install_dependencies() {
    log_info "Installing additional dependencies..."
    "$CALLER_REPO/scripts/ci_install_dependencies.sh"
}

build_ros_workspace() {
    log_info "Setting up ROS 2 workspace..."
    cd "$ROS_WORKSPACE"

    log_info "Sourcing ROS 2 setup..."
    . /opt/ros/humble/setup.sh

    log_info "Building stonefish_ros2 and vortex_utils first (dependencies for other packages)..."
    colcon build --packages-select stonefish_ros2 vortex_utils

    log_info "Sourcing workspace..."
    . install/setup.bash

    log_info "Building remaining ROS 2 packages..."
    colcon build --packages-ignore stonefish_ros2 vortex_utils

    log_info "Sourcing workspace..."
    . install/setup.bash

    log_info "Rebuilding eskf with landmark_egomotion support..."
    colcon build --packages-select eskf

    log_info "ROS 2 workspace build complete."
}

install_dependencies
build_ros_workspace
