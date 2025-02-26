#!/bin/bash
set -e

# ----------------------------- GLOBAL VARIABLES -----------------------------
STONEFISH_DIR="$HOME/opt/stonefish"
ROS_WORKSPACE="$HOME/ros2_ws"
LOG_PREFIX="[$(date +%T)]"

# ----------------------------- HELPER FUNCTIONS -----------------------------
log_info() {
    echo -e "$LOG_PREFIX [INFO] $1"
}

log_error() {
    echo -e "$LOG_PREFIX [ERROR] $1" >&2
}

# ----------------------------- PYTHON DEPENDENCIES -----------------------------
install_python_dependencies() {
    log_info "Installing/upgrading Python dependencies..."
    pip3 install --upgrade pip
    pip3 install --upgrade 'numpy<1.25' 'scipy<1.12'
    log_info "Python dependencies installed."
}

# ----------------------------- C++ DEPENDENCIES -----------------------------
install_cpp_dependencies() {
    log_info "Installing required C++ dependencies..."
    sudo apt-get update -qq
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        libglm-dev \
        libsdl2-dev \
        libfreetype6-dev
    log_info "C++ dependencies installed."
}

# ----------------------------- STONEFISH INSTALLATION -----------------------------
install_stonefish() {
    if [ -d "$STONEFISH_DIR" ]; then
        log_info "Stonefish is already installed at $STONEFISH_DIR. Skipping clone."
    else
        log_info "Cloning Stonefish repository..."
        mkdir -p "$STONEFISH_DIR"
        git clone https://github.com/patrykcieslak/stonefish.git "$STONEFISH_DIR"
    fi

    log_info "Building Stonefish..."
    mkdir -p "$STONEFISH_DIR/build"
    cd "$STONEFISH_DIR/build"

    cmake ..
    make -j"$(nproc)"
    sudo make install

    log_info "Stonefish installation complete."
}

# ----------------------------- BUILD ROS 2 PACKAGES -----------------------------
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

# ----------------------------- EXECUTE INSTALLATION -----------------------------
log_info "Starting manual installation of extra dependencies..."
install_python_dependencies
install_cpp_dependencies
install_stonefish
build_ros_workspace

log_info "All dependencies installed successfully."
