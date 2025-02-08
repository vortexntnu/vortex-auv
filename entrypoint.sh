#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for the image, base, target stage, and platform.
# ------------------------------------------------------------------------------
export IMAGE="auv-image:latest"             # Name for the built Docker image
export BASE_IMAGE="ros:humble-ros-base"        # Base image (an official ROS 2 Humble image)
export TARGET="dev"                          # Target build stage (e.g., dev, run, build)

# Set the target platform.
# On Darwin (macOS), we force a Linux platform (since Docker builds Linux images),
# otherwise we use the host's architecture.
if [[ "$(uname)" == "Darwin" ]]; then
    export PLATFORM="linux/arm64"
else
    export PLATFORM="$(dpkg --print-architecture)"
fi

# ------------------------------------------------------------------------------
# Determine paths.
# ------------------------------------------------------------------------------
# SCRIPT_DIR: The absolute path of the directory containing this script.
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# WORKSPACE: The parent directory of SCRIPT_DIR (assumed to be the repository root).
WORKSPACE="$(realpath "$SCRIPT_DIR/..")"

# ------------------------------------------------------------------------------
# 1) Build the Docker image using the build script.
# ------------------------------------------------------------------------------
"$SCRIPT_DIR/docker/build.sh"

# ------------------------------------------------------------------------------
# 2) Run the Docker container.
#    Mount the workspace into the container to allow read/write operations.
# ------------------------------------------------------------------------------
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -v "$WORKSPACE":/docker/ws/src \
    "$IMAGE" /bin/bash
