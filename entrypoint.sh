#!/usr/bin/env bash
set -euo pipefail

# ------------------------------------------------------------------------------
# Set environment variables for the image, base, and platform.
# ------------------------------------------------------------------------------
export IMAGE="auv-image:latest"             # Name of the built Docker image
export BASE_IMAGE="ros:humble-ros-base"     # Base image (official ROS 2 Humble)

# ------------------------------------------------------------------------------
# Set the target platform.
# On Darwin (macOS), we force a Linux platform (since Docker builds Linux images),
# otherwise we use the host's architecture.
# ------------------------------------------------------------------------------
if [[ "$(uname)" == "Darwin" ]]; then
    export PLATFORM="linux/arm64"
else
    export PLATFORM="$(dpkg --print-architecture)"
fi

# ------------------------------------------------------------------------------
# Locate this script and the project root
# ------------------------------------------------------------------------------
SCRIPT_DIR="$(dirname "$(realpath "$0")")"
WORKSPACE="$(realpath "$SCRIPT_DIR/../..")"

# ------------------------------------------------------------------------------
# 1) Build the Docker image
# ------------------------------------------------------------------------------
"$SCRIPT_DIR/docker/build.sh"

# ------------------------------------------------------------------------------
# 2) Run the Docker container
# ------------------------------------------------------------------------------
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -v "$WORKSPACE":/docker/ws \
    "$IMAGE" /bin/bash
