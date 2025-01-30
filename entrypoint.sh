#!/bin/bash
# This script builds and runs a Docker image

# Define environment variables for the Docker build process and execute the build script
IMAGE="asv-image:latest" \
BASE_IMAGE="rwthika/ros2:humble" \
TARGET="dev" \
PLATFORM="linux/arm64" \
./docker/docker-ros/scripts/build.sh

# Run the built Docker image with appropriate flags
docker run -it --rm \
    --privileged \
    --network=host \
    --ipc=host \
    -v "$(pwd):/docker-ros/ws/src/target:rshared" \
    "asv-image:latest" /bin/bash
