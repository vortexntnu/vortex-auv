#!/bin/bash
# This script builds and runs a Docker image

# Define environment variables for the Docker build process and execute the build script
export IMAGE="asv-image:latest"
export BASE_IMAGE="rwthika/ros2:humble"
export TARGET="dev"
# export PLATFORM="linux/arm64"  # Uncomment this line for ARM64 builds
# export PLATFORM="amd64"  # Uncomment this line for AMD64 builds

# Run the build script
./docker/docker-ros/scripts/build.sh

# Run the built Docker image with appropriate flags
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -v "$(pwd):/docker-ros/ws/src/target:rshared" \
    "asv-image:latest" /bin/bash
