#!/bin/bash
set -e
set -o pipefail

echo "Testing that the ESKF node is able to start up and publish odom"

# Load ROS 2 environment
echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. "${WORKSPACE:-$HOME/ros2_ws}/install/setup.bash"

# Function to terminate processes safely on error
cleanup() {
    echo "Error detected. Cleaning up..."
    kill -TERM -"$ESKF_PID" || true
    exit 1
}
trap cleanup ERR

# Launch eskf node
setsid ros2 launch eskf eskf.launch.py &
ESKF_PID=$!
echo "Launched eskf with PID: $ESKF_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Check if eskf correctly publishes odom
echo "Waiting for odom data..."
timeout 10s ros2 topic echo /orca/odom --once
echo "Got odom data"

# Terminate processes
kill -TERM -"$ESKF_PID"

echo "Test completed successfully."
