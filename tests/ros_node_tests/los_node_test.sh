#!/bin/bash
set -e
set -o pipefail

# Load ROS 2 environment
echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. "${WORKSPACE:-$HOME/ros2_ws}/install/setup.bash"

# Function to terminate processes safely on error
cleanup() {
    echo "Error detected. Cleaning up..."
    kill -TERM -"$LOS_PID" || true
    exit 1
}
trap cleanup ERR

# Launch los guidance node
setsid ros2 launch los_guidance los_guidance.launch.py &
LOS_PID=$!
echo "Launched los guidance with PID: $LOS_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Send action goal
echo "Sending goal..."
ros2 action send_goal /orca/los_guidance vortex_msgs/action/LOSGuidance "{goal: {point: {x: 20.0, y: 20.0, z: 5.0}}}" &
# Check if node correctly publishes guidance
echo "Waiting for guidance data..."
timeout 10s ros2 topic echo /orca/guidance/los --once
echo "Got guidance data"

# Terminate processes
kill -TERM -"$LOS_PID"

echo "Test completed successfully."
