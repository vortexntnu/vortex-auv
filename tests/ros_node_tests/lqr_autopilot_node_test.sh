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
    kill -TERM -"$CONTROLLER_PID" || true
    exit 1
}
trap cleanup ERR

# Launch controller node
setsid ros2 launch velocity_controller_lqr velocity_controller_lqr.launch.py &
CONTROLLER_PID=$!
echo "Launched controller with PID: $CONTROLLER_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 topic pub /orca/killswitch std_msgs/msg/Bool "{data: false}" -t 5
ros2 topic pub /orca/operation_mode std_msgs/msg/String "{data: 'autonomous mode'}" -t 5

# Check if controller correctly publishes tau
echo "Waiting for wrench data..."
timeout 10s ros2 topic echo /orca/wrench_input --once
echo "Got wrench data"

# Terminate processes
kill -TERM -"$CONTROLLER_PID"

echo "Test completed successfully."
