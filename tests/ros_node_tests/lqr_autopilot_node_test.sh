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
    kill -TERM -"$OP_MODE_PID" || true
    exit 1
}
trap cleanup ERR

# Launch controller node
setsid ros2 launch velocity_controller_lqr velocity_controller_lqr.launch.py &
CONTROLLER_PID=$!
sleep 5
echo "Launched controller with PID: $CONTROLLER_PID"

# launch operation mode service
setsid ros2 launch operation_mode_manager operation_mode_manager.launch.py &
OP_MODE_PID=$!
echo "Launched operation mode service with PID: $OP_MODE_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 service call /orca/set_killswitch vortex_msgs/srv/SetKillswitch "{killswitch_on: false}"
ros2 service call /orca/set_operation_mode vortex_msgs/srv/SetOperationMode "{requested_operation_mode: {operation_mode: 1}}"

sleep 2
# Check if controller correctly publishes tau
echo "Waiting for wrench data..."
timeout 20s ros2 topic echo /orca/wrench_input --once
echo "Got wrench data"

# Terminate processes
kill -TERM -"$CONTROLLER_PID"
kill -TERM -"$OP_MODE_PID"

echo "Test completed successfully."
