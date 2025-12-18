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
    kill -TERM -"$FILTER_PID" || true
    exit 1
}
trap cleanup ERR

# Launch reference filter node
setsid ros2 launch reference_filter_dp reference_filter_dp.launch.py &
FILTER_PID=$!
echo "Launched reference filter with PID: $FILTER_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Send action goal
echo "Sending goal..."
ros2 action send_goal /orca/reference_filter vortex_msgs/action/ReferenceFilterWaypoint \
"{waypoint: {pose: {position: {x: 1.0,y: 0.0,z: 0.0}, orientation:{x: 0,y: 0,z: 0,w: 1}}, mode: 0}}" &

sleep 2

# Check if controller correctly publishes guidance
echo "Waiting for guidance data..."
timeout 10s ros2 topic echo /orca/guidance/dp --once
echo "Got guidance data"

# Terminate processes
kill -TERM -"$FILTER_PID"

echo "Test completed successfully."
