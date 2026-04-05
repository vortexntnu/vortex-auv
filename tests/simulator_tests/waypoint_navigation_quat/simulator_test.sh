#!/bin/bash
set -e
set -o pipefail

echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. "${WORKSPACE:-$HOME/ros2_ws}/install/setup.bash"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Get the directory of this script dynamically
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Function to terminate processes safely on error
cleanup() {
    echo "Error detected. Cleaning up..."
    kill -TERM -"$SIM_PID" -"$NAUTILUS_PID" -"$CONTROLLER_PID" -"$FILTER_PID" -"$BRIDGE_PID" -"$OP_MODE_PID" || true
    exit 1
}
trap cleanup ERR

setsid ros2 bag record -o ${WORKSPACE}/bags/recording -s mcap -a &
BAG_PID=$!
echo "Started bagging with PID: $BAG_PID"

# Launch Stonefish Simulator
setsid ros2 launch stonefish_sim simulation.launch.py rendering:=false scenario:=nautilus_no_gpu &
SIM_PID=$!
echo "Launched simulator with PID: $SIM_PID"

# Launch NAUTILUS Simulation
setsid ros2 launch stonefish_sim drone_sim.launch.py &
NAUTILUS_PID=$!
echo "Launched nautilus with PID: $NAUTILUS_PID"

echo "Waiting for simulator to start..."
timeout 30s bash -c '
    while ! ros2 topic list | grep -q "/nautilus/odom"; do
        sleep 1
    done || true'
echo "Simulator started"

# Check for ROS errors in logs
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Wait for odometry data
echo "Waiting for odom data..."
timeout 10s ros2 topic echo /nautilus/odom --once
echo "Got odom data"

echo "Waiting for sim interface to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/nautilus/pose"; do sleep 1; done'
echo "Simulator started"

# Check for ROS errors again
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Wait for pose data
echo "Waiting for pose data..."
timeout 10s ros2 topic echo /nautilus/pose --once
echo "Got pose data"

# Launch quaternion reference filter
setsid ros2 launch reference_filter_dp_quat reference_filter_dp_quat.launch.py &
FILTER_PID=$!
echo "Launched quat reference filter with PID: $FILTER_PID"

# Launch quat-to-euler bridge so the Euler-based controller gets guidance/dp
setsid python3 "$SCRIPT_DIR/quat_to_euler_bridge.py" &
BRIDGE_PID=$!
echo "Launched quat-to-euler bridge with PID: $BRIDGE_PID"

# Launch controller separately
setsid ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py &
CONTROLLER_PID=$!
echo "Launched controller with PID: $CONTROLLER_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

echo "Sleeping for 5 seconds to make sure operation is stable..."
sleep 5

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 service call /nautilus/set_killswitch vortex_msgs/srv/SetKillswitch "{killswitch_on: false}"
ros2 service call /nautilus/set_operation_mode vortex_msgs/srv/SetOperationMode "{requested_operation_mode: {operation_mode: 1}}"

echo "Sleeping for 5 seconds to make sure operation is stable..."
sleep 5

# Send waypoint goal
echo "Sending goal"
python3 "$SCRIPT_DIR/send_goal.py"

# Check if goal reached
echo "Checking if goal reached"
python3 "$SCRIPT_DIR/check_goal.py"

if [ $? -ne 0 ]; then
    echo "Test failed: Drone did not reach goal."
    exit 1
else
    echo "Test passed: Drone reached goal."
fi

# Terminate processes
kill -TERM -"$SIM_PID" -"$NAUTILUS_PID" -"$CONTROLLER_PID" -"$FILTER_PID" -"$BRIDGE_PID" -"$BAG_PID" -"$OP_MODE_PID"

echo "Test completed successfully."
