#!/bin/bash
set -e
set -o pipefail

echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. "${WORKSPACE:-$HOME/ros2_ws}/install/setup.bash"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Get the directory of this script dynamically
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Function to terminate processes safely on error
cleanup() {
    echo "Error detected. Cleaning up..."
    kill -TERM -"$SIM_PID" -"$MOBY_PID" -"$CONTROLLER_PID" -"$FILTER_PID" -"$OP_MODE_PID" || true
    exit 1
}
trap cleanup ERR

setsid ros2 bag record -o ${WORKSPACE}/bags/recording -s mcap -a &
BAG_PID=$!
echo "Started bagging with PID: $BAG_PID"

# Launch Stonefish Simulator
setsid ros2 launch stonefish_sim simulation.launch.py rendering:=false scenario:=moby_no_gpu &
SIM_PID=$!
echo "Launched simulator with PID: $SIM_PID"

# Launch MOBY Simulation
setsid ros2 launch stonefish_sim drone_sim.launch.py &
MOBY_PID=$!
echo "Launched moby with PID: $MOBY_PID"

echo "Waiting for simulator to start..."
timeout 30s bash -c '
    while ! ros2 topic list | grep -q "/moby/odom"; do
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
timeout 10s ros2 topic echo /moby/odom --once
echo "Got odom data"

echo "Waiting for sim interface to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/moby/pose"; do sleep 1; done'
echo "Simulator started"

# Check for ROS errors again
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Wait for pose data
echo "Waiting for pose data..."
timeout 10s ros2 topic echo /moby/pose --once
echo "Got pose data"

# Launch controller and reference filter
setsid ros2 launch auv_setup dp.launch.py &
CONTROLLER_PID=$!
echo "Launched controller and reference filter with PID: $CONTROLLER_PID"

# Check for ROS errors before continuing
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

echo "Sleeping for 5 seconds to make sure operation is stable..."
sleep 5

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 service call /moby/set_killswitch vortex_msgs/srv/SetKillswitch "{killswitch_on: false}"
ros2 service call /moby/set_operation_mode vortex_msgs/srv/SetOperationMode "{requested_operation_mode: {operation_mode: 1}}"

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
kill -TERM -"$SIM_PID" -"$MOBY_PID" -"$CONTROLLER_PID" -"$BAG_PID" -"$OP_MODE_PID"

echo "Test completed successfully."
