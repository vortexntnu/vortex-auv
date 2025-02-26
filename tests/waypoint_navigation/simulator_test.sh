#!/bin/bash
set -e

# Load ROS 2 environment
echo "Setting up ROS 2 environment..."
. /opt/ros/humble/setup.sh
. ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

# Get the directory of this script dynamically
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Launch Stonefish Simulator
setsid ros2 launch stonefish_sim simulation_nogpu.launch.py &
SIM_PID=$!
echo "Launched simulator with PID: $SIM_PID"

echo "Waiting for simulator to start..."
timeout 30s bash -c '
    while ! ros2 topic list | grep -q "/orca/odom"; do
        sleep 1
    done || true'
echo "Simulator started"

# Wait for odometry data
echo "Waiting for odom data..."
timeout 10s ros2 topic echo /orca/odom --once
echo "Got odom data"

# Launch ORCA Simulation
setsid ros2 launch stonefish_sim orca_sim.launch.py &
ORCA_PID=$!
echo "Launched orca with PID: $ORCA_PID"

echo "Waiting for sim interface to start..."
timeout 30s bash -c 'until ros2 topic list | grep -q "/orca/pose"; do sleep 1; done'
echo "Simulator started"

# Wait for pose data
echo "Waiting for pose data..."
timeout 10s ros2 topic echo /orca/pose --once
echo "Got pose data"

# Launch Controllers
setsid ros2 launch dp_adapt_backs_controller dp_adapt_backs_controller.launch.py &
CONTROLLER_PID=$!
echo "Launched controller with PID: $CONTROLLER_PID"

setsid ros2 launch reference_filter_dp reference_filter.launch.py &
FILTER_PID=$!
echo "Launched filter with PID: $FILTER_PID"

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 topic pub /orca/killswitch std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /orca/operation_mode std_msgs/msg/String "{data: 'autonomous mode'}" -1

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
kill -TERM -"$SIM_PID" -"$ORCA_PID" -"$CONTROLLER_PID" -"$FILTER_PID"
