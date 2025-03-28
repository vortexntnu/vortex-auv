#!/bin/bash
set -e
set -o pipefail

echo "Setting up ROS 2 environment..."
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Function to terminate processes safely on error
cleanup() {
    echo "Error detected. Cleaning up..."
    kill -TERM -"$SIM_PID" -"$ORCA_PID" -"$CONTROLLER_PID" -"$LOS_PID" || true
    exit 1
}
trap cleanup ERR

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

# Check for ROS errors in logs
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

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

# Check for ROS errors again
if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Wait for pose data
echo "Waiting for pose data..."
timeout 10s ros2 topic echo /orca/pose --once
echo "Got pose data"

setsid ros2 launch los_guidance los_guidance.launch.py &
LOS_PID=$!

setsid ros2 launch velocity_controller_lqr velocity_controller_lqr.launch.py &
CONTROLLER_PID=$!

if journalctl -u ros2 | grep -i "error"; then
    echo "Error detected in ROS logs. Exiting..."
    exit 1
fi

# Set operation mode
echo "Turning off killswitch and setting operation mode to autonomous mode"
ros2 topic pub /orca/killswitch std_msgs/msg/Bool "{data: false}" -1
ros2 topic pub /orca/operation_mode std_msgs/msg/String "{data: 'autonomous mode'}" -1

# Send waypoint goal
echo "Sending goal"
python3 "$SCRIPT_DIR/send_goal.py"

if [ $? -ne 0 ]; then
    echo "Test failed: Drone did not reach goal."
    exit 1
else
    echo "Test passed: Drone reached goal."
fi

# Terminate processes
kill -TERM -"$SIM_PID" -"$ORCA_PID" -"$CONTROLLER_PID"

echo "Test completed successfully."
