#!/bin/bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source "${WORKSPACE:-$HOME/ros2_ws}/install/setup.bash"
ros2 run eskf ros_contract_test.py
