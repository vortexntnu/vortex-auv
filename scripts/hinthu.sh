#!/bin/bash
 
# Navigate to the ROS 2 workspace directory
cd /home/vortex/stonefish_ws
 
# Source the ROS 2 setup file to set up the environment
source install/setup.bash
 
# Launch the Stonefish simulation in the background
ros2 launch stonefish_sim simulation.launch.py scenario:=docking rendering_quality:=low &
 
# Launch the Orca simulation
ros2 launch stonefish_sim orca_sim.launch.py &
 
ros2 launch auv_setup dp.launch.py &

ros2 launch keyboard_joy keyboard_joy_node.launch.py &

ros2 launch aruco_detector aruco_detector.launch.py &
# Launch the Foxglove Bridge in the background
ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
 
# Wait for all background processes to finish
wait