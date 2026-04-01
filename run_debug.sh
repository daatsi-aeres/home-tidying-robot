#!/bin/bash
# TidyBot — Debug mode (teleop)
# Starts Gazebo, RViz, and keyboard teleop. No autonomous navigation.
# Drive around with i/j/k/l keys in the xterm window.

cd "$(dirname "$0")"
source install/setup.bash 2>/dev/null || { echo "Run 'colcon build' first."; exit 1; }
ros2 launch tidybot debug.launch.py 2>&1 | grep --line-buffered -E "ground_truth|map_server|teleop|Managed nodes"
