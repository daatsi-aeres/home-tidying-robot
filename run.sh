#!/bin/bash
# TidyBot — One-command launch
# Starts Gazebo, Nav2, RViz, and the autonomous tidying task.
# Press Enter when prompted to start the mission.

cd "$(dirname "$0")"
source install/setup.bash 2>/dev/null || { echo "Run 'colcon build' first."; exit 1; }

# Clean stale signal
rm -f /tmp/tidybot_start

# Launch with filtered output (background)
ros2 launch tidybot simulation.launch.py 2>&1 \
    | grep --line-buffered -E "tidy_task|ground_truth|Managed nodes" &
PIPE_PID=$!

# Wait for systems to load, then prompt user
sleep 28
echo ""
read -r -p ">>> Press Enter to start the tidying mission... "
echo ""

# Signal the task node
touch /tmp/tidybot_start

# Wait for completion
wait $PIPE_PID
