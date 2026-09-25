#!/bin/bash
#
# Launch script for planar_arm_control
# Usage: 
#   ./launch.sh              # Launches in default 'velocity' mode
#   ./launch.sh position     # Launches in 'position' mode

cleanup() {
    echo "Cleaning up ROS 2 processes..."
    # Ensure background ROS nodes are killed cleanly when script exits
    pkill -2 -f "controller_node|gui_node|ros2 launch planar_arm_control"
    sleep 1
}

# Catch Ctrl+C (SIGINT) and kill signals (SIGTERM)
trap 'cleanup' SIGINT SIGTERM

# Default parameters
CONTROL_MODE="velocity"
MAX_VEL="1.5"

# Allow passing 'position' or 'velocity' as an argument
if [ "$1" = "position" ]; then
    CONTROL_MODE="position"
elif [ "$1" = "velocity" ]; then
    CONTROL_MODE="velocity"
elif [ -n "$1" ]; then
    echo "Unknown mode: $1. Defaulting to velocity."
fi

echo "==================================================="
echo " Launching planar_arm_control in $CONTROL_MODE mode"
echo "==================================================="

ros2 launch planar_arm_control bringup.launch.py \
    control_mode:=$CONTROL_MODE \
    max_joint_velocity:=$MAX_VEL &

# Wait for background processes to finish
wait