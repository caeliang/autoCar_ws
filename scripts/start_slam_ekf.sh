#!/bin/bash
# SLAM + EKF Quick Start Script

set -e

WORKSPACE="/home/ranim/autoCar_ws"
cd "$WORKSPACE"

source install/setup.bash

echo "================================"
echo "  SLAM + EKF Quick Start"
echo "================================"
echo ""
echo "Make sure you have already started:"
echo "  1. Gazebo simulation (Terminal 1)"
echo "  2. This script will start SLAM + EKF (Terminal 2)"
echo "  3. You should run separately:"
echo "     - RViz: rviz2"
echo "     - Teleop: python3 scripts/keyboard_control.py"
echo ""
echo "Starting SLAM + EKF in 3 seconds..."
sleep 3

echo ""
echo "Starting ros2 launch slam_mapping slam_with_ekf.launch.py"
echo ""

# Check if processes are still running, allow graceful shutdown
trap 'echo "Shutting down SLAM + EKF..."; kill $!; exit' INT TERM

ros2 launch slam_mapping slam_with_ekf.launch.py &
LAUNCH_PID=$!

wait $LAUNCH_PID
