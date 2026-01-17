#!/bin/bash
# Test script to verify SLAM + EKF integration

set -e

WORKSPACE="/home/ranim/autoCar_ws"
cd "$WORKSPACE"

echo "================================"
echo "SLAM + EKF Integration Test"
echo "================================"
echo ""

# Source the setup
source install/setup.bash

# Check if ros2 is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    echo "Make sure to source ROS2 setup: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "[1/5] Checking required topics..."
sleep 2

# List available topics
echo ""
echo "Available topics:"
ros2 topic list | grep -E "(odom|imu|gps|scan)" || echo "  (waiting for topics...)"

echo ""
echo "[2/5] Checking TF tree..."
sleep 1

# Check TF
echo ""
echo "Transform tree:"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>/dev/null || echo "  (map -> odom not yet available)"

echo ""
echo "[3/5] Launching SLAM + EKF..."
echo ""
echo "Command to run:"
echo "  ros2 launch slam_mapping slam_with_ekf.launch.py"
echo ""

read -p "Start SLAM + EKF now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Launching SLAM + EKF..."
    ros2 launch slam_mapping slam_with_ekf.launch.py
else
    echo "Skipped. You can start manually with:"
    echo "  cd $WORKSPACE"
    echo "  source install/setup.bash"
    echo "  ros2 launch slam_mapping slam_with_ekf.launch.py"
fi
