#!/bin/bash
# Run 3D SLAM with Octomap

echo "════════════════════════════════════════"
echo "  Starting 3D SLAM System (Octomap)"
echo "════════════════════════════════════════"
echo ""
echo "Features:"
echo "  ✓ 3D Point Cloud Mapping"
echo "  ✓ Real-time Octomap Generation"
echo "  ✓ EKF Sensor Fusion"
echo "  ✓ 2D SLAM for comparison"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Source the workspace
source /home/ranim/autoCar_ws/install/setup.bash

# Launch 3D SLAM
ros2 launch slam_3d slam_3d.launch.py
