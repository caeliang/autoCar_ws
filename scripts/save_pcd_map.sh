#!/bin/bash
# Save 3D Map as PCD - Quick and Easy!

MAP_NAME="${1:-my_3d_map}"

echo "════════════════════════════════════════"
echo "  Saving 3D Map (PCD): $MAP_NAME"
echo "════════════════════════════════════════"
echo ""

source /home/ranim/autoCar_ws/install/setup.bash

# Check if 3D SLAM is running
if ! ros2 node list 2>/dev/null | grep -q "octomap_server"; then
    echo "✗ ERROR: 3D SLAM not running!"
    echo "  Run: ./run_slam_3d.sh"
    exit 1
fi

# Save PCD
python3 /home/ranim/autoCar_ws/src/slam_3d/scripts/save_pcd.py "$MAP_NAME"

echo ""
echo "Done! Map saved to: maps/${MAP_NAME}.pcd"
echo ""
