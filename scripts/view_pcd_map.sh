#!/bin/bash
# View saved PCD map

if [ -z "$1" ]; then
    echo "Usage: ./view_pcd_map.sh <map_name>"
    echo ""
    echo "Available maps:"
    ls -1 maps/*.pcd 2>/dev/null | sed 's/maps\//  - /' | sed 's/\.pcd//'
    echo ""
    exit 1
fi

MAP_FILE="maps/${1}.pcd"

if [ ! -f "$MAP_FILE" ]; then
    MAP_FILE="$1"
fi

if [ ! -f "$MAP_FILE" ]; then
    echo "✗ Map not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    ls -1 maps/*.pcd 2>/dev/null | sed 's/maps\//  - /' | sed 's/\.pcd//'
    exit 1
fi

echo "Opening: $MAP_FILE"
echo ""

# Try pcl_viewer first
if command -v pcl_viewer &> /dev/null; then
    pcl_viewer "$MAP_FILE"
# Try cloudcompare
elif command -v cloudcompare &> /dev/null; then
    cloudcompare "$MAP_FILE"
# Try CloudCompare (capital C)
elif command -v CloudCompare &> /dev/null; then
    CloudCompare "$MAP_FILE"
else
    echo "✗ No viewer found!"
    echo ""
    echo "Install a viewer:"
    echo "  sudo apt install pcl-tools"
    echo "  # or"
    echo "  sudo snap install cloudcompare"
    echo ""
    echo "Or open in RViz:"
    echo "  1. Start RViz: rviz2"
    echo "  2. Add → PointCloud2"
    echo "  3. Topic → (none) → Browse"
    echo "  4. Select: $MAP_FILE"
fi
