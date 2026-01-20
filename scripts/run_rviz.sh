#!/bin/bash
#===============================================================================
# COMPACT CITY - RViz Visualization
#===============================================================================
# This script launches RViz with pre-configured topics for Compact City
#
# Topics displayed:
# - /prius/scan        - Lidar (red points)
# - /map               - SLAM map
# - /prius/odom        - Raw odometry (orange arrows)
# - /odometry/filtered - EKF filtered odometry (green arrows)
# - /prius/front_camera/image_raw - Camera image
# - TF transforms
#===============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║           COMPACT CITY - RViz Visualization                    ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Navigate to workspace
cd ~/autoCar_ws

# Source workspace
echo -e "${YELLOW}[1/2] Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}  ✓ Workspace sourced${NC}"

# Check if RViz config exists
RVIZ_CONFIG="$HOME/autoCar_ws/config/compact_city.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo -e "${RED}  ✗ RViz config not found: $RVIZ_CONFIG${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ RViz config found${NC}"

# Launch RViz
echo -e "${YELLOW}[2/2] Starting RViz...${NC}"
echo ""
echo -e "${BLUE}Pre-configured displays:${NC}"
echo "  • PointCloud2 → /prius/scan (cyan - 3D lidar)"
echo "  • LaserScan   → /prius/scan_2d (red - 2D for SLAM)"
echo "  • Map         → /map (SLAM map)"
echo "  • Odometry    → /prius/odom (orange - raw)"
echo "  • Odometry    → /odometry/filtered (green - EKF)"
echo "  • Camera      → /prius/front_camera/image_raw"
echo "  • TF          → All transforms"
echo ""
echo -e "${YELLOW}Starting RViz with use_sim_time:=true...${NC}"
echo ""

rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
