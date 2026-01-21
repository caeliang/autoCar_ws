#!/bin/bash
#===============================================================================
# 3D SLAM - RViz Visualization
#===============================================================================
# This script launches RViz with pre-configured topics for 3D SLAM
#
# 3D Topics displayed:
# - /occupied_cells_vis_array - 3D Octomap (colored voxels)
# - /free_cells_vis_array - Free space visualization
# - /prius/scan        - Raw 3D Point Cloud (cyan)
# - /odometry/filtered - EKF filtered odometry (green arrows)
# - /path              - Trajectory path
# - TF transforms
#
# Optional 2D topics (disabled by default):
# - /map               - 2D SLAM map
# - /prius/scan_2d     - 2D LaserScan
#===============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║           3D SLAM - RViz Visualization                         ║"
echo "║           (Octomap + Point Cloud)                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Navigate to workspace
cd ~/autoCar_ws

# Source workspace
echo -e "${YELLOW}[1/3] Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}  ✓ Workspace sourced${NC}"

# Check if RViz config exists
RVIZ_CONFIG="$HOME/autoCar_ws/config/slam_3d.rviz"

if [ ! -f "$RVIZ_CONFIG" ]; then
    echo -e "${RED}  ✗ RViz config not found: $RVIZ_CONFIG${NC}"
    echo -e "${YELLOW}  Creating default 3D SLAM config...${NC}"
    # Config will be created by the file creation above
fi
echo -e "${GREEN}  ✓ RViz config found${NC}"

# Display information
echo -e "\n${CYAN}[2/3] Configuration:${NC}"
echo -e "${MAGENTA}  Topics to visualize:${NC}"
echo -e "    • ${GREEN}/occupied_cells_vis_array${NC} - 3D Voxel Map (Occupied)"
echo -e "    • ${GREEN}/free_cells_vis_array${NC} - 3D Voxel Map (Free Space)"
echo -e "    • ${GREEN}/prius/scan${NC} - Raw 3D Point Cloud"
echo -e "    • ${GREEN}/odometry/filtered${NC} - Robot Odometry"
echo -e "    • ${GREEN}/path${NC} - Trajectory"
echo ""
echo -e "${YELLOW}  Tips:${NC}"
echo -e "    • Use mouse wheel to zoom"
echo -e "    • Middle-click + drag to rotate view"
echo -e "    • Shift + middle-click to pan"
echo -e "    • Toggle displays in left panel"
echo -e "    • Fixed Frame: ${CYAN}map${NC}"
echo ""

# Launch RViz
echo -e "${YELLOW}[3/3] Starting RViz...${NC}"
echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  RViz is starting... Please wait${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Run RViz with 3D SLAM config
rviz2 -d "$RVIZ_CONFIG"

# Cleanup message
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  RViz closed${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
