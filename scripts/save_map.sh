#!/bin/bash
#===============================================================================
# SLAM Map Saver Script
#===============================================================================
# This script saves the current SLAM map to a file
# 
# Output files:
#   - maps/<map_name>.pgm  (occupancy grid image)
#   - maps/<map_name>.yaml (map metadata)
#   - maps/<map_name>.posegraph (slam_toolbox format - optional)
#===============================================================================

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default map name with timestamp
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DEFAULT_MAP_NAME="compact_city_map_${TIMESTAMP}"

# Get map name from argument or use default
MAP_NAME="${1:-$DEFAULT_MAP_NAME}"

echo -e "${BLUE}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    SLAM Map Saver                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Navigate to workspace
cd ~/autoCar_ws

# Source workspace
source install/setup.bash

# Create maps directory if not exists
mkdir -p maps

MAP_PATH="$HOME/autoCar_ws/maps/${MAP_NAME}"

echo -e "${YELLOW}Map name: ${MAP_NAME}${NC}"
echo -e "${YELLOW}Save path: ${MAP_PATH}${NC}"
echo ""

# Check if SLAM is running
echo -e "${YELLOW}[1/3] Checking SLAM status...${NC}"
if ! ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
    echo -e "${RED}  ✗ SLAM toolbox is not running!${NC}"
    echo -e "${YELLOW}  Please start SLAM first: ./run_slam_ekf.sh${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ SLAM toolbox is running${NC}"

# Check if /map topic exists and has publishers
echo -e "${YELLOW}[2/3] Checking map data...${NC}"
MAP_INFO=$(ros2 topic info /map 2>&1 || echo "")
if echo "$MAP_INFO" | grep -q "Publisher count: 0"; then
    echo -e "${RED}  ✗ No map publishers available!${NC}"
    exit 1
elif echo "$MAP_INFO" | grep -q "Publisher count:"; then
    echo -e "${GREEN}  ✓ Map topic has publishers${NC}"
else
    echo -e "${YELLOW}  ⚠ Could not verify map topic, continuing anyway...${NC}"
fi

# Save map using map_saver_cli (standard ROS 2 way)
echo -e "${YELLOW}[3/3] Saving map...${NC}"
echo ""

# Method 1: Use nav2_map_server's map_saver_cli
if command -v ros2 &> /dev/null; then
    echo -e "${BLUE}Saving with map_saver_cli...${NC}"
    ros2 run nav2_map_server map_saver_cli -f "${MAP_PATH}" --ros-args -p use_sim_time:=true
    
    if [ -f "${MAP_PATH}.pgm" ] && [ -f "${MAP_PATH}.yaml" ]; then
        echo ""
        echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${GREEN}║                    Map Saved Successfully!                     ║${NC}"
        echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
        echo ""
        echo -e "${BLUE}Saved files:${NC}"
        echo "  📄 ${MAP_PATH}.pgm  (occupancy grid image)"
        echo "  📄 ${MAP_PATH}.yaml (map metadata)"
        echo ""
        
        # Show file sizes
        PGM_SIZE=$(ls -lh "${MAP_PATH}.pgm" | awk '{print $5}')
        YAML_SIZE=$(ls -lh "${MAP_PATH}.yaml" | awk '{print $5}')
        echo -e "${BLUE}File sizes:${NC}"
        echo "  • PGM:  ${PGM_SIZE}"
        echo "  • YAML: ${YAML_SIZE}"
        echo ""
        
        # Also try to save slam_toolbox serialized map
        echo -e "${YELLOW}Saving SLAM toolbox serialized map...${NC}"
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${MAP_PATH}'}" 2>/dev/null || true
        
        if [ -f "${MAP_PATH}.posegraph" ]; then
            echo -e "${GREEN}  ✓ Posegraph saved: ${MAP_PATH}.posegraph${NC}"
        fi
        
        echo ""
        echo -e "${BLUE}To use this map later:${NC}"
        echo "  # For Nav2 navigation:"
        echo "  ros2 launch nav2_bringup bringup_launch.py map:=${MAP_PATH}.yaml"
        echo ""
        echo "  # For SLAM localization mode:"
        echo "  ros2 launch slam_toolbox online_async_launch.py map_file_name:=${MAP_PATH}"
        echo ""
    else
        echo -e "${RED}  ✗ Failed to save map!${NC}"
        exit 1
    fi
fi
