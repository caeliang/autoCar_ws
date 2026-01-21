#!/bin/bash
#===============================================================================
# COMPACT CITY - SLAM + EKF Launcher (3D Lidar)
#===============================================================================
# This script launches SLAM + EKF for Compact City with 3D Lidar support
# Run this AFTER starting the simulation with: ./run_compact_city.sh
#
# Components:
# - PointCloud to LaserScan: Converts 3D PointCloud2 to 2D LaserScan
# - EKF: Fuses Odom + IMU + GPS
# - SLAM Toolbox: Creates map from 2D LaserScan
# - TF Publishers: Required transforms
#
# Topics:
#   Input:
#     /prius/scan (PointCloud2) - 3D Lidar data
#   Converted:
#     /prius/scan_2d (LaserScan) - 2D scan for SLAM
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
echo "║           COMPACT CITY - SLAM + EKF                            ║"
echo "╠════════════════════════════════════════════════════════════════╣"
echo "║  EKF: Odom + IMU + GPS fusion                                  ║"
echo "║  SLAM: Lidar-based mapping                                     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Navigate to workspace
cd ~/autoCar_ws

# Source workspace
echo -e "${YELLOW}[1/3] Sourcing workspace...${NC}"
source install/setup.bash
echo -e "${GREEN}  ✓ Workspace sourced${NC}"

# Check if Gazebo is running
echo -e "${YELLOW}[2/3] Checking Gazebo...${NC}"
if ! pgrep -x "gzserver" > /dev/null; then
    echo -e "${RED}  ✗ Gazebo is not running!${NC}"
    echo -e "${YELLOW}  Please start simulation first: ./run_compact_city.sh${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ Gazebo is running${NC}"

# Check if required topics exist
echo -e "${YELLOW}[3/3] Checking sensors...${NC}"
sleep 2

TOPICS_OK=true
for topic in "/prius/scan" "/prius/odom" "/prius/imu"; do
    if timeout 2 ros2 topic info "$topic" &> /dev/null; then
        echo -e "${GREEN}  ✓ $topic${NC}"
    else
        echo -e "${RED}  ✗ $topic not found${NC}"
        TOPICS_OK=false
    fi
done

if [ "$TOPICS_OK" = false ]; then
    echo -e "${YELLOW}  Warning: Some topics missing. Continuing anyway...${NC}"
fi

# Launch SLAM + EKF
LAUNCH_FILE="$HOME/autoCar_ws/src/launch/compact_city_slam_ekf.launch.py"

echo ""
echo -e "${YELLOW}Starting SLAM + EKF...${NC}"
echo ""

ros2 launch "$LAUNCH_FILE" use_sim_time:=true &
SLAM_PID=$!

sleep 5

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║         SLAM + EKF (3D Lidar) is running!                      ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}ROS 2 Topics:${NC}"
echo "  3D Lidar Input:"
echo "    /prius/scan          - PointCloud2 (3D, z∈[0.5m,1.5m])"
echo ""
echo "  Converted 2D (for SLAM):"
echo "    /prius/scan_2d       - LaserScan (2D)"
echo ""
echo "  Other Sensors:"
echo "    /prius/odom          - Odometry"
echo "    /prius/imu           - IMU"
echo "    /prius/gps/fix       - GPS"
echo ""
echo "  Output:"
echo "    /odometry/filtered   - EKF fused odometry"
echo "    /map                 - SLAM map"
echo ""
echo -e "${BLUE}Commands:${NC}"
echo "  # Visualize (with pre-configured topics):"
echo "  ./run_rviz.sh"
echo ""
echo "  # Check 3D PointCloud:"
echo "  ros2 topic hz /prius/scan"
echo ""
echo "  # Check 2D LaserScan:"
echo "  ros2 topic hz /prius/scan_2d"
echo ""
echo "  # Check EKF output:"
echo "  ros2 topic echo /odometry/filtered"
echo ""
echo "  # Save map:"
echo "  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: 'compact_city_map'}}\""
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop SLAM + EKF${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping SLAM + EKF...${NC}"
    kill $SLAM_PID 2>/dev/null || true
    echo -e "${GREEN}SLAM + EKF stopped.${NC}"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Wait
wait
