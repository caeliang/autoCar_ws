rviz2#!/bin/bash
#
# Complete SLAM Workflow Script for AutoCar
# Step 1: Run SLAM while driving around
# Step 2: Save the map
# Step 3: Use map for localization
#
# Usage: 
#   ./slam_workflow.sh slam     - Start SLAM mode (mapping)
#   ./slam_workflow.sh save     - Save current map
#   ./slam_workflow.sh localize - Start localization with saved map
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
MAPS_DIR="$WORKSPACE_DIR/maps"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_header() {
    echo -e "${GREEN}=============================================="
    echo -e "    AutoCar SLAM Workflow"
    echo -e "==============================================${NC}"
}

print_usage() {
    echo "Usage: $0 {slam|save|localize|help}"
    echo ""
    echo "Commands:"
    echo "  slam      - Start SLAM mode for mapping"
    echo "  save      - Save current map to file"
    echo "  localize  - Start localization with saved map"
    echo "  help      - Show this help message"
    echo ""
    echo "Workflow:"
    echo "  1. Start simulation: ./launch_simulation.sh"
    echo "  2. Start EKF: ros2 launch localization ekf_localization.launch.py"
    echo "  3. Start SLAM: ./slam_workflow.sh slam"
    echo "  4. Drive around to build map (use teleop)"
    echo "  5. Save map: ./slam_workflow.sh save"
    echo "  6. Stop SLAM (Ctrl+C)"
    echo "  7. Start localization: ./slam_workflow.sh localize"
}

start_slam() {
    print_header
    echo -e "${YELLOW}Starting SLAM mode...${NC}"
    echo "Drive the car around to build the map."
    echo "When done, open another terminal and run: ./slam_workflow.sh save"
    echo ""
    
    # Check if scan topic is available
    if ! ros2 topic list | grep -q "/prius/scan"; then
        echo -e "${RED}Error: /prius/scan topic not found!${NC}"
        echo "Make sure the simulation is running."
        exit 1
    fi
    
    ros2 launch slam_mapping slam.launch.py
}

save_map() {
    print_header
    mkdir -p "$MAPS_DIR"
    
    MAP_NAME="${1:-city_map}"
    MAP_PATH="$MAPS_DIR/$MAP_NAME"
    
    echo -e "${YELLOW}Saving map to: $MAP_PATH${NC}"
    
    ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH" --ros-args -p use_sim_time:=true
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Map saved successfully!${NC}"
        echo "Files created:"
        echo "  - ${MAP_PATH}.pgm"
        echo "  - ${MAP_PATH}.yaml"
    else
        echo -e "${RED}Failed to save map${NC}"
        exit 1
    fi
}

start_localization() {
    print_header
    
    MAP_FILE="${1:-$MAPS_DIR/city_map.yaml}"
    
    if [ ! -f "$MAP_FILE" ]; then
        echo -e "${RED}Error: Map file not found: $MAP_FILE${NC}"
        echo "Please run SLAM and save a map first."
        echo "Available maps:"
        ls -la "$MAPS_DIR"/*.yaml 2>/dev/null || echo "  (none)"
        exit 1
    fi
    
    echo -e "${YELLOW}Starting localization with map: $MAP_FILE${NC}"
    
    ros2 launch slam_mapping localization.launch.py map:="$MAP_FILE"
}

# Main
case "$1" in
    slam)
        start_slam
        ;;
    save)
        save_map "$2"
        ;;
    localize)
        start_localization "$2"
        ;;
    help|--help|-h)
        print_usage
        ;;
    *)
        echo -e "${RED}Invalid command: $1${NC}"
        print_usage
        exit 1
        ;;
esac
