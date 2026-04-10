#!/bin/bash
#===============================================================================
# 🚀 COMPACT CITY - FULL AUTONOMOUS SYSTEM
#===============================================================================
# Complete integration:
#  • Gazebo Simulator (compact_city.world)
#  • Localization (ICP)
#  • Path Planning (A* with 87 waypoints: 3,3 → 45,55)
#  • Pure Pursuit Steering Control
#
# Start: ./run_compact_city.sh
#===============================================================================

set -e

# Arguman sayisini kontrol et
if [ "$#" -lt 4 ]; then
    echo -e "\033[0;31m[HATA] Baslangic ve Bitis koordinatlarini (4 adet) sirasiyla girmelisiniz!\033[0m"
    echo -e "\033[1;33mKullanim:\033[0m ./scripts/run_compact_city.sh <BAS_X> <BAS_Y> <HEDEF_X> <HEDEF_Y>"
    echo -e "\033[1;33mOrnek:\033[0m    ./scripts/run_compact_city.sh 3 3 18 56\n"
    exit 1
fi

START_X=$1
START_Y=$2
GOAL_X=$3
GOAL_Y=$4

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

prompt_for_y() {
    local name="$1"
    while true; do
        echo -en "${YELLOW}▶ Start ${name}? (Press 'y' and Enter to start): ${NC}"
        read -r input
        if [[ "$input" == "y" || "$input" == "Y" ]]; then
            break
        fi
    done
}

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║     🚗 COMPACT CITY - AUTONOMOUS PATH FOLLOWING TEST           ║"
echo "║                                                                ║"
echo "║  Path: ($START_X, $START_Y) → ($GOAL_X, $GOAL_Y) via A*                    ║"
echo "║  System: Gazebo + ICP + Pure Pursuit                          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Navigate to workspace
cd ~/autoCar_ws

# Source workspace
echo -e "${YELLOW}[1/5] Sourcing workspace...${NC}"
source install/setup.bash

# Set environment variables
WORKSPACE_DIR="$HOME/autoCar_ws"
WORLD_FILE="$WORKSPACE_DIR/src/worlds/compact_city_scaled.world"
MODELS_PATH="$WORKSPACE_DIR/src/models"
PLUGIN_PATH="$WORKSPACE_DIR/install/gazebo_traffic_light_plugin/lib"
PLANNED_ROUTE="$WORKSPACE_DIR/waypoints/planned_route.csv"
MAP_PCD="$WORKSPACE_DIR/maps/scaled_build.pcd"

export GAZEBO_MODEL_PATH="$MODELS_PATH:$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PLUGIN_PATH:${GAZEBO_PLUGIN_PATH:-}"
export HOME="$HOME"

echo -e "${GREEN}  ✓ Environment configured${NC}"

# Define Grid Waypoints from arguments, fallback to default if not provided
START_X=${1}
START_Y=${2}
GOAL_X=${3}
GOAL_Y=${4}

echo -e "${YELLOW}📍  Kullanilan Koordinatlar: BASTA ($START_X, $START_Y) -> BITIS ($GOAL_X, $GOAL_Y)${NC}"

echo -e "${YELLOW}[2/5] Generating New Route (A*)...${NC}"
python3 src/path_planning/scripts/path_planning/generate_route.py $START_X $START_Y $GOAL_X $GOAL_Y
echo -e "${GREEN}  ✓ Route generation complete${NC}"

# Clean up any existing processes
echo -e "${YELLOW}[3/6] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -f pure_pursuit 2>/dev/null || true
pkill -f waypoint_manager 2>/dev/null || true
sleep 2
echo -e "${GREEN}  ✓ Cleanup complete${NC}"

# Start Gazebo server
echo -e "${YELLOW}[4/6] Starting Gazebo server...${NC}"
gzserver --verbose "$WORLD_FILE" &
GZSERVER_PID=$!
sleep 5

# Check if Gazebo server is running
if ! ps -p $GZSERVER_PID > /dev/null 2>&1; then
    echo -e "${RED}  ✗ Gazebo server failed to start!${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ Gazebo server started (PID: $GZSERVER_PID)${NC}"

# Start Gazebo client (GUI)
echo -e "${YELLOW}[5/6] Starting Gazebo GUI...${NC}"
prompt_for_y "Gazebo GUI"
gzclient &
GZCLIENT_PID=$!
sleep 3
echo -e "${GREEN}  ✓ Gazebo GUI started (PID: $GZCLIENT_PID)${NC}"

# Visualize the planned path
echo -e "${YELLOW}[5b/6] Visualizing planned route...${NC}"
prompt_for_y "Route Visualization"
(
  sleep 2
  echo -e "${CYAN}  → Plotting route from ($START_X,$START_Y) to ($GOAL_X,$GOAL_Y)...${NC}"
  python3 scripts/visualize_path.py matrices/road_grid_4wide.txt waypoints/planned_route.csv $START_X $START_Y $GOAL_X $GOAL_Y
) &
PLOT_PID=$!
sleep 2
echo -e "${GREEN}  ✓ Route visualization launched${NC}"

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║        🚗 AUTONOMOUS SYSTEM LAUNCHING...                       ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Start Localization (ICP)
echo -e "${YELLOW}[6/6] Starting Localization (ICP)...${NC}"
prompt_for_y "ICP Localization"
(
  sleep 3
  echo -e "${CYAN}  → Launching ICP localization...${NC}"
  ros2 launch localization simple_localization.launch.py \
    map_pcd_path:="$MAP_PCD" 2>&1 | sed 's/^/    [ICP] /'
) &
ICP_PID=$!
sleep 4
echo -e "${GREEN}  ✓ ICP localization started${NC}"

# Start Full Autonomous System (Waypoint Manager + Tracker + Pure Pursuit)
echo -e "${CYAN}  → Launching Autonomous System (A* + Pure Pursuit)...${NC}"
prompt_for_y "Autonomous System (Path Planning & Pure Pursuit)"
(
  sleep 2
  ros2 launch path_planning full_autonomous_system.launch.py \
    waypoint_file:="$PLANNED_ROUTE" 2>&1 | sed 's/^/    [AUTO] /'
) &
AUTO_PID=$!
sleep 3
echo -e "${GREEN}  ✓ Autonomous system started${NC}"

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              🎯 SYSTEM READY FOR AUTONOMOUS NAV                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo -e "${CYAN}📊 Running Components:${NC}"
echo "  • Gazebo (Server PID: $GZSERVER_PID)"
echo "  • Gazebo GUI (Client PID: $GZCLIENT_PID)"
echo "  • Route Visualization (PID: $PLOT_PID)"
echo "  • ICP Localization (PID: $ICP_PID)"
echo "  • Autonomous System (PID: $AUTO_PID)"
echo ""

echo -e "${CYAN}📈 Monitor in another terminal:${NC}"
echo "  source ~/autoCar_ws/install/setup.bash"
echo "  ros2 topic echo /prius/cmd_vel                   # Steering commands"
echo "  ros2 topic echo /waypoints/path --once            # Path visualization"
echo "  ros2 run rviz2 rviz2                             # RViz visualization"
echo ""

echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping all systems...${NC}"
    kill $AUTO_PID 2>/dev/null || true
    kill $ICP_PID 2>/dev/null || true
    kill $PLOT_PID 2>/dev/null || true
    kill $GZCLIENT_PID 2>/dev/null || true
    kill $GZSERVER_PID 2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    pkill -f pure_pursuit 2>/dev/null || true
    pkill -f waypoint_manager 2>/dev/null || true
    echo -e "${GREEN}All systems stopped.${NC}"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Wait for user to stop
wait
