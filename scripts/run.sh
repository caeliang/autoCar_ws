WORKSPACE_DIR="$HOME/autoCar_ws"
WORLD_FILE="$WORKSPACE_DIR/src/worlds/compact_city.world"
WORLD_FILE="$WORKSPACE_DIR/src/worlds/compact_city_scaled.world"
MODELS_PATH="$WORKSPACE_DIR/src/models"
PLUGIN_PATH="$WORKSPACE_DIR/install/gazebo_traffic_light_plugin/lib"

export GAZEBO_MODEL_PATH="$MODELS_PATH:$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PLUGIN_PATH:${GAZEBO_PLUGIN_PATH:-}"
export HOME="$HOME"

echo -e "${GREEN}  ✓ Environment configured${NC}"

# Clean up any existing processes
echo -e "${YELLOW}[2/4] Cleaning up old processes...${NC}"
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 2
echo -e "${GREEN}  ✓ Cleanup complete${NC}"

# Start Gazebo server
echo -e "${YELLOW}[3/4] Starting Gazebo server...${NC}"
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
echo -e "${YELLOW}[4/4] Starting Gazebo GUI...${NC}"
gzclient &
GZCLIENT_PID=$!
sleep 3
echo -e "${GREEN}  ✓ Gazebo GUI started (PID: $GZCLIENT_PID)${NC}"

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║              Gazebo Simulation is running!                     ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Next step - Open a new terminal and run:${NC}"
echo "  ./run_slam_ekf.sh"
echo ""
echo -e "${BLUE}Or control the vehicle:${NC}"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/prius/cmd_vel"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping Gazebo...${NC}"
    kill $GZCLIENT_PID 2>/dev/null || true
    kill $GZSERVER_PID 2>/dev/null || true
    pkill -9 gzserver 2>/dev/null || true
    pkill -9 gzclient 2>/dev/null || true
    echo -e "${GREEN}Gazebo stopped.${NC}"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Wait for user to stop