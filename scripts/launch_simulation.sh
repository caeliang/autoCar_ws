#!/bin/bash
#
# Launch script for AutoCar Gazebo Simulation
# Traffic lights work automatically when the world is loaded
# Usage: ./launch_simulation.sh [--background]
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

# Add the plugin library path to GAZEBO_PLUGIN_PATH
export GAZEBO_PLUGIN_PATH="$WORKSPACE_DIR/install/gazebo_traffic_light_plugin/lib:$GAZEBO_PLUGIN_PATH"

# Set Gazebo model path to include custom models
export GAZEBO_MODEL_PATH="$WORKSPACE_DIR/src/models:/home/ranim/.gazebo/models:$GAZEBO_MODEL_PATH"

echo "=============================================="
echo "       AutoCar Gazebo Simulation"
echo "=============================================="
echo "Workspace: $WORKSPACE_DIR"
echo "=============================================="

# Kill any existing Gazebo processes
pkill -f gzserver 2>/dev/null
pkill -f gzclient 2>/dev/null
sleep 1

# Launch Gazebo with the world file
if [ "$1" == "--background" ]; then
    echo "Starting simulation in background..."
    gazebo "$WORKSPACE_DIR/src/worlds/simple_city_copy.world" --verbose &
    sleep 5
    echo "Simulation started. PID: $!"
else
    gazebo "$WORKSPACE_DIR/src/worlds/simple_city_copy.world" --verbose
fi
