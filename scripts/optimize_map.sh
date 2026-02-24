#!/bin/bash

# Map Optimization Script
# Cleans and optimizes PCD maps for localization

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if input file provided
if [ -z "$1" ]; then
    echo -e "${RED}❌ Error: No input file specified${NC}"
    echo ""
    echo "Usage: ./optimize_map.sh <input_map.pcd> [output_name]"
    echo "Example: ./optimize_map.sh maps/my_map.pcd my_map_clean"
    echo ""
    exit 1
fi

INPUT_FILE="$1"
OUTPUT_NAME="${2:-$(basename "$INPUT_FILE" .pcd)_optimized}"

# Check if file exists
if [ ! -f "$INPUT_FILE" ]; then
    echo -e "${RED}❌ Error: File not found: $INPUT_FILE${NC}"
    exit 1
fi

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║        MAP OPTIMIZATION FOR LOCALIZATION               ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2
cd /home/ranim/autoCar_ws
source install/setup.bash

# Use virtual environment Python
PYTHON_CMD="/home/ranim/autoCar_ws/.venv/bin/python"

# Run Python optimization script
$PYTHON_CMD src/slam_3d/scripts/optimize_map.py "$INPUT_FILE" "$OUTPUT_NAME"

echo ""
echo -e "${GREEN}✅ Done!${NC}"
