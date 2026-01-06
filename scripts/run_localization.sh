#!/bin/bash
# Script để chạy Localization với FAST-LIO-LOCALIZATION2
# Usage: ./run_localization.sh <map_root_dir> [use_rviz]

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WS_PATH="$PROJECT_ROOT/ws"
DRIVE_WS_PATH="$PROJECT_ROOT/dependencies/drive_ws"

# Check arguments
if [ $# -lt 1 ]; then
    echo -e "${RED}Usage: $0 <map_root_dir> [use_rviz]${NC}"
    echo -e "${YELLOW}Example: $0 /home/an/Desktop/lidar/livo2/ws/src/FAST-LIVO2/Log/fastloc_map True${NC}"
    exit 1
fi

MAP_ROOT="$1"
USE_RVIZ="${2:-True}"

# Check map root
if [ ! -d "$MAP_ROOT" ]; then
    echo -e "${RED}Error: Map root directory does not exist: $MAP_ROOT${NC}"
    exit 1
fi

if [ ! -f "$MAP_ROOT/pose.json" ]; then
    echo -e "${RED}Error: pose.json not found in $MAP_ROOT${NC}"
    exit 1
fi

# Check workspace setup
WS_SETUP="$WS_PATH/install/setup.sh"
if [ ! -f "$WS_SETUP" ]; then
    echo -e "${RED}Error: Workspace not built. Please build workspace first.${NC}"
    echo -e "${YELLOW}Run: ./build.sh and select fast_lio_localization${NC}"
    exit 1
fi

# Check drive_ws (optional, for CustomMsg support)
DRIVE_WS_SETUP="$DRIVE_WS_PATH/install/setup.sh"
USE_DRIVE_WS=false
if [ -f "$DRIVE_WS_SETUP" ]; then
    USE_DRIVE_WS=true
    echo -e "${GREEN}✓ Found drive_ws, will source it for CustomMsg support${NC}"
fi

# ROS2 setup
ROS2_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${YELLOW}Warning: ROS2 setup not found at $ROS2_SETUP${NC}"
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}FAST-LIO Localization (Smile Reference)${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Map root: ${YELLOW}$MAP_ROOT${NC}"
echo -e "RViz2: ${YELLOW}$USE_RVIZ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Source environments
source "$ROS2_SETUP"
if [ "$USE_DRIVE_WS" = true ]; then
    source "$DRIVE_WS_SETUP"
fi
source "$WS_SETUP"

# Launch localization
ros2 launch fast_lio_localization localization.launch.py \
    map_root:="$MAP_ROOT" \
    rviz:="$USE_RVIZ" \
    config_file:="mid360.yaml"

