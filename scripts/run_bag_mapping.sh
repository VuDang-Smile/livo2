#!/bin/bash
# Script để chạy Bag Mapping với FAST-LIVO2
# Usage: ./run_bag_mapping.sh <bag_folder_path> [use_rviz]

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
    echo -e "${RED}Usage: $0 <bag_folder_path> [use_rviz] [bag_rate]${NC}"
    echo -e "${YELLOW}Example: $0 /path/to/bag_folder True 0.5${NC}"
    echo -e "${YELLOW}Example: $0 /path/to/bag_folder False 1.0${NC}"
    exit 1
fi

BAG_PATH="$1"
USE_RVIZ="${2:-False}"
BAG_RATE="${3:-1.0}"

# Check bag path
if [ ! -d "$BAG_PATH" ]; then
    echo -e "${RED}Error: Bag folder does not exist: $BAG_PATH${NC}"
    exit 1
fi

# Check workspace setup
WS_SETUP="$WS_PATH/install/setup.sh"
if [ ! -f "$WS_SETUP" ]; then
    echo -e "${RED}Error: Workspace not built. Please build workspace first.${NC}"
    echo -e "${YELLOW}Run: cd $WS_PATH && colcon build${NC}"
    exit 1
fi

# Check drive_ws (optional, for CustomMsg support)
DRIVE_WS_SETUP="$DRIVE_WS_PATH/install/setup.sh"
USE_DRIVE_WS=false
if [ -f "$DRIVE_WS_SETUP" ]; then
    USE_DRIVE_WS=true
    echo -e "${GREEN}✓ Found drive_ws, will source it for CustomMsg support${NC}"
fi

# ROS2 Network Isolation - Prevents interference from other machines
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=10
echo -e "${GREEN}🔒 ROS2 Network Isolation: LOCALHOST_ONLY=1, DOMAIN_ID=10${NC}"

# ROS2 setup
ROS2_SETUP="/opt/ros/jazzy/setup.bash"
if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${YELLOW}Warning: ROS2 setup not found at $ROS2_SETUP${NC}"
    echo -e "${YELLOW}Please source ROS2 manually or adjust ROS2_SETUP in script${NC}"
fi

# Launch file - Single unified config
LAUNCH_FILE="mapping_mid360_equirectangular.launch.py"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}FAST-LIVO2 Bag Mapping${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Bag path: ${YELLOW}$BAG_PATH${NC}"
echo -e "Launch file: ${YELLOW}$LAUNCH_FILE${NC}"
echo -e "RViz2: ${YELLOW}$USE_RVIZ${NC}"
echo -e "Bag rate: ${YELLOW}${BAG_RATE}x${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up processes...${NC}"
    if [ ! -z "$MAPPING_PID" ]; then
        echo -e "${YELLOW}Stopping mapping node (PID: $MAPPING_PID)...${NC}"
        kill -TERM "$MAPPING_PID" 2>/dev/null || true
        wait "$MAPPING_PID" 2>/dev/null || true
    fi
    if [ ! -z "$BAG_PID" ]; then
        echo -e "${YELLOW}Stopping bag playback (PID: $BAG_PID)...${NC}"
        kill -TERM "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
    echo -e "${GREEN}Cleanup complete${NC}"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# Start mapping node
echo -e "${GREEN}[1/2] Starting mapping node...${NC}"
if [ "$USE_DRIVE_WS" = true ]; then
    source "$ROS2_SETUP"
    source "$DRIVE_WS_SETUP"
    source "$WS_SETUP"
    ros2 launch fast_livo "$LAUNCH_FILE" use_rviz:="$USE_RVIZ" &
else
    source "$ROS2_SETUP"
    source "$WS_SETUP"
    ros2 launch fast_livo "$LAUNCH_FILE" use_rviz:="$USE_RVIZ" &
fi
MAPPING_PID=$!

echo -e "${GREEN}✓ Mapping node started (PID: $MAPPING_PID)${NC}"
echo -e "${YELLOW}Waiting 5 seconds for mapping node to initialize...${NC}"
sleep 5

# Check if mapping process is still running
if ! kill -0 "$MAPPING_PID" 2>/dev/null; then
    echo -e "${RED}Error: Mapping node exited immediately${NC}"
    exit 1
fi

# Start bag playback
echo ""
echo -e "${GREEN}[2/2] Starting bag playback...${NC}"
if [ "$USE_DRIVE_WS" = true ]; then
    source "$ROS2_SETUP"
    source "$DRIVE_WS_SETUP"
    source "$WS_SETUP"
    ros2 bag play "$BAG_PATH" --rate "$BAG_RATE" &
else
    source "$ROS2_SETUP"
    source "$WS_SETUP"
    ros2 bag play "$BAG_PATH" --rate "$BAG_RATE" &
fi
BAG_PID=$!

echo -e "${GREEN}✓ Bag playback started (PID: $BAG_PID)${NC}"
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Bag Mapping is running!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Mapping PID: ${YELLOW}$MAPPING_PID${NC}"
echo -e "Bag PID: ${YELLOW}$BAG_PID${NC}"
echo -e "Press ${YELLOW}Ctrl+C${NC} to stop"
echo ""

# Wait for processes
wait $MAPPING_PID $BAG_PID || true

# Cleanup if processes ended
cleanup
