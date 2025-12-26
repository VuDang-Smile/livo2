#!/bin/bash

# Script để start Localization2 với FAST-LIVO2
# Sử dụng: ./start_localization2.sh [map_directory] [config_file]

set -e

# Màu sắc
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Đường dẫn
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS_DIR="$PROJECT_ROOT/ws"
ROS2_SETUP="/opt/ros/jazzy/setup.bash"

# Tham số
MAP_DIR="$1"
CONFIG_FILE="${2:-mid360.yaml}"
USE_FAST_LIVO="${3:-false}"  # Default: false - chỉ chạy localization, không chạy mapping
USE_RVIZ="${4:-true}"

# Kiểm tra ROS2
if [ ! -f "$ROS2_SETUP" ]; then
    echo -e "${RED}Error: ROS2 Jazzy not found at $ROS2_SETUP${NC}"
    exit 1
fi

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${BLUE}Sourcing ROS2 Jazzy...${NC}"
    source "$ROS2_SETUP"
fi

# Source workspace
if [ -f "$WS_DIR/install/setup.bash" ]; then
    echo -e "${BLUE}Sourcing workspace...${NC}"
    source "$WS_DIR/install/setup.bash"
else
    echo -e "${YELLOW}Warning: Workspace not built. Building now...${NC}"
    cd "$PROJECT_ROOT"
    ./build.sh
    source "$WS_DIR/install/setup.bash"
fi

# Kiểm tra map directory
if [ -z "$MAP_DIR" ]; then
    echo -e "${YELLOW}Usage: $0 <map_directory> [config_file] [use_fast_livo] [use_rviz]${NC}"
    echo -e "${YELLOW}Example: $0 /path/to/map mid360.yaml true true${NC}"
    echo ""
    echo -e "${BLUE}Available maps in Log directory:${NC}"
    if [ -d "$PROJECT_ROOT/ws/src/FAST-LIVO2/Log" ]; then
        find "$PROJECT_ROOT/ws/src/FAST-LIVO2/Log" -type d -name "map_*" | head -5
    fi
    exit 1
fi

if [ ! -d "$MAP_DIR" ]; then
    echo -e "${RED}Error: Map directory not found: $MAP_DIR${NC}"
    exit 1
fi

# Kiểm tra package (sửa lỗi BrokenPipeError - tránh pipe error)
# Thử kiểm tra trực tiếp trong workspace trước (nhanh hơn và không bị pipe error)
if [ -f "$WS_DIR/install/fast_lio_localization/share/fast_lio_localization/package.xml" ]; then
    # Package đã được build, OK
    :
elif ros2 pkg list 2>/dev/null | grep -q "fast_lio_localization" 2>/dev/null; then
    # Package có trong ros2 pkg list, OK
    :
else
    # Package không tìm thấy
    echo -e "${RED}Error: fast_lio_localization package not found!${NC}"
    echo -e "${YELLOW}Please build the workspace first:${NC}"
    echo "  cd $PROJECT_ROOT && ./build.sh"
    exit 1
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Starting Localization2 with FAST-LIVO2${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Map directory: ${BLUE}$MAP_DIR${NC}"
echo -e "Config file: ${BLUE}$CONFIG_FILE${NC}"
echo -e "Use FAST-LIVO2: ${BLUE}$USE_FAST_LIVO${NC}"
echo -e "Use RViz: ${BLUE}$USE_RVIZ${NC}"
echo ""

# Launch localization
ros2 launch fast_lio_localization localization_with_fast_livo2.launch.py \
    map_root:="$MAP_DIR" \
    localization_config:="$CONFIG_FILE" \
    use_fast_livo:="$USE_FAST_LIVO" \
    rviz:="$USE_RVIZ"

