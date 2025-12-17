#!/bin/bash
# Script to create calibration bag from MCAP recording using ROS2 nodes
# This script uses ros2 bag play/record with converter nodes instead of rosbags library

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
DURATION=5.0
INPUT_PATH=""
OUTPUT_PATH=""

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --input|-i)
            INPUT_PATH="$2"
            shift 2
            ;;
        --output|-o)
            OUTPUT_PATH="$2"
            shift 2
            ;;
        --duration|-d)
            DURATION="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 --input <input_recording_path> [--output <output_bag_path>] [--duration <seconds>]"
            echo ""
            echo "Example:"
            echo "  $0 --input /path/to/recording_20251210_165248 --output calibration_bag --duration 5"
            exit 0
            ;;
        *)
            if [ -z "$INPUT_PATH" ]; then
                INPUT_PATH="$1"
            else
                echo -e "${RED}Error: Unknown argument $1${NC}"
                exit 1
            fi
            shift
            ;;
    esac
done

# Check if input path is provided
if [ -z "$INPUT_PATH" ]; then
    echo -e "${RED}Error: Input recording path is required${NC}"
    echo "Usage: $0 --input <input_recording_path> [--output <output_bag_path>] [--duration <seconds>]"
    exit 1
fi

# Convert to absolute path
INPUT_PATH=$(realpath "$INPUT_PATH")

# Check if input path exists
if [ ! -d "$INPUT_PATH" ]; then
    echo -e "${RED}Error: Input path does not exist: $INPUT_PATH${NC}"
    exit 1
fi

# Determine output path
if [ -z "$OUTPUT_PATH" ]; then
    OUTPUT_PATH="${INPUT_PATH}_calibration"
else
    OUTPUT_PATH=$(realpath -m "$OUTPUT_PATH")
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Creating Calibration Bag${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Input recording:${NC} $INPUT_PATH"
echo -e "${GREEN}Output bag:${NC} $OUTPUT_PATH"
echo -e "${GREEN}Duration:${NC} $DURATION seconds"
echo ""

# Source ROS2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo -e "${RED}Error: ROS2 Jazzy not found. Please install ROS2 Jazzy first.${NC}"
    exit 1
fi

# Source workspace if exists
WS_PATH=$(dirname $(dirname $(dirname $(realpath "$0"))))
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
    echo -e "${GREEN}Sourced workspace:${NC} $WS_PATH"
fi

# Check if output directory exists, remove if it does
if [ -d "$OUTPUT_PATH" ]; then
    echo -e "${YELLOW}Warning: Output directory already exists. Removing...${NC}"
    rm -rf "$OUTPUT_PATH"
fi

# Create output directory
mkdir -p "$(dirname "$OUTPUT_PATH")"

echo ""
echo -e "${BLUE}Starting bag recording with converter nodes...${NC}"
echo ""

# Topics to record
TOPICS="/image_raw /camera_info /livox/lidar /livox/point2"

# Function to cleanup on exit
cleanup() {
    echo -e "${YELLOW}Cleaning up processes...${NC}"
    kill $RECORD_PID $CAMERA_INFO_PID $CONVERTER_PID 2>/dev/null || true
    wait $RECORD_PID $CAMERA_INFO_PID $CONVERTER_PID 2>/dev/null || true
    exit 0
}

# Set trap to cleanup on exit
trap cleanup SIGINT SIGTERM EXIT

# Start camera_info_publisher node first
echo -e "${GREEN}Starting camera_info_publisher node...${NC}"
ros2 run theta_driver camera_info_publisher_node &
CAMERA_INFO_PID=$!

# Start livox_msg_converter node with output topic /livox/point2
echo -e "${GREEN}Starting livox_msg_converter node...${NC}"
ros2 run livox_msg_converter livox_msg_converter_node \
    --ros-args \
    -p input_topic:=/livox/lidar \
    -p output_topic:=/livox/point2 \
    -p frame_id:=livox_frame &
CONVERTER_PID=$!

# Wait a bit for nodes to start
sleep 3

# Start recording in background
echo -e "${GREEN}Starting ros2 bag record...${NC}"
ros2 bag record -o "$OUTPUT_PATH" $TOPICS &
RECORD_PID=$!

# Wait a bit for recorder to start
sleep 2

# Play the recording
echo -e "${GREEN}Playing recording for $DURATION seconds...${NC}"
timeout $DURATION ros2 bag play "$INPUT_PATH" || true

# Wait a bit for messages to be processed
sleep 3

# Stop recording first
echo -e "${YELLOW}Stopping bag recording...${NC}"
kill $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

# Then stop converter nodes
echo -e "${YELLOW}Stopping converter nodes...${NC}"
kill $CAMERA_INFO_PID $CONVERTER_PID 2>/dev/null || true
wait $CAMERA_INFO_PID $CONVERTER_PID 2>/dev/null || true

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Calibration bag created successfully!${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Output bag:${NC} $OUTPUT_PATH"
echo ""
echo -e "${BLUE}Topics in bag:${NC}"
ros2 bag info "$OUTPUT_PATH" | grep -E "^\s+/" || true
echo ""


