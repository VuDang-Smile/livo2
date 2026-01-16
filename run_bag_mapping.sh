#!/bin/bash
# Script helper to run Theta Viewer GUI

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "------------------------------------------------"
echo "   THETA DRIVER DASHBOARD INITIALIZING..."
echo "------------------------------------------------"

# 1. Source ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS2 Jazzy"
else
    echo "[!] Warning: ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
fi

# 2. Source drive_ws (livox_ros_driver2)
if [ -f "$SCRIPT_DIR/dependencies/drive_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/dependencies/drive_ws/install/setup.bash"
    echo "[OK] Sourced drive_ws"
else
    echo "[!] Warning: drive_ws not found. Check: $SCRIPT_DIR/dependencies/drive_ws"
fi

# 3. Source main workspace
if [ -f "$SCRIPT_DIR/ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ws/install/setup.bash"
    echo "[OK] Sourced main workspace"
else
    echo "[!] Error: ws/install/setup.bash not found. Please run 'colcon build' first."
fi

# 4. Python Environment Management
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    source "$GUI_DIR/venv/bin/activate"
    echo "[OK] Activated Virtual Environment"
else
    echo "[i] Using System Python"
fi

# 5. Run GUI
echo "------------------------------------------------"
echo "Starting interface at: $GUI_DIR"
cd "$GUI_DIR"

# Check if python file exists before running
if [ -f "main_bag_mapping.py" ]; then
    python3 main_bag_mapping.py
else
    echo "[X] Error: File gui/main_bag_mapping.py not found"
fi