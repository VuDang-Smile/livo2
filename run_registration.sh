#!/bin/bash
# Script helper to run Theta Viewer GUI

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "------------------------------------------------"
echo "   THETA DRIVER DASHBOARD INITIALIZING..."
echo "------------------------------------------------"

# 1. Source main workspace
if [ -f "$SCRIPT_DIR/ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ws/install/setup.bash"
    echo "[OK] Sourced main workspace"
else
    echo "[!] Error: ws/install/setup.bash not found. Please run 'colcon build' first."
fi

# 2. Python Environment Management
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    source "$GUI_DIR/venv/bin/activate"
    echo "[OK] Activated Virtual Environment"
else
    echo "[i] Using System Python"
fi

# 3. Run GUI
echo "------------------------------------------------"
echo "Starting interface at: $GUI_DIR"
cd "$GUI_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"


# Check if python file exists before running
if [ -f "main_registration.py" ]; then
    python3 main_registration.py
else
    echo "[X] Error: File gui/main_registration.py not found"
fi