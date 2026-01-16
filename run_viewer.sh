#!/bin/bash
# Script helper to run Theta Viewer GUI

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Warning: ROS2 Jazzy not found at /opt/ros/jazzy/setup.bash"
    echo "Please install ROS2 Jazzy or adjust the path"
fi

# Source drive_ws (for livox_ros_driver2)
if [ -f "$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh" ]; then
    source "$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh"
    echo "Sourced dependencies/drive_ws/install/setup.sh (livox_ros_driver2)"
else
    echo "Warning: dependencies/drive_ws/install/setup.sh not found"
    echo "Please build drive_ws first"
fi

# Source ws (for other packages like livox_msg_converter, theta_driver)
if [ -f "$SCRIPT_DIR/ws/install/setup.sh" ]; then
    source "$SCRIPT_DIR/ws/install/setup.sh"
    echo "Sourced ws/install/setup.sh (other packages)"
else
    echo "Warning: ws/install/setup.sh not found"
    echo "Please build workspace first:"
    echo "  cd $SCRIPT_DIR/ws"
    echo "  colcon build --packages-select livox_msg_converter theta_driver --symlink-install"
fi

# Activate virtual environment if available (optional)
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    echo "Activating virtual environment..."
    source "$GUI_DIR/venv/bin/activate"
else
    echo "Not using virtual environment, using system Python"
    echo "If dependencies are not installed, run: ./install_dependencies.sh"
fi

# Fix Qt plugin issue with OpenCV
# Unset or clean QT_PLUGIN_PATH to avoid conflicts with OpenCV Qt plugins
if [ -n "$QT_PLUGIN_PATH" ]; then
    # Remove paths containing cv2 or opencv
    export QT_PLUGIN_PATH=$(echo "$QT_PLUGIN_PATH" | tr ':' '\n' | grep -v cv2 | grep -v -i opencv | tr '\n' ':' | sed 's/:$//')
    if [ -z "$QT_PLUGIN_PATH" ]; then
        unset QT_PLUGIN_PATH
    fi
fi

# Set QT_QPA_PLATFORM_PLUGIN_PATH to avoid loading Qt plugins from OpenCV
export QT_QPA_PLATFORM_PLUGIN_PATH=""

# Run GUI
cd "$GUI_DIR"
python3 theta_viewer.py


