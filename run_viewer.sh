#!/bin/bash
# Script helper để chạy Theta Viewer GUI

# Lấy đường dẫn script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# ROS2 Network Isolation - Prevents interference from other machines
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=10
echo "🔒 ROS2 Network Isolation: LOCALHOST_ONLY=1, DOMAIN_ID=10"

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Cảnh báo: Không tìm thấy ROS2 Jazzy tại /opt/ros/jazzy/setup.bash"
    echo "Vui lòng cài đặt ROS2 Jazzy hoặc điều chỉnh đường dẫn"
fi

# Source drive_ws (cho livox_ros_driver2)
if [ -f "$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh" ]; then
    source "$SCRIPT_DIR/dependencies/drive_ws/install/setup.sh"
    echo "Đã source dependencies/drive_ws/install/setup.sh (livox_ros_driver2)"
else
    echo "Cảnh báo: Không tìm thấy dependencies/drive_ws/install/setup.sh"
    echo "Vui lòng build drive_ws trước"
fi

# Source ws (cho các packages khác như livox_msg_converter, theta_driver)
if [ -f "$SCRIPT_DIR/ws/install/setup.sh" ]; then
    source "$SCRIPT_DIR/ws/install/setup.sh"
    echo "Đã source ws/install/setup.sh (các packages khác)"
else
    echo "Cảnh báo: Không tìm thấy ws/install/setup.sh"
    echo "Vui lòng build workspace trước:"
    echo "  cd $SCRIPT_DIR/ws"
    echo "  colcon build --packages-select livox_msg_converter theta_driver --symlink-install"
fi

# Activate virtual environment nếu có (không bắt buộc)
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    echo "Đang activate virtual environment..."
    source "$GUI_DIR/venv/bin/activate"
else
    echo "Không dùng virtual environment, sử dụng system Python"
    echo "Nếu chưa cài dependencies, chạy: ./install_dependencies.sh"
fi

# Fix Qt plugin issue với OpenCV
# Unset hoặc clean QT_PLUGIN_PATH để tránh xung đột với OpenCV Qt plugins
if [ -n "$QT_PLUGIN_PATH" ]; then
    # Loại bỏ các path chứa cv2 hoặc opencv
    export QT_PLUGIN_PATH=$(echo "$QT_PLUGIN_PATH" | tr ':' '\n' | grep -v cv2 | grep -v -i opencv | tr '\n' ':' | sed 's/:$//')
    if [ -z "$QT_PLUGIN_PATH" ]; then
        unset QT_PLUGIN_PATH
    fi
fi

# Set QT_QPA_PLATFORM_PLUGIN_PATH để tránh load Qt plugins từ OpenCV
export QT_QPA_PLATFORM_PLUGIN_PATH=""

# Chạy GUI
cd "$GUI_DIR"
python3 theta_viewer.py


