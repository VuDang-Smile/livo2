#!/bin/bash
# Script helper để chạy Theta Viewer GUI

# Lấy đường dẫn script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Source ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Cảnh báo: Không tìm thấy ROS2 Jazzy tại /opt/ros/jazzy/setup.bash"
    echo "Vui lòng cài đặt ROS2 Jazzy hoặc điều chỉnh đường dẫn"
fi

# Source workspace
if [ -f "$SCRIPT_DIR/ws/install/setup.sh" ]; then
    source "$SCRIPT_DIR/ws/install/setup.sh"
    echo "Đã source workspace setup.sh"
else
    echo "Cảnh báo: Không tìm thấy workspace setup.sh"
    echo "Vui lòng build workspace trước:"
    echo "  cd $SCRIPT_DIR/ws"
    echo "  colcon build --packages-select theta_driver --symlink-install"
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

# Chạy GUI
cd "$GUI_DIR"
python3 theta_viewer.py


