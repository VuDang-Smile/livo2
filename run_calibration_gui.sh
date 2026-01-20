#!/bin/bash
# Script để khởi chạy Calibration Standalone GUI

cd "$(dirname "$0")"

# Source ROS2 environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
else
    echo "Cảnh báo: Không tìm thấy ROS2 Jazzy setup. Vui lòng cài đặt ROS2 Jazzy."
fi

# Chạy GUI
python3 gui/main_calibration.py

