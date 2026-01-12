#!/bin/bash
# Script helper để chạy Theta Viewer GUI

# Lấy đường dẫn script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "------------------------------------------------"
echo "   THETA DRIVER DASHBOARD INITIALIZING..."
echo "------------------------------------------------"

# 1. Source ROS2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "[OK] Sourced ROS2 Jazzy"
else
    echo "[!] Cảnh báo: Không tìm thấy ROS2 Jazzy tại /opt/ros/jazzy/setup.bash"
fi

# 2. Source drive_ws (livox_ros_driver2)
if [ -f "$SCRIPT_DIR/dependencies/drive_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/dependencies/drive_ws/install/setup.bash"
    echo "[OK] Sourced drive_ws"
else
    echo "[!] Cảnh báo: Không tìm thấy drive_ws. Kiểm tra: $SCRIPT_DIR/dependencies/drive_ws"
fi

# 3. Source ws chính
if [ -f "$SCRIPT_DIR/ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ws/install/setup.bash"
    echo "[OK] Sourced workspace chính"
else
    echo "[!] Lỗi: Không tìm thấy ws/install/setup.bash. Hãy chạy 'colcon build' trước."
fi

# 4. Quản lý Python Environment
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    source "$GUI_DIR/venv/bin/activate"
    echo "[OK] Activated Virtual Environment"
else
    echo "[i] Sử dụng System Python"
fi

# 5. Fix Qt/OpenCV Conflict (Quan trọng nếu dùng camera)
export QT_QPA_PLATFORM_PLUGIN_PATH=""
if [ -n "$QT_PLUGIN_PATH" ]; then
    export QT_PLUGIN_PATH=$(echo "$QT_PLUGIN_PATH" | tr ':' '\n' | grep -v cv2 | grep -v -i opencv | tr '\n' ':' | sed 's/:$//')
fi

# 6. Chạy GUI
echo "------------------------------------------------"
echo "Đang khởi động giao diện tại: $GUI_DIR"
cd "$GUI_DIR"

export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

# Kiểm tra file python có tồn tại không trước khi chạy
if [ -f "main_recording.py" ]; then
    # 1. Chạy script kiểm tra trạng thái ban đầu
    python3 check_status_worker.py
    STATUS=$?

    if [ $STATUS -eq 0 ]; then
        echo "Thiết bị đã đăng ký. Đang mở Main..."
        python3 main_recording.py
    else
        echo "Thiết bị chưa đăng ký. Đang mở Register..."
        # Chạy đăng ký
        python3 main_registration.py
        
        # Sau khi main_registration.py đóng (do lệnh self.root.destroy())
        # Chúng ta kiểm tra lại một lần nữa hoặc tin tưởng mở luôn Main:
        echo "Đang kiểm tra lại sau đăng ký..."
        python3 check_status_worker.py
        if [ $? -eq 0 ]; then
             python3 main_recording.py
        else
             echo "Đăng ký không thành công hoặc người dùng đã tắt form."
        fi
    fi
else
    echo "[X] Lỗi: Không tìm thấy file main_recording.py"
fi