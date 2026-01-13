#!/bin/bash
# Script helper để chạy Theta Viewer GUI

# Lấy đường dẫn script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "------------------------------------------------"
echo "   THETA DRIVER DASHBOARD INITIALIZING..."
echo "------------------------------------------------"

# 1. Source ws chính
if [ -f "$SCRIPT_DIR/ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ws/install/setup.bash"
    echo "[OK] Sourced workspace chính"
else
    echo "[!] Lỗi: Không tìm thấy ws/install/setup.bash. Hãy chạy 'colcon build' trước."
fi

# 2. Quản lý Python Environment
GUI_DIR="$SCRIPT_DIR/gui"
if [ -d "$GUI_DIR/venv" ]; then
    source "$GUI_DIR/venv/bin/activate"
    echo "[OK] Activated Virtual Environment"
else
    echo "[i] Sử dụng System Python"
fi

# 3. Chạy GUI
echo "------------------------------------------------"
echo "Đang khởi động giao diện tại: $GUI_DIR"
cd "$GUI_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"


# Kiểm tra file python có tồn tại không trước khi chạy
if [ -f "main_registration.py" ]; then
    python3 main_registration.py
else
    echo "[X] Lỗi: Không tìm thấy file gui/main_registration.py"
fi