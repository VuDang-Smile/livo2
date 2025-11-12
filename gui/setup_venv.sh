#!/bin/bash
# Script để setup virtual environment và cài đặt dependencies

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=== Setup Virtual Environment cho Theta Viewer GUI ==="

# Tạo venv nếu chưa có
if [ ! -d "venv" ]; then
    echo "Đang tạo virtual environment..."
    python3 -m venv venv
    if [ $? -ne 0 ]; then
        echo "Lỗi: Không thể tạo virtual environment"
        echo "Vui lòng cài đặt python3-venv: sudo apt install python3-venv"
        exit 1
    fi
    echo "✓ Đã tạo virtual environment"
else
    echo "✓ Virtual environment đã tồn tại"
fi

# Activate venv
echo "Đang activate virtual environment..."
source venv/bin/activate

# Đảm bảo dùng pip từ venv
PIP_CMD="$SCRIPT_DIR/venv/bin/pip"
PYTHON_CMD="$SCRIPT_DIR/venv/bin/python"

# Upgrade pip
echo "Đang upgrade pip..."
$PIP_CMD install --upgrade pip

# Cài đặt dependencies
echo "Đang cài đặt dependencies từ requirements.txt..."
if [ -f "requirements.txt" ]; then
    $PIP_CMD install -r requirements.txt
    if [ $? -eq 0 ]; then
        echo "✓ Đã cài đặt tất cả dependencies"
    else
        echo "✗ Lỗi khi cài đặt dependencies"
        exit 1
    fi
else
    echo "✗ Không tìm thấy requirements.txt"
    exit 1
fi

echo ""
echo "=== Setup hoàn tất ==="
echo ""
echo "Để sử dụng virtual environment:"
echo "  source venv/bin/activate"
echo ""
echo "Để chạy GUI:"
echo "  ./run_viewer.sh"
echo "  hoặc"
echo "  source venv/bin/activate && python3 theta_viewer.py"

