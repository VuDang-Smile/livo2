#!/bin/bash
# Script để cài đặt dependencies trực tiếp (không dùng venv)

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=== Cài đặt Dependencies (không dùng venv) ==="
echo ""

# Kiểm tra xem có venv không
if [ -d "venv" ]; then
    echo "⚠️  Phát hiện virtual environment đã tồn tại"
    echo "Khuyến nghị: Sử dụng venv thay vì cài đặt system-wide"
    echo ""
    read -p "Bạn có muốn tiếp tục cài đặt system-wide? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Đã hủy. Sử dụng venv: ./run_viewer.sh"
        exit 0
    fi
fi

# Thử cài với --user flag trước
echo "Thử cài đặt với --user flag..."
if pip3 install --user -r requirements.txt 2>/dev/null; then
    echo "✓ Đã cài đặt tất cả dependencies vào ~/.local"
    echo ""
    echo "Lưu ý: Đảm bảo ~/.local/bin nằm trong PATH"
    exit 0
fi

# Nếu --user không được, thử với --break-system-packages
echo ""
echo "⚠️  Cảnh báo: Hệ thống Python được quản lý bởi OS"
echo "Sẽ cài đặt với --break-system-packages flag"
echo "Điều này có thể ảnh hưởng đến hệ thống Python"
echo ""
read -p "Bạn có muốn tiếp tục? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "Đã hủy. Khuyến nghị sử dụng virtual environment:"
    echo "  ./setup_venv.sh"
    exit 0
fi

# Cài đặt với --break-system-packages
if [ -f "requirements.txt" ]; then
    echo ""
    echo "Đang cài đặt với --break-system-packages..."
    pip3 install --break-system-packages -r requirements.txt
    if [ $? -eq 0 ]; then
        echo ""
        echo "✓ Đã cài đặt tất cả dependencies"
        echo "⚠️  Lưu ý: Packages đã được cài vào system Python"
    else
        echo ""
        echo "✗ Lỗi khi cài đặt dependencies"
        echo ""
        echo "Khuyến nghị: Sử dụng virtual environment"
        echo "  ./setup_venv.sh"
        exit 1
    fi
else
    echo "✗ Không tìm thấy requirements.txt"
    exit 1
fi

