#!/bin/bash

# Script để chạy Theta X QR Code Detection
# Chạy gst_loopback và QR detector cùng lúc

set -e

# Màu sắc cho output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Hàm in thông báo
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Hàm cleanup khi thoát
cleanup() {
    print_info "Đang dừng các tiến trình..."
    
    # Dừng QR detector
    if [ ! -z "$QR_PID" ]; then
        kill $QR_PID 2>/dev/null || true
        print_info "Đã dừng QR detector"
    fi
    
    # Dừng gst_loopback
    if [ ! -z "$GST_PID" ]; then
        kill $GST_PID 2>/dev/null || true
        print_info "Đã dừng gst_loopback"
    fi
    
    # Xóa file tạm
    rm -f /tmp/theta_qr_running
    
    print_success "Đã dọn dẹp xong"
    exit 0
}

# Đăng ký signal handler
trap cleanup SIGINT SIGTERM

# Kiểm tra quyền sudo
if [ "$EUID" -ne 0 ]; then
    print_error "Script này cần chạy với sudo để tạo /dev/video2"
    print_info "Sử dụng: sudo $0"
    exit 1
fi

# Kiểm tra các file cần thiết
if [ ! -f "./gst_loopback" ]; then
    print_error "Không tìm thấy gst_loopback. Hãy build trước!"
    print_info "Chạy: make"
    exit 1
fi

if [ ! -f "./qr_detector.py" ]; then
    print_error "Không tìm thấy qr_detector.py"
    exit 1
fi

# Kiểm tra Python dependencies
print_info "Kiểm tra Python dependencies..."
python3 -c "import cv2, numpy, pyzbar" 2>/dev/null || {
    print_error "Thiếu Python dependencies!"
    print_info "Cài đặt bằng:"
    print_info "  pip3 install opencv-python numpy pyzbar"
    exit 1
}

# Kiểm tra Theta X camera
print_info "Kiểm tra Theta X camera..."
./gst_loopback -l 2>/dev/null | grep -i theta > /dev/null || {
    print_warning "Không tìm thấy Theta X camera. Hãy chắc chắn camera đã kết nối."
    read -p "Tiếp tục? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
}

# Tạo file đánh dấu đang chạy
touch /tmp/theta_qr_running

print_success "Bắt đầu hệ thống Theta X QR Detection..."

# Chạy gst_loopback trong background
print_info "Khởi động gst_loopback..."
sudo ./gst_loopback &
GST_PID=$!

# Đợi một chút để gst_loopback khởi động
sleep 3

# Kiểm tra xem /dev/video2 đã được tạo chưa
if [ ! -e "/dev/video2" ]; then
    print_error "/dev/video2 chưa được tạo. gst_loopback có thể gặp lỗi."
    cleanup
    exit 1
fi

print_success "gst_loopback đã khởi động (PID: $GST_PID)"
print_success "/dev/video2 đã sẵn sàng"

# Chạy QR detector
print_info "Khởi động QR detector..."
python3 qr_detector.py --video /dev/video2 &
QR_PID=$!

print_success "QR detector đã khởi động (PID: $QR_PID)"
print_info "Hệ thống đã sẵn sàng!"
print_info "Nhấn Ctrl+C để dừng"

# Đợi các tiến trình
wait $QR_PID $GST_PID
