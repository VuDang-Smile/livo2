#!/bin/bash

# Script khởi động Enhanced QR Detection
# Tối ưu hóa khả năng quét QR code

set -e

# Màu sắc
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# Cleanup function
cleanup() {
    print_info "Dừng tất cả tiến trình..."
    if [ ! -z "$GST_PID" ]; then
        kill $GST_PID 2>/dev/null || true
    fi
    if [ ! -z "$QR_PID" ]; then
        kill $QR_PID 2>/dev/null || true
    fi
    exit 0
}

trap cleanup SIGINT SIGTERM

print_info "=== Enhanced Theta X QR Detection ==="

# Kiểm tra dependencies
if [ ! -f "./gst_loopback" ]; then
    print_error "Không tìm thấy gst_loopback. Chạy: make"
    exit 1
fi

if [ ! -f "./qr_detector_enhanced.py" ]; then
    print_error "Không tìm thấy qr_detector_enhanced.py"
    exit 1
fi

# Dừng các process cũ
print_info "Dừng các process cũ..."
sudo pkill -f gst_loopback 2>/dev/null || true
sleep 1

# Kiểm tra camera
print_info "Kiểm tra camera Theta X..."
if ! ./gst_loopback -l | grep -i theta > /dev/null; then
    print_error "Không tìm thấy camera Theta X"
    exit 1
fi
print_success "Camera Theta X đã được phát hiện"

# Khởi động gst_loopback
print_info "Khởi động gst_loopback..."
sudo ./gst_loopback &
GST_PID=$!

# Đợi gst_loopback khởi động
sleep 3

# Kiểm tra /dev/video2
if [ ! -e "/dev/video2" ]; then
    print_error "/dev/video2 chưa được tạo"
    cleanup
    exit 1
fi
print_success "gst_loopback đã khởi động thành công"

# Chạy Enhanced QR detector
print_info "Khởi động Enhanced QR detector..."
python3 qr_detector_enhanced.py --video /dev/video2 --qr-interval 2 &
QR_PID=$!

print_success "Enhanced QR detector đã khởi động"
print_info "Tối ưu hóa QR detection:"
print_info "  - Image preprocessing (CLAHE, adaptive threshold)"
print_info "  - Multiple scales detection (0.8x, 1.2x, 1.5x)"
print_info "  - Rotation detection (90°, 180°, 270°)"
print_info "  - Duplicate removal"
print_info "  - Enhanced camera settings (exposure, contrast)"
print_info "  - QR detection mỗi 2 frame"
print_info ""
print_info "Lợi ích:"
print_info "  - Khả năng quét QR code cao hơn đáng kể"
print_info "  - Quét được QR code ở nhiều góc độ"
print_info "  - Quét được QR code ở nhiều kích thước"
print_info "  - Quét được QR code trong điều kiện ánh sáng khó khăn"
print_info ""
print_info "Phím tắt:"
print_info "  - 'q': Thoát"
print_info "  - 's': Chụp ảnh"
print_info "  - 'p': Toggle preprocessing"
print_info ""
print_info "Nhấn Ctrl+C để dừng"

# Đợi các tiến trình
wait $QR_PID $GST_PID
