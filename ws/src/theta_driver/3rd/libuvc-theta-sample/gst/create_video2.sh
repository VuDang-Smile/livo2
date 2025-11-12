#!/bin/bash

# Script tạo /dev/video2 cho Theta loopback streaming
# Giải quyết vấn đề module v4l2loopback

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

# Kiểm tra quyền sudo
if [ "$EUID" -ne 0 ]; then
    print_error "Script này cần chạy với sudo"
    print_info "Sử dụng: sudo $0"
    exit 1
fi

print_info "=== Tạo /dev/video2 cho Theta Loopback ==="

# Dừng tất cả process liên quan
print_info "Dừng các process liên quan..."
pkill -f gst_loopback 2>/dev/null || true
pkill -f qr_detector 2>/dev/null || true
sleep 2

# Kiểm tra xem /dev/video2 đã tồn tại chưa
if [ -e "/dev/video2" ]; then
    print_info "/dev/video2 đã tồn tại, kiểm tra quyền..."
    chmod 666 /dev/video2
    print_success "/dev/video2 đã sẵn sàng"
    exit 0
fi

# Unload module cũ
print_info "Unload v4l2loopback module cũ..."
modprobe -r v4l2loopback 2>/dev/null || true
sleep 1

# Load module mới với cấu hình đúng
print_info "Load v4l2loopback module với cấu hình mới..."
modprobe v4l2loopback devices=3 video_nr=0,1,2 card_label="Theta Loopback" exclusive_caps=1

sleep 3

# Kiểm tra kết quả
if [ -e "/dev/video2" ]; then
    print_success "/dev/video2 đã được tạo thành công!"
    
    # Thiết lập quyền
    chmod 666 /dev/video2
    print_success "Đã thiết lập quyền cho /dev/video2"
    
    # Hiển thị thông tin
    print_info "Thông tin /dev/video2:"
    v4l2-ctl --device=/dev/video2 --info 2>/dev/null || print_warning "Không thể lấy thông tin device"
    
    print_success "=== Hoàn tất ==="
    print_info "/dev/video2 đã sẵn sàng cho loopback streaming"
    print_info "Bây giờ bạn có thể chạy:"
    print_info "  sudo ./gst_loopback"
    print_info "  hoặc"
    print_info "  sudo ./start_enhanced.sh"
    
else
    print_error "Không thể tạo /dev/video2"
    print_info "Thử các bước sau:"
    print_info "1. Kiểm tra v4l2loopback module: modinfo v4l2loopback"
    print_info "2. Cài đặt v4l2loopback: sudo apt install v4l2loopback-dkms"
    print_info "3. Reboot hệ thống"
    exit 1
fi


