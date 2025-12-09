#!/bin/bash
# Script để cài đặt Sophus library system-wide

set -e  # Exit on error

echo "=== Cài đặt Sophus Library ==="
echo ""

# Kiểm tra quyền sudo
if [ "$EUID" -ne 0 ] && ! sudo -n true 2>/dev/null; then
    echo "Script này cần quyền sudo để cài đặt Sophus"
    echo "Vui lòng chạy với sudo hoặc nhập mật khẩu khi được yêu cầu"
    echo ""
fi

# Kiểm tra xem đã cài đặt chưa
if [ -f "/usr/local/lib/cmake/Sophus/SophusConfig.cmake" ] || [ -f "/usr/local/lib64/cmake/Sophus/SophusConfig.cmake" ]; then
    echo "⚠️  Sophus đã được cài đặt trong hệ thống"
    read -p "Bạn có muốn cài đặt lại? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Bỏ qua cài đặt Sophus"
        exit 0
    fi
fi

# Tạo thư mục tạm
TMP_DIR="/tmp/sophus_install_$$"
mkdir -p "$TMP_DIR"
cd "$TMP_DIR"

echo "Đang clone Sophus repository..."
git clone https://github.com/strasdat/Sophus.git sophus_install

cd sophus_install
echo "Đang checkout commit a621ff..."
git checkout a621ff

# Sửa lỗi so2.cpp nếu cần
echo "Đang sửa lỗi so2.cpp..."
sed -i 's/unit_complex_\.real() = 1\.;/unit_complex_.real(1.);/' sophus/so2.cpp
sed -i 's/unit_complex_\.imag() = 0\.;/unit_complex_.imag(0.);/' sophus/so2.cpp

# Build
echo "Đang build Sophus..."
mkdir -p build
cd build
cmake ..
make -j$(nproc)

# Install
echo "Đang cài đặt Sophus vào system..."
sudo make install

# Update library cache
echo "Đang cập nhật library cache..."
sudo ldconfig

# Cleanup
cd /
rm -rf "$TMP_DIR"

echo ""
echo "✓ Đã cài đặt Sophus thành công!"
echo ""
echo "Kiểm tra cài đặt:"
if [ -f "/usr/local/lib/cmake/Sophus/SophusConfig.cmake" ] || [ -f "/usr/local/lib64/cmake/Sophus/SophusConfig.cmake" ]; then
    echo "  ✓ SophusConfig.cmake found"
else
    echo "  ⚠️  SophusConfig.cmake not found, có thể cần kiểm tra lại"
fi

echo ""
echo "Bây giờ bạn có thể build các package trong ws/ mà không gặp lỗi Sophus nữa."

