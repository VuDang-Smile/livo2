#!/bin/bash
# Script để cài đặt tất cả dependencies cho project

set -e  # Exit on error

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=== Cài đặt Dependencies cho Livo Project ==="
echo ""

# Kiểm tra quyền sudo
if [ "$EUID" -ne 0 ] && ! sudo -n true 2>/dev/null; then
    echo "Script này cần quyền sudo để cài đặt system packages"
    echo "Vui lòng chạy với sudo hoặc nhập mật khẩu khi được yêu cầu"
    echo ""
fi

# ============================================
# 1. Cài đặt system dependencies (apt packages)
# ============================================
echo "=== Bước 1: Cài đặt system dependencies ==="
echo "Đang cài đặt: libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev"
echo ""

sudo apt update
sudo apt install -y \
    libomp-dev \
    libboost-all-dev \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    build-essential \
    cmake \
    git

if [ $? -eq 0 ]; then
    echo "✓ Đã cài đặt system dependencies"
else
    echo "✗ Lỗi khi cài đặt system dependencies"
    exit 1
fi

echo ""

# ============================================
# 2. Cài đặt GTSAM
# ============================================
echo "=== Bước 2: Cài đặt GTSAM ==="
echo ""

GTSAM_DIR="$SCRIPT_DIR/deps/gtsam"
GTSAM_BUILD_DIR="$GTSAM_DIR/build"

# Kiểm tra xem đã cài đặt chưa
if [ -f "/usr/local/lib/libgtsam.so" ] || [ -f "/usr/local/lib64/libgtsam.so" ]; then
    echo "⚠️  GTSAM đã được cài đặt trong hệ thống"
    read -p "Bạn có muốn cài đặt lại? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Bỏ qua cài đặt GTSAM"
        SKIP_GTSAM=true
    fi
fi

if [ "$SKIP_GTSAM" != "true" ]; then
    # Clone hoặc update GTSAM
    if [ ! -d "$GTSAM_DIR" ]; then
        echo "Đang clone GTSAM..."
        mkdir -p "$(dirname "$GTSAM_DIR")"
        git clone https://github.com/borglab/gtsam "$GTSAM_DIR"
    else
        echo "GTSAM đã được clone, đang update..."
        cd "$GTSAM_DIR"
        git fetch
        git checkout 4.2a9
        cd "$SCRIPT_DIR"
    fi

    cd "$GTSAM_DIR"
    git checkout 4.2a9

    # Build GTSAM
    echo "Đang build GTSAM..."
    mkdir -p build
    cd build

    cmake .. \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON

    make -j$(nproc)
    sudo make install

    if [ $? -eq 0 ]; then
        echo "✓ Đã cài đặt GTSAM"
    else
        echo "✗ Lỗi khi cài đặt GTSAM"
        exit 1
    fi

    cd "$SCRIPT_DIR"
fi

echo ""

# ============================================
# 3. Cài đặt Ceres Solver
# ============================================
echo "=== Bước 3: Cài đặt Ceres Solver ==="
echo ""

CERES_DIR="$SCRIPT_DIR/deps/ceres-solver"
CERES_BUILD_DIR="$CERES_DIR/build"

# Kiểm tra xem đã cài đặt chưa
if [ -f "/usr/local/lib/libceres.so" ] || [ -f "/usr/local/lib64/libceres.so" ]; then
    echo "⚠️  Ceres Solver đã được cài đặt trong hệ thống"
    read -p "Bạn có muốn cài đặt lại? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Bỏ qua cài đặt Ceres Solver"
        SKIP_CERES=true
    fi
fi

if [ "$SKIP_CERES" != "true" ]; then
    # Clone hoặc update Ceres
    if [ ! -d "$CERES_DIR" ]; then
        echo "Đang clone Ceres Solver..."
        mkdir -p "$(dirname "$CERES_DIR")"
        git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver "$CERES_DIR"
    else
        echo "Ceres Solver đã được clone, đang update..."
        cd "$CERES_DIR"
        git fetch
        git checkout e47a42c2957951c9fafcca9995d9927e15557069
        git submodule update --init --recursive
        cd "$SCRIPT_DIR"
    fi

    cd "$CERES_DIR"
    git checkout e47a42c2957951c9fafcca9995d9927e15557069
    git submodule update --init --recursive

    # Build Ceres
    echo "Đang build Ceres Solver..."
    mkdir -p build
    cd build

    cmake .. \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
        -DUSE_CUDA=OFF

    make -j$(nproc)
    sudo make install

    if [ $? -eq 0 ]; then
        echo "✓ Đã cài đặt Ceres Solver"
    else
        echo "✗ Lỗi khi cài đặt Ceres Solver"
        exit 1
    fi

    cd "$SCRIPT_DIR"
fi

echo ""

# ============================================
# 4. Cài đặt Iridescence
# ============================================
echo "=== Bước 4: Cài đặt Iridescence ==="
echo ""

IRIDESCENCE_DIR="$SCRIPT_DIR/deps/iridescence"
IRIDESCENCE_BUILD_DIR="$IRIDESCENCE_DIR/build"

# Kiểm tra xem đã cài đặt chưa
if [ -f "/usr/local/lib/libiridescence.so" ] || [ -f "/usr/local/lib64/libiridescence.so" ]; then
    echo "⚠️  Iridescence đã được cài đặt trong hệ thống"
    read -p "Bạn có muốn cài đặt lại? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Bỏ qua cài đặt Iridescence"
        SKIP_IRIDESCENCE=true
    fi
fi

if [ "$SKIP_IRIDESCENCE" != "true" ]; then
    # Clone hoặc update Iridescence
    if [ ! -d "$IRIDESCENCE_DIR" ]; then
        echo "Đang clone Iridescence..."
        mkdir -p "$(dirname "$IRIDESCENCE_DIR")"
        git clone --recursive https://github.com/koide3/iridescence "$IRIDESCENCE_DIR"
    else
        echo "Iridescence đã được clone, đang update..."
        cd "$IRIDESCENCE_DIR"
        git fetch
        git pull
        git submodule update --init --recursive
        cd "$SCRIPT_DIR"
    fi

    cd "$IRIDESCENCE_DIR"

    # Build Iridescence
    echo "Đang build Iridescence..."
    mkdir -p build
    cd build

    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install

    if [ $? -eq 0 ]; then
        echo "✓ Đã cài đặt Iridescence"
    else
        echo "✗ Lỗi khi cài đặt Iridescence"
        exit 1
    fi

    cd "$SCRIPT_DIR"
fi

echo ""

# ============================================
# 5. Cài đặt Python dependencies
# ============================================
echo "=== Bước 5: Cài đặt Python dependencies ==="
echo ""

REQUIREMENTS_FILE="$SCRIPT_DIR/gui/requirements.txt"
if [ -f "$REQUIREMENTS_FILE" ]; then
    # Thử cài với --user flag trước
    echo "Thử cài đặt với --user flag..."
    if pip3 install --user -r "$REQUIREMENTS_FILE" 2>/dev/null; then
        echo "✓ Đã cài đặt tất cả Python dependencies vào ~/.local"
        echo ""
        echo "Lưu ý: Đảm bảo ~/.local/bin nằm trong PATH"
    else
        # Nếu --user không được, thử với --break-system-packages
        echo ""
        echo "⚠️  Cảnh báo: Hệ thống Python được quản lý bởi OS"
        echo "Sẽ cài đặt với --break-system-packages flag"
        echo ""
        read -p "Bạn có muốn tiếp tục? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pip3 install --break-system-packages -r "$REQUIREMENTS_FILE"
            if [ $? -eq 0 ]; then
                echo ""
                echo "✓ Đã cài đặt tất cả Python dependencies"
            else
                echo ""
                echo "✗ Lỗi khi cài đặt Python dependencies"
                exit 1
            fi
        else
            echo "Bỏ qua cài đặt Python dependencies"
        fi
    fi
else
    echo "⚠️  Không tìm thấy requirements.txt tại $REQUIREMENTS_FILE, bỏ qua cài đặt Python dependencies"
fi

echo ""
echo "=== Cài đặt hoàn tất ==="
echo ""
echo "Các thư viện đã được cài đặt:"
echo "  - System dependencies (apt packages)"
echo "  - GTSAM"
echo "  - Ceres Solver"
echo "  - Iridescence"
echo "  - Python dependencies"
echo ""
echo "Lưu ý: Nếu gặp lỗi khi link libraries, có thể cần chạy:"
echo "  sudo ldconfig"

