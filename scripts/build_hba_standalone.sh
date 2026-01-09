#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Tìm thư mục HBA thay vì dùng hardcoded path
echo "Searching for HBA directory..."
HBA_DIR=$(find "$PROJECT_ROOT" -type d -name "HBA" -not -path "*/build*" -not -path "*/.*" | head -n 1)

if [ -z "$HBA_DIR" ]; then
    echo "Error: Could not find HBA directory in $PROJECT_ROOT"
    exit 1
fi

echo "Found HBA at: $HBA_DIR"
BUILD_DIR="$HBA_DIR/build_standalone"

echo "Building HBA as a standalone tool..."

cd "$HBA_DIR"

if [ -f "CMakeLists.txt" ] && [ ! -f "CMakeLists.txt.ros1" ]; then
    echo "Backing up original ROS1 CMakeLists.txt"
    mv CMakeLists.txt CMakeLists.txt.ros1
fi

cp CMakeLists.standalone.txt CMakeLists.txt

# Tạo build directory nếu chưa có
mkdir -p build_standalone
cd build_standalone

# Kiểm tra CMakeCache.txt có đường dẫn khác không
NEED_CLEAN=false
if [ -f "CMakeCache.txt" ]; then
    # Đọc đường dẫn source từ CMakeCache.txt
    CACHED_SOURCE=$(grep "^CMAKE_SOURCE_DIR:" CMakeCache.txt 2>/dev/null | cut -d "=" -f2 | tr -d ' ')
    CURRENT_SOURCE=$(cd .. && pwd)
    
    if [ -n "$CACHED_SOURCE" ] && [ "$CACHED_SOURCE" != "$CURRENT_SOURCE" ]; then
        echo "Warning: CMakeCache.txt has different source path:"
        echo "  Cached: $CACHED_SOURCE"
        echo "  Current: $CURRENT_SOURCE"
        echo "Cleaning CMakeCache.txt to avoid path mismatch..."
        rm -f CMakeCache.txt
        NEED_CLEAN=true
    fi
fi

# Chạy cmake (sẽ tự động tạo CMakeCache.txt nếu chưa có hoặc đã bị xóa)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

echo "HBA build successful! Binary located at $BUILD_DIR/hba_standalone"

