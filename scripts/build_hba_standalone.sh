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

# Xóa build directory cũ để tránh lỗi CMakeCache.txt với đường dẫn khác
if [ -d "build_standalone" ]; then
    echo "Cleaning old build directory..."
    rm -rf build_standalone
fi

mkdir -p build_standalone
cd build_standalone

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "HBA build successful! Binary located at $BUILD_DIR/hba_standalone"

