#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
HBA_DIR="$PROJECT_ROOT/dependencies/HBA"
BUILD_DIR="$HBA_DIR/build_standalone"

echo "Building HBA as a standalone tool..."

cd "$HBA_DIR"

if [ -f "CMakeLists.txt" ] && [ ! -f "CMakeLists.txt.ros1" ]; then
    echo "Backing up original ROS1 CMakeLists.txt"
    mv CMakeLists.txt CMakeLists.txt.ros1
fi

cp CMakeLists.standalone.txt CMakeLists.txt

mkdir -p build_standalone
cd build_standalone

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo "HBA build successful! Binary located at $BUILD_DIR/hba_standalone"

