#!/bin/bash

# Build script for Ceres Solver library
# This script builds and installs Ceres Solver with optimized configuration

# Function to display "press enter to exit" when script ends
cleanup() {
    echo ""
    echo "=========================================="
    read -p "Press Enter to exit..."
}

# Ensure cleanup is always called when script ends (success or error)
trap cleanup EXIT

set -e  # Exit on any error

echo "=========================================="
echo "Build Script for Ceres Solver"
echo "=========================================="
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory: $SCRIPT_DIR"

# Check prerequisites
echo "Checking prerequisites..."

# Check for cmake
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake not found. Please install cmake first."
    echo "  sudo apt update && sudo apt install -y cmake"
    exit 1
else
    echo "✓ CMake found: $(cmake --version | head -n1)"
fi

# Check for make
if ! command -v make &> /dev/null; then
    echo "Error: Make not found. Please install build-essential first."
    echo "  sudo apt update && sudo apt install -y build-essential"
    exit 1
else
    echo "✓ Make found: $(make --version | head -n1)"
fi

echo ""
echo "=========================================="
echo "Building Ceres Solver..."
echo "=========================================="

# Navigate to script directory
cd "$SCRIPT_DIR"

# Create build directory
BUILD_DIR="$SCRIPT_DIR/build"
if [ -d "$BUILD_DIR" ]; then
    echo "Build directory already exists. Cleaning..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with cmake
echo ""
echo "Configuring Ceres Solver with cmake..."
cmake .. -DBUILD_EXAMPLES=OFF \
         -DBUILD_TESTING=OFF \
         -DUSE_CUDA=OFF

# Build with make
echo ""
echo "Building Ceres Solver..."
make -j$(nproc)

# Install
echo ""
echo "Installing Ceres Solver..."
sudo make install

echo ""
echo "=========================================="
echo "✅ Ceres Solver build and install completed successfully!"
echo "=========================================="
