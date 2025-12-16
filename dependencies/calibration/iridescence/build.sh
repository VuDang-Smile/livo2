#!/bin/bash

# Build script for Iridescence library
# This script builds and installs Iridescence with Release configuration

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
echo "Build Script for Iridescence"
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
echo "Building Iridescence..."
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
echo "Configuring Iridescence with cmake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build with make
echo ""
echo "Building Iridescence..."
make -j$(nproc)

# Install
echo ""
echo "Installing Iridescence..."
sudo make install

echo ""
echo "=========================================="
echo "✅ Iridescence build and install completed successfully!"
echo "=========================================="