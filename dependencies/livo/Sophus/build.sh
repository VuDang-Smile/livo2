#!/bin/bash

# Build script for Sophus library
# This script builds and installs Sophus library

# Function to display "press enter to exit" when script ends
cleanup() {
    echo ""
    echo "=========================================="
    read -p "Press Enter to exit..."
}

# Ensure cleanup is always called when script ends (success or error)
trap cleanup EXIT

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "Build Script for Sophus Library"
echo "=========================================="
echo ""
echo "Script directory: $SCRIPT_DIR"
echo ""

# Change to Sophus directory
cd "$SCRIPT_DIR"

# Check if build directory exists, remove it if it does
if [ -d "build" ]; then
    echo "Removing existing build directory..."
    rm -rf build
fi

# Create build directory and change to it
echo "Creating build directory..."
mkdir build && cd build

# Run cmake
echo ""
echo "=========================================="
echo "Running CMake configuration..."
echo "=========================================="
cmake ..

# Build with make
echo ""
echo "=========================================="
echo "Building Sophus library..."
echo "=========================================="
make

# Install with sudo make install
echo ""
echo "=========================================="
echo "Installing Sophus library..."
echo "=========================================="
sudo make install

echo ""
echo "=========================================="
echo "SUCCESS: Sophus library has been built and installed!"
echo "=========================================="
echo ""
