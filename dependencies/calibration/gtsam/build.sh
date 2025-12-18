#!/bin/bash

# Build script for GTSAM library
# This script builds and installs GTSAM with optimized configuration

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
echo "Build Script for GTSAM"
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
echo "Building GTSAM..."
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

# Detect Ubuntu version for system Eigen option
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || echo "")
USE_SYSTEM_EIGEN=""

# Check if system Eigen is available
if [ -f "/usr/include/eigen3/Eigen/Core" ] || [ -f "/usr/local/include/eigen3/Eigen/Core" ]; then
    SYSTEM_EIGEN_AVAILABLE=true
else
    SYSTEM_EIGEN_AVAILABLE=false
fi

if [ -n "$UBUNTU_VERSION" ]; then
    echo "Detected Ubuntu version: $UBUNTU_VERSION"
    
    # For Ubuntu 22.04 and newer, use system Eigen to ensure compatibility
    # This prevents version mismatch between GTSAM and other packages
    UBUNTU_MAJOR=$(echo "$UBUNTU_VERSION" | cut -d. -f1)
    UBUNTU_MINOR=$(echo "$UBUNTU_VERSION" | cut -d. -f2)
    
    if [ "$UBUNTU_MAJOR" -ge 22 ]; then
        if [ "$SYSTEM_EIGEN_AVAILABLE" = true ]; then
            USE_SYSTEM_EIGEN="-DGTSAM_USE_SYSTEM_EIGEN=ON"
            echo "Using system Eigen for Ubuntu $UBUNTU_VERSION (recommended for compatibility)"
        else
            echo "Warning: Ubuntu $UBUNTU_VERSION detected but system Eigen not found"
            echo "GTSAM will use bundled Eigen, which may cause version conflicts"
        fi
    else
        echo "Ubuntu version $UBUNTU_VERSION detected, using bundled Eigen"
    fi
else
    # If Ubuntu version cannot be detected, check if system Eigen is available
    if [ "$SYSTEM_EIGEN_AVAILABLE" = true ]; then
        echo "Could not detect Ubuntu version, but system Eigen is available"
        echo "Using system Eigen to ensure compatibility"
        USE_SYSTEM_EIGEN="-DGTSAM_USE_SYSTEM_EIGEN=ON"
    else
        echo "Could not detect Ubuntu version, proceeding without system Eigen option"
    fi
fi

# Configure with cmake
echo ""
echo "Configuring GTSAM with cmake..."
CMAKE_ARGS=(
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
    -DGTSAM_BUILD_TESTS=OFF
    -DGTSAM_WITH_TBB=OFF
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
)

if [ -n "$USE_SYSTEM_EIGEN" ]; then
    CMAKE_ARGS+=("$USE_SYSTEM_EIGEN")
fi

cmake .. "${CMAKE_ARGS[@]}"

# Build with make
echo ""
echo "Building GTSAM..."
make -j$(nproc)

# Install
echo ""
echo "Installing GTSAM..."
sudo make install

echo ""
echo "=========================================="
echo "✅ GTSAM build and install completed successfully!"
echo "=========================================="
