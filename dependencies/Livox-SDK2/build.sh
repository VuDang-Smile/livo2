#!/bin/bash

# Livox-SDK2 Build Script
# This script builds and installs Livox-SDK2 if not already installed
#
# Improvements:
# - Supports local install prefix via $LIVOX_SDK2_PREFIX or --prefix flag
# - Avoids unconditional sudo by installing to a writable prefix by default
# - Clear verification of installed libraries/headers

set -e  # Exit on any error

echo "=== Livox-SDK2 Build Script ==="

# Determine install prefix (priority: --prefix arg > $LIVOX_SDK2_PREFIX > /usr/local)
INSTALL_PREFIX=""
while [ $# -gt 0 ]; do
    case "$1" in
        --prefix)
            shift
            INSTALL_PREFIX="${1:-}"
            shift
            ;;
        *)
            echo "Unknown argument: $1"
            exit 2
            ;;
    esac
done

if [ -z "$INSTALL_PREFIX" ] && [ -n "${LIVOX_SDK2_PREFIX:-}" ]; then
    INSTALL_PREFIX="$LIVOX_SDK2_PREFIX"
fi
if [ -z "$INSTALL_PREFIX" ]; then
    INSTALL_PREFIX="/usr/local"
fi

echo "Install prefix: $INSTALL_PREFIX"

# Check if liblivox_lidar_sdk_* already exists in INSTALL_PREFIX/lib
if ls "$INSTALL_PREFIX"/lib/liblivox_lidar_sdk_* >/dev/null 2>&1; then
    echo "✓ Livox-SDK2 libraries already installed in $INSTALL_PREFIX/lib/"
    echo "  Found: $(ls "$INSTALL_PREFIX"/lib/liblivox_lidar_sdk_* | tr '\n' ' ')"
    echo "  Skipping build and installation."
    echo ""
    echo "✅ STATUS: Livox-SDK2 already installed - No need to rebuild"
    echo "=== Script completed successfully ==="
    echo ""
    read -p "Press Enter to exit..."
    exit 0
fi

echo "Livox-SDK2 libraries not found. Starting build process..."

# Check prerequisites
echo "Checking prerequisites..."

# Check for cmake
if ! command -v cmake &> /dev/null; then
    echo "❌ CMake not found. Installing cmake..."
    sudo apt update
    sudo apt install -y cmake
else
    echo "✓ CMake found: $(cmake --version | head -n1)"
fi

# Check for gcc/g++
if ! command -v g++ &> /dev/null; then
    echo "❌ g++ compiler not found. Installing build-essential..."
    sudo apt update
    sudo apt install -y build-essential
else
    echo "✓ g++ compiler found: $(g++ --version | head -n1)"
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Building from directory: $SCRIPT_DIR"

# Create build directory
BUILD_DIR="$SCRIPT_DIR/build"
echo "Creating build directory: $BUILD_DIR"

if [ -d "$BUILD_DIR" ]; then
    echo "Build directory already exists. Cleaning..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Ensure install prefix exists
mkdir -p "$INSTALL_PREFIX"

# Configure with cmake (respect chosen install prefix)
echo "Configuring with cmake..."
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..

# Build with make
echo "Building with make..."
make -j$(nproc)

# Install (use sudo only if required)
echo "Installing Livox-SDK2..."
SUDO_CMD=""
if [ -w "$INSTALL_PREFIX" ] || [ -w "$(dirname "$INSTALL_PREFIX")" ]; then
    SUDO_CMD=""
else
    SUDO_CMD="sudo"
fi
$SUDO_CMD make install

# Verify installation
echo ""
echo "=========================================="
echo "Verifying installation..."
echo "=========================================="

VERIFY_SUCCESS=true
VERIFY_ERRORS=""

# Check libraries
if ls "$INSTALL_PREFIX"/lib/liblivox_lidar_sdk_* >/dev/null 2>&1; then
    echo "✓ Libraries: Found"
    echo "  Location: $INSTALL_PREFIX/lib/"
    echo "  Files: $(ls "$INSTALL_PREFIX"/lib/liblivox_lidar_sdk_* | tr '\n' ' ')"
else
    echo "❌ Libraries: NOT found in $INSTALL_PREFIX/lib/"
    VERIFY_SUCCESS=false
    VERIFY_ERRORS="${VERIFY_ERRORS}- Libraries do not exist\n"
fi

# Check headers
if ls "$INSTALL_PREFIX"/include/livox_lidar_* >/dev/null 2>&1; then
    echo "✓ Headers: Found"
    echo "  Location: $INSTALL_PREFIX/include/"
    echo "  Files: $(ls "$INSTALL_PREFIX"/include/livox_lidar_* | tr '\n' ' ')"
else
    echo "⚠️  Headers: Not found in $INSTALL_PREFIX/include/"
    VERIFY_ERRORS="${VERIFY_ERRORS}- Headers do not exist (may not be critical)\n"
fi

echo ""
echo "=========================================="
if [ "$VERIFY_SUCCESS" = true ]; then
    echo "✅ VERIFY RESULT: SUCCESS"
    echo "✅ STATUS: Build and installation of Livox-SDK2 completed successfully!"
    echo "=========================================="
    echo ""
    echo "Livox-SDK2 is ready to use at: $INSTALL_PREFIX"
    echo "=== Build completed successfully ==="
    echo ""
    read -p "Press Enter to exit..."
    exit 0
else
    echo "❌ VERIFY RESULT: FAILED"
    echo "❌ STATUS: Build and installation of Livox-SDK2 failed!"
    echo "=========================================="
    echo ""
    echo "Error details:"
    echo -e "$VERIFY_ERRORS"
    echo "Please check the build and installation process."
    echo "=== Build failed ==="
    echo ""
    read -p "Press Enter to exit..."
    exit 1
fi
