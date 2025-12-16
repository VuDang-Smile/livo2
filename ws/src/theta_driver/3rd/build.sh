#!/bin/bash

# Build script for libuvc-theta and libuvc-theta-sample
# This script builds both libuvc-theta library and libuvc-theta-sample applications

set -e  # Exit on any error

echo "=== Build Script for libuvc-theta and libuvc-theta-sample ==="
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory: $SCRIPT_DIR"

# Define paths
LIBUVC_THETA_DIR="${SCRIPT_DIR}/libuvc-theta"
LIBUVC_THETA_SAMPLE_DIR="${SCRIPT_DIR}/libuvc-theta-sample"
LIBUVC_THETA_SAMPLE_GST_DIR="${LIBUVC_THETA_SAMPLE_DIR}/gst"

# Determine install prefix (priority: --prefix arg > $LIBUVC_THETA_PREFIX > /usr/local)
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
            echo ""
            read -p "Press Enter to exit..."
            exit 2
            ;;
    esac
done

if [ -z "$INSTALL_PREFIX" ] && [ -n "${LIBUVC_THETA_PREFIX:-}" ]; then
    INSTALL_PREFIX="$LIBUVC_THETA_PREFIX"
fi
if [ -z "$INSTALL_PREFIX" ]; then
    INSTALL_PREFIX="/usr/local"
fi

echo "Install prefix: $INSTALL_PREFIX"

# Check if directories exist
if [ ! -d "$LIBUVC_THETA_DIR" ]; then
    echo "Error: libuvc-theta directory not found at: $LIBUVC_THETA_DIR"
    echo ""
    read -p "Press Enter to exit..."
    exit 1
fi

if [ ! -d "$LIBUVC_THETA_SAMPLE_DIR" ]; then
    echo "Error: libuvc-theta-sample directory not found at: $LIBUVC_THETA_SAMPLE_DIR"
    echo ""
    read -p "Press Enter to exit..."
    exit 1
fi

# Check prerequisites
echo "Checking prerequisites..."

# Check for cmake
if ! command -v cmake &> /dev/null; then
    echo "Error: CMake not found. Please install cmake first."
    echo "  sudo apt update && sudo apt install -y cmake"
    echo ""
    read -p "Press Enter to exit..."
    exit 1
else
    echo "✓ CMake found: $(cmake --version | head -n1)"
fi

# Check for make
if ! command -v make &> /dev/null; then
    echo "Error: Make not found. Please install build-essential first."
    echo "  sudo apt update && sudo apt install -y build-essential"
    echo ""
    read -p "Press Enter to exit..."
    exit 1
else
    echo "✓ Make found: $(make --version | head -n1)"
fi

# Check for pkg-config (needed for libuvc-theta-sample)
if ! command -v pkg-config &> /dev/null; then
    echo "Error: pkg-config not found. Please install pkg-config first."
    echo "  sudo apt update && sudo apt install -y pkg-config"
    echo ""
    read -p "Press Enter to exit..."
    exit 1
else
    echo "✓ pkg-config found: $(pkg-config --version)"
fi

# Check for libusb (required by libuvc-theta)
if ! pkg-config --exists libusb-1.0 2>/dev/null; then
    echo "Warning: libusb-1.0 not found via pkg-config."
    echo "  Please install: sudo apt install -y libusb-1.0-0-dev"
    echo "  Continuing anyway..."
else
    echo "✓ libusb-1.0 found"
fi

echo ""
echo "=========================================="
echo "Building libuvc-theta..."
echo "=========================================="

# Build libuvc-theta
cd "$LIBUVC_THETA_DIR"

# Create build directory
BUILD_DIR="$LIBUVC_THETA_DIR/build"
if [ -d "$BUILD_DIR" ]; then
    echo "Build directory already exists. Cleaning..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure with cmake
echo "Configuring libuvc-theta with cmake..."
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" ..

# Build with make
echo "Building libuvc-theta..."
make -j$(nproc)

# Install libuvc-theta (needed for libuvc-theta-sample to find it via pkg-config)
echo "Installing libuvc-theta..."
echo "Install prefix: $INSTALL_PREFIX"

# Determine if sudo is needed
SUDO_CMD=""
NEED_SUDO=false

# Check if install prefix exists and is writable
if [ -d "$INSTALL_PREFIX" ]; then
    if [ ! -w "$INSTALL_PREFIX" ]; then
        NEED_SUDO=true
    fi
elif [ -d "$(dirname "$INSTALL_PREFIX")" ]; then
    if [ ! -w "$(dirname "$INSTALL_PREFIX")" ]; then
        NEED_SUDO=true
    fi
else
    # If parent directory doesn't exist, check if we can create it
    # For system directories like /usr/local, we'll need sudo
    if [[ "$INSTALL_PREFIX" == /usr* ]] || [[ "$INSTALL_PREFIX" == /opt* ]]; then
        NEED_SUDO=true
    fi
fi

if [ "$NEED_SUDO" = true ]; then
    SUDO_CMD="sudo"
    echo "Using sudo for installation (install prefix requires root privileges)"
    # Create directory with sudo if needed
    if [ ! -d "$INSTALL_PREFIX" ]; then
        sudo mkdir -p "$INSTALL_PREFIX"
    fi
else
    echo "Installing without sudo (install prefix is writable)"
    mkdir -p "$INSTALL_PREFIX"
fi

echo "Running: $SUDO_CMD make install"
$SUDO_CMD make install

echo "✓ libuvc-theta build and install completed successfully!"
echo ""

# Build libuvc-theta-sample
echo "=========================================="
echo "Building libuvc-theta-sample..."
echo "=========================================="

# Check if gst directory exists
if [ ! -d "$LIBUVC_THETA_SAMPLE_GST_DIR" ]; then
    echo "Warning: gst directory not found at: $LIBUVC_THETA_SAMPLE_GST_DIR"
    echo "Skipping libuvc-theta-sample build..."
else
    cd "$LIBUVC_THETA_SAMPLE_GST_DIR"
    
    # Check if Makefile exists
    if [ ! -f "Makefile" ]; then
        echo "Warning: Makefile not found in gst directory. Skipping build..."
    else
        # Set PKG_CONFIG_PATH to find libuvc.pc from install prefix
        export PKG_CONFIG_PATH="${INSTALL_PREFIX}/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
        export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
        
        # Check for gstreamer dependencies (required)
        echo "Checking dependencies for libuvc-theta-sample..."
        if ! pkg-config --exists gstreamer-app-1.0 2>/dev/null; then
            echo "Error: gstreamer-app-1.0 not found via pkg-config."
            echo "  Please install: sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev"
            echo ""
            read -p "Press Enter to exit..."
            exit 1
        else
            echo "✓ gstreamer-app-1.0 found"
        fi
        
        # Check if libuvc can be found via pkg-config (required)
        if ! pkg-config --exists libuvc 2>/dev/null; then
            echo "Error: libuvc not found via pkg-config."
            echo "  PKG_CONFIG_PATH: $PKG_CONFIG_PATH"
            echo "  This usually means libuvc-theta was not installed correctly."
            echo "  Please check the libuvc-theta installation above."
            echo ""
            read -p "Press Enter to exit..."
            exit 1
        else
            echo "✓ libuvc found via pkg-config"
        fi
        
        # Clean previous build
        echo "Cleaning previous build..."
        make clean 2>/dev/null || true
        
        # Build (temporarily disable set -e to catch build errors gracefully)
        echo "Building libuvc-theta-sample..."
        set +e
        make -j$(nproc)
        BUILD_EXIT_CODE=$?
        set -e
        
        if [ $BUILD_EXIT_CODE -ne 0 ]; then
            echo ""
            echo "Error: Build failed with exit code $BUILD_EXIT_CODE"
            echo "Please check the error messages above."
            echo ""
            read -p "Press Enter to exit..."
            exit 1
        fi
        
        echo "✓ libuvc-theta-sample build completed successfully!"
    fi
fi

echo ""
echo "=========================================="
echo "Verifying build results..."
echo "=========================================="

VERIFY_SUCCESS=true
VERIFY_ERRORS=""

# Verify libuvc-theta build
if [ -f "$INSTALL_PREFIX/lib/libuvc.so" ] || [ -f "$INSTALL_PREFIX/lib/libuvc.a" ]; then
    echo "✓ libuvc-theta: Libraries found (installed)"
    echo "  Location: $INSTALL_PREFIX/lib/"
    if [ -f "$INSTALL_PREFIX/lib/libuvc.so" ]; then
        echo "  - libuvc.so (shared library)"
    fi
    if [ -f "$INSTALL_PREFIX/lib/libuvc.a" ]; then
        echo "  - libuvc.a (static library)"
    fi
elif [ -f "$BUILD_DIR/libuvc.so" ] || [ -f "$BUILD_DIR/libuvc.a" ]; then
    echo "✓ libuvc-theta: Libraries found (build directory)"
    echo "  Location: $BUILD_DIR/"
    if [ -f "$BUILD_DIR/libuvc.so" ]; then
        echo "  - libuvc.so (shared library)"
    fi
    if [ -f "$BUILD_DIR/libuvc.a" ]; then
        echo "  - libuvc.a (static library)"
    fi
else
    echo "❌ libuvc-theta: Libraries NOT found"
    VERIFY_SUCCESS=false
    VERIFY_ERRORS="${VERIFY_ERRORS}- libuvc-theta libraries do not exist\n"
fi

# Verify libuvc.pc file
if [ -f "$INSTALL_PREFIX/lib/pkgconfig/libuvc.pc" ]; then
    echo "✓ libuvc-theta: pkg-config file found"
    echo "  Location: $INSTALL_PREFIX/lib/pkgconfig/libuvc.pc"
else
    echo "⚠️  libuvc-theta: pkg-config file not found (may affect libuvc-theta-sample build)"
fi

# Verify libuvc-theta-sample build
if [ -f "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_viewer" ]; then
    echo "✓ libuvc-theta-sample: gst_viewer executable found"
    echo "  Location: $LIBUVC_THETA_SAMPLE_GST_DIR/gst_viewer"
    if [ -L "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback" ] || [ -f "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback" ]; then
        echo "✓ libuvc-theta-sample: gst_loopback found"
        echo "  Location: $LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback"
    fi
else
    echo "⚠️  libuvc-theta-sample: gst_viewer executable not found (may not be critical)"
    VERIFY_ERRORS="${VERIFY_ERRORS}- gst_viewer executable does not exist\n"
fi

echo ""
echo "=========================================="
if [ "$VERIFY_SUCCESS" = true ]; then
    echo "✅ BUILD RESULT: SUCCESS"
    echo "✅ STATUS: Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "Build outputs:"
    echo "  - libuvc-theta (installed): $INSTALL_PREFIX/"
    echo "  - libuvc-theta (build): $BUILD_DIR/"
    if [ -f "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_viewer" ]; then
        echo "  - libuvc-theta-sample executables:"
        echo "    * gst_viewer: $LIBUVC_THETA_SAMPLE_GST_DIR/gst_viewer"
        if [ -L "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback" ] || [ -f "$LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback" ]; then
            echo "    * gst_loopback: $LIBUVC_THETA_SAMPLE_GST_DIR/gst_loopback"
        fi
    fi
    echo ""
    # Only show environment variable notes if installing to non-standard location
    if [[ "$INSTALL_PREFIX" != "/usr/local" ]] && [[ "$INSTALL_PREFIX" != "/usr" ]]; then
        echo "Note: Since libuvc-theta is installed to a non-standard location, you may need to set:"
        echo "  export LD_LIBRARY_PATH=\"${INSTALL_PREFIX}/lib:\$LD_LIBRARY_PATH\""
        echo "  export PKG_CONFIG_PATH=\"${INSTALL_PREFIX}/lib/pkgconfig:\$PKG_CONFIG_PATH\""
        echo ""
    fi
    echo "=== Build completed successfully ==="
    echo ""
    read -p "Press Enter to exit..."
    exit 0
else
    echo "❌ BUILD RESULT: FAILED"
    echo "❌ STATUS: Build completed with errors!"
    echo "=========================================="
    echo ""
    echo "Error details:"
    echo -e "$VERIFY_ERRORS"
    echo "Please check the build process above."
    echo "=== Build failed ==="
    echo ""
    read -p "Press Enter to exit..."
    exit 1
fi
