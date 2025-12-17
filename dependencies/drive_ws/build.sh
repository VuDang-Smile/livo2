#!/bin/bash

# Auto build script for livox_ros_driver2 with jazzy and check source setup.bash

# Function to display "press enter to exit" when script ends
cleanup() {
    echo ""
    echo "=========================================="
    read -p "Press Enter to exit..."
}

# Ensure cleanup is always called when script ends (success or error)
trap cleanup EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRIVE_WS_DIR="${SCRIPT_DIR}"
BUILD_SCRIPT="${DRIVE_WS_DIR}/src/livox_ros_driver2/build.sh"
SETUP_BASH="${DRIVE_WS_DIR}/install/setup.bash"
BASHRC="${HOME}/.bashrc"
ROS2_SETUP="/opt/ros/jazzy/setup.bash"

echo "=========================================="
echo "Build script for livox_ros_driver2"
echo "=========================================="

# Source ROS2 Jazzy base setup
echo "Sourcing ROS2 Jazzy base setup..."
if [ ! -f "${ROS2_SETUP}" ]; then
    echo "Error: ROS2 Jazzy not found at: ${ROS2_SETUP}"
    echo "Please install ROS2 Jazzy first!"
    exit 1
else
    source "${ROS2_SETUP}"
    echo "Successfully sourced ROS2 Jazzy base setup!"
    echo ""
fi

# Check if build.sh file exists
if [ ! -f "${BUILD_SCRIPT}" ]; then
    echo "Error: build.sh file not found at ${BUILD_SCRIPT}"
    exit 1
fi

# Check and add source setup.bash to .bashrc if not exists
if [ -f "${SETUP_BASH}" ]; then
    SOURCE_LINE="source ${SETUP_BASH}"
    
    # Check if source line already exists in .bashrc
    if ! grep -qF "${SOURCE_LINE}" "${BASHRC}" 2>/dev/null; then
        echo "Adding source setup.bash to .bashrc..."
        echo "" >> "${BASHRC}"
        echo "# Source livox_ros_driver2 setup.bash" >> "${BASHRC}"
        echo "${SOURCE_LINE}" >> "${BASHRC}"
        echo "Added: ${SOURCE_LINE} to ${BASHRC}"
        echo "Please run: source ${BASHRC} or open a new terminal to apply changes."
    else
        echo "Source setup.bash already exists in .bashrc"
    fi
else
    echo "Warning: setup.bash file does not exist at ${SETUP_BASH}"
    echo "File will be created after successful build."
fi

# Run build.sh with jazzy parameter
echo ""
echo "Starting build with jazzy..."
echo "=========================================="

cd "${DRIVE_WS_DIR}"
bash "${BUILD_SCRIPT}" jazzy

# After build completes, check again and add source if needed
if [ -f "${SETUP_BASH}" ]; then
    SOURCE_LINE="source ${SETUP_BASH}"
    
    if ! grep -qF "${SOURCE_LINE}" "${BASHRC}" 2>/dev/null; then
        echo ""
        echo "Adding source setup.bash to .bashrc after build..."
        echo "" >> "${BASHRC}"
        echo "# Source livox_ros_driver2 setup.bash" >> "${BASHRC}"
        echo "${SOURCE_LINE}" >> "${BASHRC}"
        echo "Added: ${SOURCE_LINE} to ${BASHRC}"
        echo "Please run: source ${BASHRC} or open a new terminal to apply changes."
    fi
fi

echo ""
echo "=========================================="
echo "Build completed!"
echo "=========================================="




