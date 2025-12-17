#!/bin/bash

# Build script for all calibration libraries
# This script builds and installs Ceres Solver, GTSAM, and Iridescence

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
echo "Build Script for Calibration Libraries"
echo "=========================================="
echo ""
echo "This script will build the following libraries:"
echo "  1. Ceres Solver"
echo "  2. GTSAM"
echo "  3. Iridescence"
echo ""
echo "Script directory: $SCRIPT_DIR"
echo ""

# Define build scripts
CERES_BUILD_SCRIPT="${SCRIPT_DIR}/ceres-solver/build.sh"
GTSAM_BUILD_SCRIPT="${SCRIPT_DIR}/gtsam/build.sh"
IRIDESCENCE_BUILD_SCRIPT="${SCRIPT_DIR}/iridescence/build.sh"

# Check if build scripts exist
MISSING_SCRIPTS=()

if [ ! -f "${CERES_BUILD_SCRIPT}" ]; then
    MISSING_SCRIPTS+=("ceres-solver/build.sh")
fi

if [ ! -f "${GTSAM_BUILD_SCRIPT}" ]; then
    MISSING_SCRIPTS+=("gtsam/build.sh")
fi

if [ ! -f "${IRIDESCENCE_BUILD_SCRIPT}" ]; then
    MISSING_SCRIPTS+=("iridescence/build.sh")
fi

if [ ${#MISSING_SCRIPTS[@]} -gt 0 ]; then
    echo "Error: The following build scripts are missing:"
    for script in "${MISSING_SCRIPTS[@]}"; do
        echo "  - ${script}"
    done
    exit 1
fi

# Build Ceres Solver
echo "=========================================="
echo "Building Ceres Solver (1/3)"
echo "=========================================="
echo ""
cd "${SCRIPT_DIR}"
bash "${CERES_BUILD_SCRIPT}"

# Build GTSAM
echo ""
echo "=========================================="
echo "Building GTSAM (2/3)"
echo "=========================================="
echo ""
cd "${SCRIPT_DIR}"
bash "${GTSAM_BUILD_SCRIPT}"

# Build Iridescence
echo ""
echo "=========================================="
echo "Building Iridescence (3/3)"
echo "=========================================="
echo ""
cd "${SCRIPT_DIR}"
bash "${IRIDESCENCE_BUILD_SCRIPT}"

echo ""
echo "=========================================="
echo "✅ All calibration libraries built and installed successfully!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✓ Ceres Solver"
echo "  ✓ GTSAM"
echo "  ✓ Iridescence"
<<<<<<< HEAD
echo ""
=======
echo ""


>>>>>>> f5d777a (update code gui)



