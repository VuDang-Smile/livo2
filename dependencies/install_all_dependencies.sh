#!/bin/bash

# Master installation script for all dependencies
# Installs ROS2, common dependencies, Livox driver, Theta driver, and Calibration libraries in sequence

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Script paths
ROS2_SCRIPT="${SCRIPT_DIR}/install_ros2_jazzy.sh"
COMMON_SCRIPT="${SCRIPT_DIR}/install_common_dependencies.sh"
LIVOX_SDK_SCRIPT="${SCRIPT_DIR}/Livox-SDK2/build.sh"
LIVOX_SCRIPT="${SCRIPT_DIR}/drive_ws/build.sh"
THETA_SCRIPT="${PROJECT_ROOT}/ws/src/theta_driver/3rd/build.sh"
CALIBRATION_SCRIPT="${SCRIPT_DIR}/calibration/build.sh"

# Track overall status
OVERALL_SUCCESS=true
FAILED_STEPS=()
SUDO_KEEPALIVE_PID=""

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to request sudo password at the beginning
request_sudo_password() {
    echo ""
    print_info "This script requires sudo privileges to install packages and build drivers."
    print_info "Please enter your password to continue..."
    echo ""
    
    # Request sudo password by running a test command
    # This will prompt for password if not already authenticated
    if ! sudo -v; then
        print_error "Failed to authenticate with sudo. Exiting..."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Keep sudo session alive for the duration of the script
    # This prevents multiple password prompts during installation
    print_success "Sudo authentication successful!"
    print_info "Sudo session will be kept alive during installation."
    echo ""
    
    # Start a background process to keep sudo alive
    (
        while true; do
            sudo -n true 2>/dev/null || exit 1
            sleep 60
        done
    ) &
    SUDO_KEEPALIVE_PID=$!
    
    # Ensure we kill the keepalive process on exit
    trap "if [ -n \"$SUDO_KEEPALIVE_PID\" ]; then kill $SUDO_KEEPALIVE_PID 2>/dev/null; fi" EXIT
    
    return 0
}

# Function to run a script step
run_step() {
    local step_name=$1
    local script_path=$2
    local step_number=$3
    
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step ${step_number}: ${step_name}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check if script exists
    if [ ! -f "${script_path}" ]; then
        print_error "Script not found: ${script_path}"
        OVERALL_SUCCESS=false
        FAILED_STEPS+=("${step_name} (script not found)")
        return 1
    fi
    
    # Make script executable
    chmod +x "${script_path}"
    
    # Run script and capture exit code
    # Use timeout to prevent hanging, and pipe empty input to handle "Press Enter" prompts
    print_info "Running: ${script_path}"
    echo ""
    
    # Run script with auto-continue for "Press Enter" prompts
    # Use expect-like behavior: pipe empty lines to handle read prompts
    if bash "${script_path}" < <(yes ""); then
        print_success "${step_name} completed successfully!"
        echo ""
        return 0
    else
        local exit_code=$?
        print_error "${step_name} failed with exit code: ${exit_code}"
        OVERALL_SUCCESS=false
        FAILED_STEPS+=("${step_name} (exit code: ${exit_code})")
        echo ""
        return 1
    fi
}

# Function to handle script that requires Enter press
# This function runs script and automatically provides Enter when script succeeds
run_step_with_auto_continue() {
    local step_name=$1
    local script_path=$2
    local step_number=$3
    
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step ${step_number}: ${step_name}${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check if script exists
    if [ ! -f "${script_path}" ]; then
        print_error "Script not found: ${script_path}"
        OVERALL_SUCCESS=false
        FAILED_STEPS+=("${step_name} (script not found)")
        return 1
    fi
    
    # Make script executable
    chmod +x "${script_path}"
    
    print_info "Running: ${script_path}"
    echo ""
    
    # Run script with automatic Enter input for "Press Enter" prompts
    # Use yes command to provide continuous empty input
    # The scripts use "read -p" or "read -r" which will consume input from stdin
    local exit_code=0
    
    # Use yes "" to provide continuous newlines for "Press Enter" prompts
    # Use timeout to prevent script from hanging indefinitely
    if command -v timeout >/dev/null 2>&1; then
        # Use timeout with yes command to provide input
        # yes "" generates infinite empty lines, timeout prevents hanging
        # Run script and capture exit code properly
        yes "" | timeout 3600 bash "${script_path}" 2>&1
        exit_code=${PIPESTATUS[1]}
        # Check if timeout occurred (exit code 124)
        if [ $exit_code -eq 124 ]; then
            print_error "Script timed out after 1 hour"
            exit_code=1
        fi
    else
        # Fallback: run without timeout, use yes to provide input
        # Note: This may hang if script waits for input indefinitely
        yes "" | bash "${script_path}" 2>&1
        exit_code=${PIPESTATUS[1]}
    fi
    
    # Check exit code
    if [ $exit_code -eq 0 ]; then
        print_success "${step_name} completed successfully!"
        echo ""
        return 0
    else
        print_error "${step_name} failed with exit code: ${exit_code}"
        OVERALL_SUCCESS=false
        FAILED_STEPS+=("${step_name} (exit code: ${exit_code})")
        echo ""
        return 1
    fi
}

# Main installation function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Master Dependencies Installation${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${BLUE}This script will install:${NC}"
    echo -e "  ${BLUE}1.${NC} ROS2 Jazzy"
    echo -e "  ${BLUE}2.${NC} Common ROS2 Dependencies"
    echo -e "  ${BLUE}3.${NC} Livox SDK2"
    echo -e "  ${BLUE}4.${NC} Livox ROS2 Driver"
    echo -e "  ${BLUE}5.${NC} Theta Driver"
    echo -e "  ${BLUE}6.${NC} Calibration Libraries (Ceres Solver, GTSAM, Iridescence)"
    echo ""
    
    # Request sudo password at the beginning
    if ! request_sudo_password; then
        exit 1
    fi
    
    # Step 1: Install ROS2 Jazzy
    if ! run_step_with_auto_continue "Installing ROS2 Jazzy" "${ROS2_SCRIPT}" "1"; then
        print_error "Failed at Step 1: ROS2 Jazzy installation"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Step 2: Install Common Dependencies
    if ! run_step_with_auto_continue "Installing Common Dependencies" "${COMMON_SCRIPT}" "2"; then
        print_error "Failed at Step 2: Common Dependencies installation"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Step 3: Build Livox SDK2
    if ! run_step_with_auto_continue "Building Livox SDK2" "${LIVOX_SDK_SCRIPT}" "3"; then
        print_error "Failed at Step 3: Livox SDK2 build"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Step 4: Build Livox Driver
    if ! run_step_with_auto_continue "Building Livox Driver" "${LIVOX_SCRIPT}" "4"; then
        print_error "Failed at Step 4: Livox Driver build"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Step 5: Build Theta Driver
    if ! run_step_with_auto_continue "Building Theta Driver" "${THETA_SCRIPT}" "5"; then
        print_error "Failed at Step 5: Theta Driver build"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Step 6: Build Calibration Libraries
    if ! run_step_with_auto_continue "Building Calibration Libraries" "${CALIBRATION_SCRIPT}" "6"; then
        print_error "Failed at Step 6: Calibration Libraries build"
        print_info "Please check the error messages above and fix the issues before continuing."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
    
    # Final summary
    echo ""
    echo -e "${BLUE}========================================${NC}"
    if [ "$OVERALL_SUCCESS" = true ]; then
        echo -e "${GREEN}    ALL INSTALLATIONS SUCCESSFUL!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "All dependencies have been installed successfully!"
        echo ""
        print_info "Installed components:"
        echo -e "  ${GREEN}✓${NC} ROS2 Jazzy"
        echo -e "  ${GREEN}✓${NC} Common ROS2 Dependencies"
        echo -e "  ${GREEN}✓${NC} Livox SDK2"
        echo -e "  ${GREEN}✓${NC} Livox ROS2 Driver"
        echo -e "  ${GREEN}✓${NC} Theta Driver"
        echo -e "  ${GREEN}✓${NC} Calibration Libraries (Ceres Solver, GTSAM, Iridescence)"
        echo ""
        print_info "You can now use ROS2 and the installed drivers."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 0
    else
        echo -e "${RED}    INSTALLATION FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        print_error "Some installation steps failed!"
        echo ""
        print_error "Failed steps:"
        for step in "${FAILED_STEPS[@]}"; do
            echo -e "  ${RED}✗ ${step}${NC}"
        done
        echo ""
        print_info "Please check the error messages above and fix the issues."
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
        exit 1
    fi
}

# Run main function
main
