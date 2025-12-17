#!/bin/bash

# Common ROS2 Dependencies Installation Script
# Installs common ROS2 dependencies required for development

# Don't use set -e to handle errors better
# set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Variables to track installation status
INSTALL_SUCCESS=false
INSTALL_FAILED=false

# List of dependencies to install
# Add more dependencies here as needed
DEPENDENCIES=(
    "ros-jazzy-ament-cmake-auto"
    "ros-jazzy-pcl-ros"
    "libgstreamer1.0-dev"
    "libgstreamer-plugins-base1.0-dev"
    "libomp-dev"
    "libboost-all-dev"
    "libglm-dev"
    "libglfw3-dev"
    "libpng-dev"
    "libjpeg-dev"
    "libpcl-dev"
    "libgoogle-glog-dev"
    "python3-pip"
)

# Python packages to install via pip
PYTHON_PACKAGES=(
)

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

# Function to display result message and confirm before exit
confirm_exit() {
    local exit_code=$1
    local message=$2
    
    echo ""
    echo -e "${BLUE}========================================${NC}"
    
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}    SUCCESS!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo -e "${GREEN}$message${NC}"
    else
        echo -e "${RED}    FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        echo -e "${RED}$message${NC}"
    fi
    
    echo ""
    echo -e "${YELLOW}Press Enter to exit...${NC}"
    read -r
    exit $exit_code
}

# Function to check if a package is already installed
check_package_installed() {
    local package=$1
    
    if dpkg -l | grep -q "^ii.*${package}"; then
        return 0  # Already installed
    fi
    
    return 1  # Not installed
}

# Function to install a single dependency
install_dependency() {
    local package=$1
    
    print_info "Installing ${package}..."
    
    if ! sudo apt update; then
        print_error "Failed to update apt packages."
        return 1
    fi
    
    if ! sudo apt install -y "${package}"; then
        print_error "Failed to install ${package}."
        return 1
    fi
    
    print_success "${package} installed successfully."
    return 0
}

# Function to install all dependencies
install_all_dependencies() {
    local failed_packages=()
    local installed_count=0
    local skipped_count=0
    
    print_info "Checking and installing dependencies..."
    echo ""
    
    for package in "${DEPENDENCIES[@]}"; do
        echo -e "${BLUE}----------------------------------------${NC}"
        print_info "Processing: ${package}"
        
        if check_package_installed "${package}"; then
            print_success "${package} is already installed. Skipping..."
            skipped_count=$((skipped_count + 1))
        else
            if install_dependency "${package}"; then
                installed_count=$((installed_count + 1))
            else
                failed_packages+=("${package}")
                INSTALL_FAILED=true
            fi
        fi
        echo ""
    done
    
    # Summary
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Installation Summary${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    print_info "Total packages: ${#DEPENDENCIES[@]}"
    print_success "Installed: ${installed_count}"
    print_info "Already installed (skipped): ${skipped_count}"
    
    if [ ${#failed_packages[@]} -gt 0 ]; then
        print_error "Failed: ${#failed_packages[@]}"
        echo ""
        print_error "Failed packages:"
        for pkg in "${failed_packages[@]}"; do
            echo -e "  ${RED}- ${pkg}${NC}"
        done
        echo ""
        return 1
    else
        INSTALL_SUCCESS=true
        echo ""
        return 0
    fi
}

# Function to install Python packages via pip
install_python_packages() {
    print_info "Installing Python packages via pip..."
    echo ""
    
    # Check if pip is available
    if ! command -v pip3 &> /dev/null && ! command -v python3 -m pip &> /dev/null; then
        print_error "pip3 is not available. Please install python3-pip first."
        return 1
    fi
    
    # Determine pip command
    if command -v pip3 &> /dev/null; then
        PIP_CMD="pip3"
    else
        PIP_CMD="python3 -m pip"
    fi
    
    local failed_packages=()
    local installed_count=0
    local skipped_count=0
    
    for package in "${PYTHON_PACKAGES[@]}"; do
        echo -e "${BLUE}----------------------------------------${NC}"
        print_info "Processing Python package: ${package}"
        
        # Check if package is already installed
        if $PIP_CMD show "${package}" &> /dev/null; then
            print_success "${package} is already installed. Skipping..."
            skipped_count=$((skipped_count + 1))
        else
            print_info "Installing ${package}..."
            if $PIP_CMD install --user "${package}"; then
                print_success "${package} installed successfully."
                installed_count=$((installed_count + 1))
            else
                print_error "Failed to install ${package}."
                failed_packages+=("${package}")
                INSTALL_FAILED=true
            fi
        fi
        echo ""
    done
    
    # Summary
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Python Packages Installation Summary${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        print_info "Total Python packages: ${#PYTHON_PACKAGES[@]}"
        print_success "Installed: ${installed_count}"
        print_info "Already installed (skipped): ${skipped_count}"
        
        if [ ${#failed_packages[@]} -gt 0 ]; then
            print_error "Failed: ${#failed_packages[@]}"
            echo ""
            print_error "Failed packages:"
            for pkg in "${failed_packages[@]}"; do
                echo -e "  ${RED}- ${pkg}${NC}"
            done
            echo ""
            return 1
        fi
    fi
    
    return 0
}

# Function to verify all dependencies are installed
verify_installation() {
    print_info "Verifying installation..."
    echo ""
    
    local all_installed=true
    local missing_packages=()
    
    for package in "${DEPENDENCIES[@]}"; do
        if check_package_installed "${package}"; then
            print_success "${package} ✓"
        else
            print_error "${package} ✗"
            missing_packages+=("${package}")
            all_installed=false
        fi
    done
    
    echo ""
    
    # Verify Python packages
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        print_info "Verifying Python packages..."
        echo ""
        
        # Determine pip command
        if command -v pip3 &> /dev/null; then
            PIP_CMD="pip3"
        else
            PIP_CMD="python3 -m pip"
        fi
        
        for package in "${PYTHON_PACKAGES[@]}"; do
            if $PIP_CMD show "${package}" &> /dev/null; then
                print_success "${package} ✓"
            else
                print_error "${package} ✗"
                missing_packages+=("${package}")
                all_installed=false
            fi
        done
        echo ""
    fi
    
    if [ "$all_installed" = true ]; then
        return 0
    else
        return 1
    fi
}

# Main function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Common ROS2 Dependencies Installer${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check sudo privileges
    if ! sudo -n true 2>/dev/null; then
        print_info "This script requires sudo privileges. Please enter your password when prompted."
    fi
    
    # Display dependencies to be installed
    print_info "Dependencies to install:"
    for package in "${DEPENDENCIES[@]}"; do
        echo -e "  ${BLUE}- ${package}${NC}"
    done
    
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo ""
        print_info "Python packages to install:"
        for package in "${PYTHON_PACKAGES[@]}"; do
            echo -e "  ${BLUE}- ${package}${NC}"
        done
    fi
    echo ""
    
    # Install all dependencies
    if ! install_all_dependencies; then
        INSTALL_FAILED=true
        echo ""
        print_error "Some dependencies failed to install!"
        echo ""
        confirm_exit 1 "Dependency installation completed with errors!"
    fi
    
    # Install Python packages
    if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Installing Python Packages${NC}"
        echo -e "${BLUE}========================================${NC}"
        echo ""
        
        if ! install_python_packages; then
            INSTALL_FAILED=true
            echo ""
            print_warning "Some Python packages failed to install, but continuing..."
            echo ""
        fi
    fi
    
    # Verify installation
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Verifying Installation${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    if verify_installation; then
        INSTALL_SUCCESS=true
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}    INSTALLATION SUCCESSFUL!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "All dependencies have been installed successfully!"
        echo ""
        print_info "Installed apt packages:"
        for package in "${DEPENDENCIES[@]}"; do
            echo -e "  ${GREEN}✓ ${package}${NC}"
        done
        
        if [ ${#PYTHON_PACKAGES[@]} -gt 0 ]; then
            echo ""
            print_info "Installed Python packages:"
            for package in "${PYTHON_PACKAGES[@]}"; do
                echo -e "  ${GREEN}✓ ${package}${NC}"
            done
        fi
        echo ""
        confirm_exit 0 "All dependencies have been installed successfully!"
    else
        INSTALL_FAILED=true
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}    VERIFICATION FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        print_error "Some dependencies could not be verified. Please check the logs above."
        echo ""
        confirm_exit 1 "Dependency verification failed!"
    fi
}

# Run main function
main
