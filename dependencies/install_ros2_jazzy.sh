#!/bin/bash

# Automatic ROS2 Jazzy installation script
# Checks and installs ROS2 Jazzy if not already installed

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

# Function to check if ROS2 Jazzy is already installed
check_ros2_jazzy_installed() {
    # Check by looking for setup.bash file
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        return 0  # Already installed
    fi
    
    # Check using dpkg
    if dpkg -l | grep -q "ros-jazzy-desktop"; then
        return 0  # Already installed
    fi
    
    return 1  # Not installed
}

# Function to add ROS2 Jazzy source to .bashrc
add_to_bashrc() {
    local bashrc_file="$HOME/.bashrc"
    local source_line="source /opt/ros/jazzy/setup.bash"
    
    # Check if the line already exists in .bashrc
    if grep -Fxq "$source_line" "$bashrc_file" 2>/dev/null; then
        print_info "ROS2 Jazzy source is already in ~/.bashrc"
        return 0
    fi
    
    # Add the line to .bashrc
    print_info "Adding ROS2 Jazzy source to ~/.bashrc..."
    if echo "$source_line" >> "$bashrc_file"; then
        print_success "Added ROS2 Jazzy source to ~/.bashrc"
        print_info "The changes will take effect in new terminal sessions."
        return 0
    else
        print_warning "Failed to add ROS2 Jazzy source to ~/.bashrc"
        print_info "You can manually add it by running:"
        echo -e "  ${GREEN}echo '$source_line' >> ~/.bashrc${NC}"
        return 1
    fi
}

# Function to install UTF-8 locale
install_locale() {
    print_info "Checking and installing UTF-8 locale..."
    
    # Check current locale
    if locale | grep -q "LANG=en_US.UTF-8"; then
        print_success "UTF-8 locale is already configured."
        return 0
    fi
    
    print_info "Installing UTF-8 locale..."
    if ! sudo apt update && sudo apt install -y locales; then
        print_error "Failed to install locales."
        return 1
    fi
    
    # Generate locale
    if ! sudo locale-gen en_US en_US.UTF-8; then
        print_error "Failed to generate locale."
        return 1
    fi
    
    # Update locale
    if ! sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; then
        print_error "Failed to update locale."
        return 1
    fi
    
    # Export locale for current session
    export LANG=en_US.UTF-8
    
    print_success "UTF-8 locale installed successfully."
    
    # Verify settings
    print_info "Verifying locale settings:"
    locale
    
    return 0
}

# Function to install software-properties-common and universe repository
install_software_properties() {
    print_info "Installing software-properties-common and adding universe repository..."
    
    if ! sudo apt install -y software-properties-common; then
        print_error "Failed to install software-properties-common."
        return 1
    fi
    
    if ! sudo add-apt-repository -y universe; then
        print_error "Failed to add universe repository."
        return 1
    fi
    
    print_success "Installed software-properties-common and added universe repository."
    return 0
}

# Function to install ros-apt-source
install_ros_apt_source() {
    print_info "Installing ros-apt-source..."
    
    # Install curl if not already installed
    if ! sudo apt update && sudo apt install -y curl; then
        print_error "Failed to install curl."
        return 1
    fi
    
    # Get latest version of ros-apt-source
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    
    if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
        print_error "Failed to get ros-apt-source version. Check internet connection."
        return 1
    fi
    
    print_info "ros-apt-source version: $ROS_APT_SOURCE_VERSION"
    
    # Download and install ros-apt-source
    if ! curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"; then
        print_error "Failed to download ros-apt-source package."
        return 1
    fi
    
    if ! sudo dpkg -i /tmp/ros2-apt-source.deb; then
        print_warning "dpkg may encounter dependency errors, attempting to fix..."
        if ! sudo apt-get install -f -y; then
            print_error "Failed to fix dependencies."
            rm -f /tmp/ros2-apt-source.deb
            return 1
        fi
    fi
    
    # Cleanup
    rm -f /tmp/ros2-apt-source.deb
    
    print_success "ros-apt-source installed successfully."
    return 0
}

# Function to install ros-dev-tools
install_ros_dev_tools() {
    print_info "Installing ros-dev-tools..."
    
    if ! sudo apt update && sudo apt install -y ros-dev-tools; then
        print_error "Failed to install ros-dev-tools."
        return 1
    fi
    
    print_success "ros-dev-tools installed successfully."
    return 0
}

# Function to install ROS2 Jazzy Desktop
install_ros2_jazzy() {
    print_info "Installing ROS2 Jazzy Desktop (this may take several minutes)..."
    
    if ! sudo apt update; then
        print_error "Failed to update apt packages."
        return 1
    fi
    
    if ! sudo apt upgrade -y; then
        print_warning "Error upgrading packages, but continuing..."
    fi
    
    if ! sudo apt install -y ros-jazzy-desktop; then
        print_error "Failed to install ros-jazzy-desktop."
        return 1
    fi
    
    print_success "ROS2 Jazzy Desktop installed successfully."
    return 0
}

# Main function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    ROS2 Jazzy Installation Script${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check sudo privileges
    if ! sudo -n true 2>/dev/null; then
        print_info "This script requires sudo privileges. Please enter your password when prompted."
    fi
    
    # Check if ROS2 Jazzy is already installed
    print_info "Checking for ROS2 Jazzy installation..."
    
    if check_ros2_jazzy_installed; then
        INSTALL_SUCCESS=true
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}    ROS2 Jazzy Already Installed${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "ROS2 Jazzy is already installed!"
        print_info "Location: /opt/ros/jazzy/"
        echo ""
        
        # Check and add to .bashrc if not already there
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Checking .bashrc Configuration${NC}"
        echo -e "${BLUE}========================================${NC}"
        add_to_bashrc
        echo ""
        
        print_info "To use ROS2 Jazzy in current session, run:"
        echo -e "  ${GREEN}source /opt/ros/jazzy/setup.bash${NC}"
        echo ""
        print_info "Or open a new terminal - ROS2 Jazzy will be automatically sourced."
        echo ""
        confirm_exit 0 "ROS2 Jazzy is ready to use!"
    fi
    
    print_warning "ROS2 Jazzy is not installed."
    print_info "Starting ROS2 Jazzy installation process..."
    echo ""
    
    # Step 1: Install UTF-8 locale
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step 1: Installing UTF-8 locale${NC}"
    echo -e "${BLUE}========================================${NC}"
    if ! install_locale; then
        INSTALL_FAILED=true
        confirm_exit 1 "UTF-8 locale installation failed!"
    fi
    echo ""
    
    # Step 2: Install software-properties-common
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step 2: Installing software-properties-common${NC}"
    echo -e "${BLUE}========================================${NC}"
    if ! install_software_properties; then
        INSTALL_FAILED=true
        confirm_exit 1 "software-properties-common installation failed!"
    fi
    echo ""
    
    # Step 3: Install ros-apt-source
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step 3: Installing ros-apt-source${NC}"
    echo -e "${BLUE}========================================${NC}"
    if ! install_ros_apt_source; then
        INSTALL_FAILED=true
        confirm_exit 1 "ros-apt-source installation failed!"
    fi
    echo ""
    
    # Step 4: Install ros-dev-tools
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step 4: Installing ros-dev-tools${NC}"
    echo -e "${BLUE}========================================${NC}"
    if ! install_ros_dev_tools; then
        INSTALL_FAILED=true
        confirm_exit 1 "ros-dev-tools installation failed!"
    fi
    echo ""
    
    # Step 5: Install ROS2 Jazzy Desktop
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Step 5: Installing ROS2 Jazzy Desktop${NC}"
    echo -e "${BLUE}========================================${NC}"
    if ! install_ros2_jazzy; then
        INSTALL_FAILED=true
        confirm_exit 1 "ROS2 Jazzy Desktop installation failed!"
    fi
    echo ""
    
    # Verify installation after completion
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Verifying Installation${NC}"
    echo -e "${BLUE}========================================${NC}"
    
    if check_ros2_jazzy_installed; then
        INSTALL_SUCCESS=true
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}    INSTALLATION SUCCESSFUL!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "ROS2 Jazzy has been installed successfully!"
        echo ""
        
        # Automatically add to .bashrc
        echo -e "${BLUE}========================================${NC}"
        echo -e "${BLUE}Configuring .bashrc${NC}"
        echo -e "${BLUE}========================================${NC}"
        add_to_bashrc
        echo ""
        
        print_info "To use ROS2 Jazzy in current session, run:"
        echo -e "  ${GREEN}source /opt/ros/jazzy/setup.bash${NC}"
        echo ""
        print_info "Or open a new terminal - ROS2 Jazzy will be automatically sourced."
        echo ""
        confirm_exit 0 "ROS2 Jazzy has been installed successfully!"
    else
        INSTALL_FAILED=true
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}    INSTALLATION FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        print_error "Installation appears to have failed. Please check the logs above."
        print_info "Check the logs above for detailed error information."
        echo ""
        confirm_exit 1 "ROS2 Jazzy installation failed!"
    fi
}

# Run main function
main

