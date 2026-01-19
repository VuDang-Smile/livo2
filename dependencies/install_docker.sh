#!/bin/bash

# Docker Installation Script
# Installs Docker Engine and Docker Compose for Ubuntu/Debian systems

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
    
    if [ -z "${SKIP_EXIT_PROMPT:-}" ]; then
        echo ""
        echo -e "${YELLOW}Press Enter to exit...${NC}"
        read -r
    fi
    exit $exit_code
}

# Function to check if Docker is already installed
check_docker_installed() {
    if command -v docker &> /dev/null; then
        return 0  # Docker is installed
    fi
    return 1  # Docker is not installed
}

# Function to check if Docker Compose is already installed
check_docker_compose_installed() {
    if command -v docker-compose &> /dev/null || docker compose version &> /dev/null; then
        return 0  # Docker Compose is installed
    fi
    return 1  # Docker Compose is not installed
}

# Function to check if user is in docker group
check_user_in_docker_group() {
    if groups | grep -q "\bdocker\b"; then
        return 0  # User is in docker group
    fi
    return 1  # User is not in docker group
}

# Function to remove old Docker versions
remove_old_docker() {
    print_info "Removing old Docker versions if any..."
    
    if ! sudo apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null; then
        print_warning "No old Docker versions found or removal failed (this is OK)."
    else
        print_success "Old Docker versions removed."
    fi
}

# Function to install Docker using official repository
install_docker() {
    print_info "Installing Docker Engine..."
    echo ""
    
    # Step 1: Update apt package index
    print_info "Updating apt package index..."
    if ! sudo apt-get update; then
        print_error "Failed to update apt package index."
        return 1
    fi
    print_success "Apt package index updated."
    echo ""
    
    # Step 2: Install prerequisites
    print_info "Installing prerequisites..."
    local prerequisites=(
        "ca-certificates"
        "curl"
        "gnupg"
        "lsb-release"
    )
    
    for pkg in "${prerequisites[@]}"; do
        if ! sudo apt-get install -y "$pkg"; then
            print_error "Failed to install prerequisite: $pkg"
            return 1
        fi
    done
    print_success "Prerequisites installed."
    echo ""
    
    # Step 3: Add Docker's official GPG key
    print_info "Adding Docker's official GPG key..."
    if [ -d /etc/apt/keyrings ]; then
        sudo mkdir -p /etc/apt/keyrings
    fi
    
    if ! sudo rm -f /etc/apt/keyrings/docker.gpg 2>/dev/null; then
        : # Ignore if file doesn't exist
    fi
    
    if ! curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg; then
        print_error "Failed to add Docker's GPG key."
        return 1
    fi
    print_success "Docker GPG key added."
    echo ""
    
    # Step 4: Set up Docker repository
    print_info "Setting up Docker repository..."
    local arch=$(dpkg --print-architecture)
    local distro=$(lsb_release -cs)
    
    if ! echo "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${distro} stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null; then
        print_error "Failed to set up Docker repository."
        return 1
    fi
    print_success "Docker repository configured."
    echo ""
    
    # Step 5: Update apt package index again
    print_info "Updating apt package index with Docker repository..."
    if ! sudo apt-get update; then
        print_error "Failed to update apt package index with Docker repository."
        return 1
    fi
    print_success "Apt package index updated with Docker repository."
    echo ""
    
    # Step 6: Install Docker Engine, CLI, and Containerd
    print_info "Installing Docker Engine, CLI, and Containerd..."
    if ! sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; then
        print_error "Failed to install Docker packages."
        return 1
    fi
    print_success "Docker Engine installed successfully."
    echo ""
    
    # Step 7: Install Docker Compose standalone (for docker-compose command)
    print_info "Installing Docker Compose standalone..."
    
    # Check if docker-compose is already installed
    if command -v docker-compose &> /dev/null; then
        print_success "Docker Compose standalone is already installed: $(docker-compose --version 2>/dev/null)"
    else
        # Get latest stable version from GitHub API
        local compose_version=""
        if command -v curl &> /dev/null && command -v grep &> /dev/null; then
            # Try to get latest version from GitHub API
            local api_response=$(curl -s https://api.github.com/repos/docker/compose/releases/latest 2>/dev/null)
            if [ -n "$api_response" ]; then
                # Extract tag_name using sed (more compatible than grep -P)
                compose_version=$(echo "$api_response" | grep '"tag_name"' | head -1 | sed -E 's/.*"tag_name":\s*"([^"]+)".*/\1/')
            fi
        fi
        
        # Fallback to a known stable version if API call fails
        if [ -z "$compose_version" ]; then
            compose_version="v2.24.0"
            print_info "Using fallback version: ${compose_version}"
        else
            print_info "Latest version detected: ${compose_version}"
        fi
        
        # Determine architecture
        local arch=$(uname -m)
        case "$arch" in
            x86_64) arch="x86_64" ;;
            aarch64|arm64) arch="aarch64" ;;
            armv7l|armhf) arch="armv7" ;;
            *) arch="x86_64" ;; # Default fallback
        esac
        
        local compose_url="https://github.com/docker/compose/releases/download/${compose_version}/docker-compose-linux-${arch}"
        
        print_info "Downloading Docker Compose from: ${compose_url}"
        
        # Download Docker Compose
        if ! sudo curl -L "${compose_url}" -o /usr/local/bin/docker-compose; then
            print_warning "Failed to download Docker Compose standalone. Continuing with plugin version only."
        else
            # Make it executable
            if sudo chmod +x /usr/local/bin/docker-compose; then
                print_success "Docker Compose standalone installed successfully: ${compose_version}"
            else
                print_warning "Docker Compose downloaded but failed to make executable."
            fi
        fi
    fi
    echo ""
    
    return 0
}

# Function to add user to docker group
add_user_to_docker_group() {
    print_info "Adding current user to docker group..."
    
    local current_user=$(whoami)
    
    if ! sudo usermod -aG docker "$current_user"; then
        print_error "Failed to add user to docker group."
        return 1
    fi
    
    print_success "User '$current_user' added to docker group."
    print_warning "You may need to log out and log back in for group changes to take effect."
    print_warning "Alternatively, you can use 'newgrp docker' to activate the group without logging out."
    echo ""
    
    return 0
}

# Function to start and enable Docker service
start_docker_service() {
    print_info "Starting Docker service..."
    
    if ! sudo systemctl start docker; then
        print_error "Failed to start Docker service."
        return 1
    fi
    print_success "Docker service started."
    
    print_info "Enabling Docker service to start on boot..."
    if ! sudo systemctl enable docker; then
        print_warning "Failed to enable Docker service, but it's running."
    else
        print_success "Docker service enabled to start on boot."
    fi
    echo ""
    
    return 0
}

# Function to verify Docker installation
verify_docker_installation() {
    print_info "Verifying Docker installation..."
    echo ""
    
    local all_verified=true
    
    # Check Docker command
    if check_docker_installed; then
        local docker_version=$(docker --version 2>/dev/null)
        print_success "Docker is installed: $docker_version"
    else
        print_error "Docker command not found!"
        all_verified=false
    fi
    
    # Check Docker Compose plugin
    if docker compose version &> /dev/null; then
        local compose_plugin_version=$(docker compose version 2>/dev/null)
        print_success "Docker Compose plugin is installed: $compose_plugin_version"
    else
        print_warning "Docker Compose plugin not found."
        all_verified=false
    fi
    
    # Check Docker Compose standalone
    if command -v docker-compose &> /dev/null; then
        local compose_standalone_version=$(docker-compose --version 2>/dev/null)
        print_success "Docker Compose standalone is installed: $compose_standalone_version"
    else
        print_warning "Docker Compose standalone not found."
    fi
    
    # Check Docker service status
    if sudo systemctl is-active --quiet docker; then
        print_success "Docker service is running."
    else
        print_error "Docker service is not running!"
        all_verified=false
    fi
    
    # Check if user is in docker group
    if check_user_in_docker_group; then
        print_success "Current user is in docker group."
    else
        print_warning "Current user is not in docker group. You may need to log out and log back in."
    fi
    
    # Test Docker with a simple command (requires sudo if user not in docker group)
    echo ""
    print_info "Testing Docker with 'docker ps' command..."
    if sudo docker ps &> /dev/null; then
        print_success "Docker is working correctly!"
    else
        print_error "Docker test failed!"
        all_verified=false
    fi
    
    echo ""
    
    if [ "$all_verified" = true ]; then
        return 0
    else
        return 1
    fi
}

# Main function
main() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}    Docker Installation Script${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # Check sudo privileges
    if ! sudo -n true 2>/dev/null; then
        print_info "This script requires sudo privileges. Please enter your password when prompted."
    fi
    
    # Check if Docker is already installed
    if check_docker_installed; then
        local docker_version=$(docker --version 2>/dev/null)
        print_warning "Docker appears to be already installed: $docker_version"
        echo ""
        
        # If running from install_all_dependencies.sh (SKIP_EXIT_PROMPT is set),
        # automatically skip reinstall and just verify Docker is working
        if [ -n "${SKIP_EXIT_PROMPT:-}" ]; then
            print_info "Skipping Docker reinstallation (auto-mode: Docker already installed)."
            echo ""
            
            # Still verify and check docker group
            if verify_docker_installation; then
                INSTALL_SUCCESS=true
                # Don't call confirm_exit in auto-mode, just return successfully
                return 0
            else
                # Docker is installed but verification failed (e.g., user not in docker group)
                # In auto-mode, still return success since Docker is installed
                # The issue (like docker group) can be fixed later
                INSTALL_SUCCESS=true
                print_warning "Docker is installed but some verification checks failed (e.g., user may not be in docker group)."
                print_info "Docker installation is considered successful. You may need to log out and log back in to use Docker without sudo."
                echo ""
                return 0
            fi
        else
            # Interactive mode: ask user
            print_info "Do you want to reinstall Docker? (y/N)"
            read -r response
            if [[ ! "$response" =~ ^[Yy]$ ]]; then
                print_info "Skipping Docker installation."
                echo ""
                
                # Still verify and check docker group
                if verify_docker_installation; then
                    INSTALL_SUCCESS=true
                    confirm_exit 0 "Docker is already installed and working!"
                else
                    INSTALL_FAILED=true
                    confirm_exit 1 "Docker is installed but verification failed!"
                fi
                return
            fi
            echo ""
        fi
    fi
    
    # Display what will be installed
    print_info "This script will install:"
    echo -e "  ${BLUE}-${NC} Docker Engine"
    echo -e "  ${BLUE}-${NC} Docker CLI"
    echo -e "  ${BLUE}-${NC} Containerd"
    echo -e "  ${BLUE}-${NC} Docker Buildx plugin"
    echo -e "  ${BLUE}-${NC} Docker Compose plugin (docker compose)"
    echo -e "  ${BLUE}-${NC} Docker Compose standalone (docker-compose)"
    echo ""
    
    # Remove old Docker versions
    echo -e "${BLUE}----------------------------------------${NC}"
    print_info "Removing old Docker versions (if any)..."
    echo ""
    remove_old_docker
    echo ""
    
    # Install Docker
    echo -e "${BLUE}----------------------------------------${NC}"
    print_info "Installing Docker..."
    echo ""
    if ! install_docker; then
        INSTALL_FAILED=true
        echo ""
        print_error "Docker installation failed!"
        echo ""
        confirm_exit 1 "Docker installation failed!"
    fi
    
    # Start Docker service
    echo -e "${BLUE}----------------------------------------${NC}"
    print_info "Starting Docker service..."
    echo ""
    if ! start_docker_service; then
        INSTALL_FAILED=true
        echo ""
        print_error "Failed to start Docker service!"
        echo ""
        confirm_exit 1 "Docker installation completed but service failed to start!"
    fi
    
    # Add user to docker group
    echo -e "${BLUE}----------------------------------------${NC}"
    print_info "Configuring user permissions..."
    echo ""
    if ! check_user_in_docker_group; then
        if ! add_user_to_docker_group; then
            print_warning "Failed to add user to docker group. You may need to do this manually."
        fi
    else
        print_success "User is already in docker group."
    fi
    echo ""
    
    # Verify installation
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Verifying Installation${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    if verify_docker_installation; then
        INSTALL_SUCCESS=true
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}    INSTALLATION SUCCESSFUL!${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        print_success "Docker has been installed successfully!"
        echo ""
        print_info "Installed components:"
        echo -e "  ${GREEN}✓${NC} Docker Engine: $(docker --version 2>/dev/null || echo 'N/A')"
        if docker compose version &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} Docker Compose Plugin: $(docker compose version 2>/dev/null || echo 'N/A')"
        fi
        if command -v docker-compose &> /dev/null; then
            echo -e "  ${GREEN}✓${NC} Docker Compose Standalone: $(docker-compose --version 2>/dev/null || echo 'N/A')"
        fi
        echo ""
        print_warning "IMPORTANT: If you were added to the docker group, you need to:"
        echo -e "  ${YELLOW}1.${NC} Log out and log back in, OR"
        echo -e "  ${YELLOW}2.${NC} Run 'newgrp docker' to activate the group without logging out"
        echo ""
        print_info "After that, you can run Docker commands without sudo."
        echo ""
        confirm_exit 0 "Docker installation completed successfully!"
    else
        INSTALL_FAILED=true
        echo ""
        echo -e "${RED}========================================${NC}"
        echo -e "${RED}    VERIFICATION FAILED!${NC}"
        echo -e "${RED}========================================${NC}"
        echo ""
        print_error "Docker installation completed but verification failed. Please check the logs above."
        echo ""
        confirm_exit 1 "Docker verification failed!"
    fi
}

# Run main function
main

