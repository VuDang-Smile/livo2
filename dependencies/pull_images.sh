#!/bin/bash

# Script to pull all required Docker images with retry logic
# This helps avoid timeout issues during build

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Wait for user to press Enter before exiting
wait_for_exit() {
    echo ""
    read -p "Press Enter to exit..."
    exit 1
}

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed."
    wait_for_exit
fi

# Check Docker permissions and set DOCKER_CMD
DOCKER_CMD="docker"
if ! docker ps &> /dev/null; then
    print_warning "No Docker permissions detected."
    print_info "Requesting sudo access for Docker commands..."
    if ! sudo -n true 2>/dev/null; then
        echo "Please enter your password to use Docker with sudo:"
        sudo -v
    fi
    DOCKER_CMD="sudo docker"
    print_success "Using sudo for Docker commands"
fi

echo ""
echo "=========================================="
echo "  Docker Images Pull Script"
echo "=========================================="
echo ""

# Function to pull image with retry
pull_image_with_retry() {
    local image=$1
    local max_attempts=5
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        print_info "Pulling $image (attempt $attempt/$max_attempts)..."
        
        if $DOCKER_CMD pull "$image" 2>&1; then
            print_success "Successfully pulled $image"
            return 0
        fi
        
        if [ $attempt -lt $max_attempts ]; then
            local wait_time=$((attempt * 5))
            print_warning "Pull failed. Retrying in ${wait_time} seconds..."
            sleep $wait_time
        fi
        
        attempt=$((attempt + 1))
    done
    
    print_error "Failed to pull $image after $max_attempts attempts."
    return 1
}

# List of required images
IMAGES=(
    "node:18-alpine"
    "nginx:alpine"
    "python:3.11-slim"
    "mongo:7"
    "eclipse-mosquitto:latest"
)

# Pull all images
failed_images=()
for image in "${IMAGES[@]}"; do
    if ! pull_image_with_retry "$image"; then
        failed_images+=("$image")
    fi
    echo ""
done

# Summary
echo "=========================================="
if [ ${#failed_images[@]} -eq 0 ]; then
    print_success "All images pulled successfully!"
    echo "=========================================="
    echo ""
    echo "You can now run: ./install_host.sh (from project root)"
else
    print_error "Some images failed to pull:"
    for img in "${failed_images[@]}"; do
        echo "  - $img"
    done
    echo ""
    print_info "You can try to pull them manually:"
    for img in "${failed_images[@]}"; do
        echo "  $DOCKER_CMD pull $img"
    done
    echo ""
    print_warning "Build may still work if images are pulled during build process"
fi
echo "=========================================="
echo ""

# Wait for user to press Enter before exiting
read -p "Press Enter to exit..."

