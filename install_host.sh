#!/bin/bash

# Script to install and start all services on host with Docker Compose
# This includes nginx reverse proxy, frontend, backend, MongoDB, and MQTT

# Don't use set -e to handle errors better
# set -e

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

# Check if Docker is installed, if not, install it
check_and_install_docker() {
    if command -v docker &> /dev/null; then
        print_success "Docker is already installed: $(docker --version 2>/dev/null || echo 'unknown version')"
        return 0
    fi
    
    print_warning "Docker is not installed."
    print_info "This script will install Docker automatically."
    echo ""
    read -p "Do you want to install Docker now? (Y/n): " -n 1 -r
    echo ""
    
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        print_info "Installing Docker..."
        echo ""
        
        local docker_script="${SCRIPT_DIR}/dependencies/install_docker.sh"
        if [ ! -f "$docker_script" ]; then
            print_error "Docker installation script not found: $docker_script"
            wait_for_exit
        fi
        
        # Make script executable
        chmod +x "$docker_script"
        
        # Run Docker installation script
        # Use yes to provide Enter input for "Press Enter to exit" prompts
        if yes "" | bash "$docker_script" 2>&1; then
            print_success "Docker installed successfully!"
            echo ""
            
            # Verify Docker is working (with sudo if needed)
            if docker ps &> /dev/null; then
                print_success "Docker is working correctly!"
                return 0
            elif sudo docker ps &> /dev/null; then
                print_success "Docker is working with sudo!"
                print_info "Note: This script will use sudo for Docker commands."
                print_info "To use Docker without sudo, log out and log back in after installation."
                return 0
            else
                print_error "Docker installation completed but Docker is not working."
                print_info "Please try logging out and logging back in, then run this script again."
                wait_for_exit
            fi
        else
            print_error "Docker installation failed!"
            wait_for_exit
        fi
    else
        print_error "Docker installation cancelled. Cannot continue without Docker."
        wait_for_exit
    fi
}

# Check and install Docker if needed
check_and_install_docker

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

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Create necessary directories
print_info "Creating necessary directories..."
mkdir -p backend/uploads backend/processed

# Wait for MongoDB to be ready function
wait_for_mongodb() {
    print_info "Waiting for MongoDB to be ready..."
    local max_attempts=30
    local attempt=0
    
    while [ $attempt -lt $max_attempts ]; do
        if $DOCKER_CMD exec livo-mongodb mongosh --eval "db.adminCommand('ping')" &> /dev/null; then
            print_success "MongoDB is ready!"
            return 0
        fi
        attempt=$((attempt + 1))
        print_info "  Attempt $attempt/$max_attempts..."
        sleep 2
    done
    
    print_warning "MongoDB may not be ready yet, but continuing..."
    return 1
}

# Pull base images first with retry
echo ""
print_info "Pulling base images (this may take a while)..."

pull_image_with_retry() {
    local image=$1
    local max_attempts=3
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        print_info "Pulling $image (attempt $attempt/$max_attempts)..."
        
        if $DOCKER_CMD pull "$image" 2>&1; then
            print_success "Successfully pulled $image"
            return 0
        fi
        
        if [ $attempt -lt $max_attempts ]; then
            print_warning "Pull failed. Retrying in 10 seconds..."
            sleep 10
        fi
        
        attempt=$((attempt + 1))
    done
    
    print_error "Failed to pull $image after $max_attempts attempts."
    return 1
}

# Pull required base images
print_info "Pulling base images..."
pull_image_with_retry "node:18-alpine" || print_warning "Failed to pull node:18-alpine, will try during build"
pull_image_with_retry "nginx:alpine" || print_warning "Failed to pull nginx:alpine, will try during build"
pull_image_with_retry "python:3.11-slim" || print_warning "Failed to pull python:3.11-slim, will try during build"
pull_image_with_retry "mongo:7" || print_warning "Failed to pull mongo:7, will try during build"
pull_image_with_retry "eclipse-mosquitto:latest" || print_warning "Failed to pull eclipse-mosquitto:latest, will try during build"

# Build and start containers with retry logic
echo ""
print_info "Building and starting all containers with docker compose..."

# Function to build with retry
build_with_retry() {
    local max_attempts=3
    local attempt=1
    
    while [ $attempt -le $max_attempts ]; do
        print_info "Build attempt $attempt/$max_attempts..."
        
        # Build images first (without --no-cache to use pulled images)
        if $DOCKER_CMD compose build 2>&1; then
            print_success "Build successful!"
            return 0
        fi
        
        if [ $attempt -lt $max_attempts ]; then
            print_warning "Build failed. Retrying in 10 seconds..."
            sleep 10
        fi
        
        attempt=$((attempt + 1))
    done
    
    print_error "Build failed after $max_attempts attempts."
    return 1
}

# Try to build with retry
if ! build_with_retry; then
    print_error "Failed to build containers after multiple attempts."
    print_info "Possible solutions:"
    print_info "  1. Check your internet connection"
    print_info "  2. Try again later (Docker Hub may be slow)"
    print_info "  3. Configure Docker registry mirror if available"
    wait_for_exit
fi

# Start containers
print_info "Starting containers..."
if ! $DOCKER_CMD compose up -d; then
    print_error "Failed to start containers."
    wait_for_exit
fi

# Wait for MongoDB
wait_for_mongodb

# Health check
echo ""
print_info "Performing health checks..."

# Check Nginx
sleep 2
if curl -f http://localhost/health &> /dev/null; then
    print_success "Nginx Reverse Proxy is healthy"
else
    print_warning "Nginx health check failed (may still be starting)"
fi

# Check Frontend (via Nginx)
sleep 2
if curl -f http://localhost/health -H "Host: frontend.lidar.tm" &> /dev/null; then
    print_success "Frontend is healthy (via Nginx)"
else
    print_warning "Frontend health check via Nginx failed (may still be starting)"
fi

# Check Frontend (direct port 3000)
if curl -f http://localhost:3000/health &> /dev/null; then
    print_success "Frontend is healthy (direct port 3000)"
else
    print_warning "Frontend health check on port 3000 failed (may still be starting)"
fi

# Check Backend (via Nginx)
sleep 2
if curl -f http://localhost/health -H "Host: backend.lidar.tm" &> /dev/null; then
    print_success "Backend is healthy (via Nginx)"
else
    print_warning "Backend health check via Nginx failed (may still be starting)"
fi

# Check Backend (direct port 8000)
if curl -f http://localhost:8000/health &> /dev/null; then
    print_success "Backend is healthy (direct port 8000)"
else
    print_warning "Backend health check on port 8000 failed (may still be starting)"
fi

# Check MongoDB
if $DOCKER_CMD exec livo-mongodb mongosh --eval "db.adminCommand('ping')" &> /dev/null; then
    print_success "MongoDB is healthy"
else
    print_warning "MongoDB health check failed"
fi

echo ""
echo "=========================================="
print_success "All services started!"
echo "=========================================="
echo ""
echo "Services:"
echo ""
echo "Access via Domain (through Nginx):"
echo "  - Frontend:            http://frontend.lidar.tm"
echo "  - Backend API:         http://backend.lidar.tm"
echo "  - Backend Docs:        http://backend.lidar.tm/docs"
echo ""
echo "Access via IP and Port (direct):"
echo "  - Frontend:            http://<server_ip>:3000"
echo "  - Backend API:         http://<server_ip>:8000"
echo "  - Backend Docs:        http://<server_ip>:8000/docs"
echo ""
echo "Access via IP (through Nginx on port 80):"
echo "  - Frontend:            http://<server_ip>"
echo ""
echo "Other Services:"
echo "  - MongoDB:             localhost:27017"
echo "  - MQTT Broker:         localhost:1883"
echo ""
echo "Note: To use domain names, add these to /etc/hosts:"
echo "  <server_ip>    frontend.lidar.tm"
echo "  <server_ip>    backend.lidar.tm"
echo ""
echo "Or run: sudo ./dependencies/find_backend_lan.sh"
echo ""
echo "To view logs:"
echo "  $DOCKER_CMD compose logs -f"
echo ""
echo "To stop services:"
echo "  $DOCKER_CMD compose down"
echo ""

# Wait for user to press Enter before exiting
echo ""
read -p "Press Enter to exit..."
