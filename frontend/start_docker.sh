#!/bin/bash

# Script to build and start the frontend Docker container

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
IMAGE_NAME="livo2-frontend"
CONTAINER_NAME="livo2-frontend-container"
PORT=3000

echo -e "${GREEN}=== Livo2 Frontend Docker Setup ===${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

# Check Docker permissions and set DOCKER_CMD
DOCKER_CMD="docker"
if ! docker ps &> /dev/null; then
    echo ""
    echo -e "${YELLOW}⚠️  No Docker permissions detected.${NC}"
    echo "Requesting sudo access for Docker commands..."
    echo ""
    # Test sudo access
    if ! sudo -n true 2>/dev/null; then
        echo "Please enter your password to use Docker with sudo:"
        sudo -v
    fi
    DOCKER_CMD="sudo docker"
    echo -e "${GREEN}✅ Using sudo for Docker commands${NC}"
    echo ""
fi

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Stop and remove existing container if it exists
if [ "$($DOCKER_CMD ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo -e "${YELLOW}Stopping existing container...${NC}"
    $DOCKER_CMD stop $CONTAINER_NAME > /dev/null 2>&1 || true
    echo -e "${YELLOW}Removing existing container...${NC}"
    $DOCKER_CMD rm $CONTAINER_NAME > /dev/null 2>&1 || true
fi

# Build the Docker image
echo -e "${GREEN}Building Docker image: $IMAGE_NAME${NC}"
$DOCKER_CMD build -t $IMAGE_NAME .

if [ $? -ne 0 ]; then
    echo -e "${RED}Error: Docker build failed.${NC}"
    exit 1
fi

# Run the container
echo -e "${GREEN}Starting container: $CONTAINER_NAME${NC}"
$DOCKER_CMD run -d \
    --name $CONTAINER_NAME \
    -p $PORT:80 \
    --restart unless-stopped \
    $IMAGE_NAME

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Container started successfully!${NC}"
    echo -e "${GREEN}Frontend is available at: http://localhost:$PORT${NC}"
    echo ""
    echo "Useful commands:"
    echo "  - View logs: $DOCKER_CMD logs -f $CONTAINER_NAME"
    echo "  - Stop container: $DOCKER_CMD stop $CONTAINER_NAME"
    echo "  - Remove container: $DOCKER_CMD rm $CONTAINER_NAME"
    echo "  - Restart container: $DOCKER_CMD restart $CONTAINER_NAME"
else
    echo -e "${RED}Error: Failed to start container.${NC}"
    exit 1
fi

