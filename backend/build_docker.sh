#!/bin/bash

# Build Docker image script for Livo Backend

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

IMAGE_NAME="livo-backend"
IMAGE_TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}"

echo "=========================================="
echo "Building Docker image: ${FULL_IMAGE_NAME}"
echo "=========================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Dockerfile exists
if [ ! -f "Dockerfile" ]; then
    echo "ERROR: Dockerfile not found in current directory."
    exit 1
fi

# Validate Dockerfile syntax (basic check)
if ! docker build --dry-run -t test . &> /dev/null; then
    echo "WARNING: Dockerfile validation failed, but continuing..."
fi

# Build the image
echo "Building image..."
docker build -t "${FULL_IMAGE_NAME}" .

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ Build successful!"
    echo "Image: ${FULL_IMAGE_NAME}"
    echo "=========================================="
    echo ""
    echo "To run the container:"
    echo "  ./start_docker.sh"
    echo ""
    echo "Or manually:"
    echo "  docker compose up -d"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "❌ Build failed!"
    echo "=========================================="
    exit 1
fi

