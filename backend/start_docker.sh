#!/bin/bash

# Start Docker containers script for Livo Backend

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "Starting Livo Backend Services"
echo "=========================================="

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if docker compose is available (Docker Compose V2)
if ! docker compose version &> /dev/null; then
    echo "ERROR: docker compose is not available. Please install Docker Compose V2."
    exit 1
fi

# Check Docker permissions and set DOCKER_CMD
DOCKER_CMD="docker"
if ! docker ps &> /dev/null; then
    echo ""
    echo "⚠️  No Docker permissions detected."
    echo "Requesting sudo access for Docker commands..."
    echo ""
    # Test sudo access
    if ! sudo -n true 2>/dev/null; then
        echo "Please enter your password to use Docker with sudo:"
        sudo -v
    fi
    DOCKER_CMD="sudo docker"
    echo "✅ Using sudo for Docker commands"
    echo ""
fi

# Check if docker-compose.yml exists
if [ ! -f "docker-compose.yml" ]; then
    echo "ERROR: docker-compose.yml not found in current directory."
    exit 1
fi

# Check ports availability
check_port() {
    local port=$1
    local service=$2
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1 ; then
        echo "WARNING: Port $port is already in use. $service may not start correctly."
    fi
}

echo "Checking ports..."
check_port 8000 "FastAPI Backend"
check_port 27017 "MongoDB"
check_port 1883 "MQTT Broker"

# Create volumes if they don't exist
echo "Setting up volumes..."
mkdir -p uploads processed

# Wait for MongoDB to be ready function
wait_for_mongodb() {
    echo "Waiting for MongoDB to be ready..."
    local max_attempts=30
    local attempt=0
    
    while [ $attempt -lt $max_attempts ]; do
        if $DOCKER_CMD exec livo-mongodb mongosh --eval "db.adminCommand('ping')" &> /dev/null; then
            echo "✅ MongoDB is ready!"
            return 0
        fi
        attempt=$((attempt + 1))
        echo "  Attempt $attempt/$max_attempts..."
        sleep 2
    done
    
    echo "⚠️  MongoDB may not be ready yet, but continuing..."
    return 1
}

# Start containers
echo ""
echo "Starting containers with docker compose..."
$DOCKER_CMD compose up -d

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to start containers."
    exit 1
fi

# Wait for MongoDB
wait_for_mongodb

# Health check
echo ""
echo "Performing health checks..."

# Check FastAPI
sleep 3
if curl -f http://localhost:8000/health &> /dev/null; then
    echo "✅ FastAPI Backend is healthy"
else
    echo "⚠️  FastAPI Backend health check failed (may still be starting)"
fi

# Check MongoDB
if $DOCKER_CMD exec livo-mongodb mongosh --eval "db.adminCommand('ping')" &> /dev/null; then
    echo "✅ MongoDB is healthy"
else
    echo "⚠️  MongoDB health check failed"
fi

echo ""
echo "=========================================="
echo "✅ Services started!"
echo "=========================================="
echo ""
echo "Services:"
echo "  - FastAPI Backend: http://localhost:8000"
echo "  - Swagger UI:      http://localhost:8000/docs"
echo "  - ReDoc:           http://localhost:8000/redoc"
echo "  - Health Check:    http://localhost:8000/health"
echo "  - MongoDB:         localhost:27017"
echo "  - MQTT Broker:     localhost:1883"
echo ""
echo "To view logs:"
echo "  $DOCKER_CMD compose logs -f"
echo ""
echo "To stop services:"
echo "  $DOCKER_CMD compose down"
echo ""

