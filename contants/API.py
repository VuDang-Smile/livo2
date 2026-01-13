"""
API Configuration Constants
"""
import os

# Backend API endpoint - có thể override bằng environment variable
BACKEND_HOST = os.getenv("BACKEND_HOST", os.getenv("LIVO_BACKEND_URL", "http://backend.lidar.tm"))
VEHICLE_ENDPOINT = f"{BACKEND_HOST}/api/v1/vehicles/"

# API timeout (seconds)
API_TIMEOUT = int(os.getenv("API_TIMEOUT", "5"))

# Default headers for API requests
HEADERS = {
    'accept': 'application/json',
    'Content-Type': 'application/json'
}

