# constants.py

# Địa chỉ Server API
BASE_URL = "http://localhost:8000/api/v1"

# Các Endpoint
VEHICLE_ENDPOINT = f"{BASE_URL}/vehicles/"

# Timeout cho API (giây)
API_TIMEOUT = 5

# Các cấu hình khác nếu cần
HEADERS = {
    'accept': 'application/json',
    'Content-Type': 'application/json'
}