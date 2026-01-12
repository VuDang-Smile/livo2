import requests
import uuid
import sys
import subprocess
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS

def get_mac():
    # Lấy địa chỉ MAC của máy hiện tại
    mac = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
                    for ele in range(0, 8*6, 8)][::-1])
    return mac

def is_registered(mac):
    # Endpoint API của bạn, truyền biến mac vào URL
    vehicle_id = mac.lower().replace(':', '')[:12]
    print(f"Checking registration for MAC: {vehicle_id}")

    url = f'{VEHICLE_ENDPOINT}{vehicle_id}'
    headers = {
        'accept': 'application/json'
    }

    try:
        # Thực hiện gọi GET request tương đương với lệnh curl của bạn
        response = requests.get(url, headers=headers, timeout=5)
        data = response.json()
        

        # Kiểm tra phản hồi từ server
        if response.text and data.get("vehicle_id"):
            # Nếu API trả về 200, xe đã tồn tại trong hệ thống
            print(f"Thành công: Thiết bị {mac} đã được đăng ký.")
            return True
        else:
            print(f"Lỗi hệ thống: API trả về mã lỗi {data.get("vehicle_id")}")
            return False

    except requests.exceptions.RequestException as e:
        # Xử lý trường hợp không kết nối được server (Server die, mất mạng...)
        print(f"Không thể kết nối tới Server: {e}")
        return False

if __name__ == "__main__":
    user_mac = get_mac()
    if is_registered(user_mac):
        sys.exit(0) # Trả về code 0 nếu đã đăng ký
    else:
        sys.exit(1) # Trả về code 1 nếu chưa đăng ký