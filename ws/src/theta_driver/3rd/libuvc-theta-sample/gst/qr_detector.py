#!/usr/bin/env python3
"""
QR Code Detector cho Theta X Camera
Đọc video từ /dev/video2 và nhận diện QR code
"""

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import time
import sys
import os
from datetime import datetime

class QRDetector:
    def __init__(self, video_source="/dev/video2", display=True):
        """
        Khởi tạo QR Detector
        
        Args:
            video_source: Nguồn video (mặc định /dev/video2)
            display: Có hiển thị video hay không
        """
        self.video_source = video_source
        self.display = display
        self.cap = None
        self.running = False
        
    def initialize_camera(self):
        """Khởi tạo camera"""
        print(f"Đang kết nối với camera tại {self.video_source}...")
        
        # Thử mở camera
        self.cap = cv2.VideoCapture(self.video_source)
        
        if not self.cap.isOpened():
            print(f"Lỗi: Không thể mở camera tại {self.video_source}")
            print("Hãy chắc chắn rằng gst_loopback đang chạy và tạo /dev/video2")
            return False
            
        # Cấu hình camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        print("Camera đã được khởi tạo thành công!")
        return True
        
    def detect_qr_codes(self, frame):
        """
        Nhận diện QR code trong frame
        
        Args:
            frame: Frame video từ camera
            
        Returns:
            list: Danh sách QR codes được tìm thấy
        """
        # Chuyển đổi sang grayscale để tăng tốc độ xử lý
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Nhận diện QR codes
        qr_codes = pyzbar.decode(gray)
        
        return qr_codes
        
    def draw_qr_info(self, frame, qr_codes):
        """
        Vẽ thông tin QR code lên frame
        
        Args:
            frame: Frame gốc
            qr_codes: Danh sách QR codes được tìm thấy
            
        Returns:
            frame: Frame đã được vẽ thông tin
        """
        for qr in qr_codes:
            # Lấy tọa độ và dữ liệu
            rect = qr.rect
            data = qr.data.decode('utf-8')
            qr_type = qr.type
            
            # Vẽ hình chữ nhật bao quanh QR code
            cv2.rectangle(frame, (rect.left, rect.top), 
                         (rect.left + rect.width, rect.top + rect.height), 
                         (0, 255, 0), 2)
            
            # Vẽ text hiển thị dữ liệu QR code
            text = f"{qr_type}: {data}"
            cv2.putText(frame, text, (rect.left, rect.top - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # In thông tin ra console
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            print(f"[{timestamp}] QR Code phát hiện: {data}")
            
        return frame
        
    def run(self):
        """Chạy chương trình nhận diện QR code"""
        if not self.initialize_camera():
            return False
            
        self.running = True
        print("Bắt đầu nhận diện QR code...")
        print("Nhấn 'q' để thoát, 's' để chụp ảnh")
        
        frame_count = 0
        last_qr_time = 0
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                
                if not ret:
                    print("Không thể đọc frame từ camera")
                    break
                    
                frame_count += 1
                
                # Nhận diện QR code
                qr_codes = self.detect_qr_codes(frame)
                
                # Vẽ thông tin QR code lên frame
                if qr_codes:
                    frame = self.draw_qr_info(frame, qr_codes)
                    last_qr_time = time.time()
                    
                # Hiển thị thông tin trạng thái
                status_text = f"Frame: {frame_count} | QR Codes: {len(qr_codes)}"
                cv2.putText(frame, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Hiển thị video nếu được bật
                if self.display:
                    cv2.imshow('Theta X QR Code Detector', frame)
                    
                    # Xử lý phím bấm
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("Thoát chương trình...")
                        break
                    elif key == ord('s'):
                        # Chụp ảnh
                        filename = f"theta_qr_{int(time.time())}.jpg"
                        cv2.imwrite(filename, frame)
                        print(f"Đã lưu ảnh: {filename}")
                else:
                    # Nếu không hiển thị, chỉ in thông tin QR code
                    if qr_codes:
                        for qr in qr_codes:
                            data = qr.data.decode('utf-8')
                            print(f"QR Code: {data}")
                            
        except KeyboardInterrupt:
            print("\nDừng chương trình...")
            
        finally:
            self.cleanup()
            
        return True
        
    def cleanup(self):
        """Dọn dẹp tài nguyên"""
        self.running = False
        if self.cap:
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        print("Đã dọn dẹp tài nguyên")

def main():
    """Hàm main"""
    import argparse
    
    parser = argparse.ArgumentParser(description='QR Code Detector cho Theta X Camera')
    parser.add_argument('--video', '-v', default='/dev/video2', 
                       help='Nguồn video (mặc định: /dev/video2)')
    parser.add_argument('--no-display', action='store_true', 
                       help='Không hiển thị video (chỉ in kết quả)')
    parser.add_argument('--list-devices', action='store_true',
                       help='Liệt kê các thiết bị video có sẵn')
    
    args = parser.parse_args()
    
    if args.list_devices:
        print("Các thiết bị video có sẵn:")
        for i in range(10):  # Kiểm tra /dev/video0 đến /dev/video9
            device = f"/dev/video{i}"
            if os.path.exists(device):
                print(f"  {device}")
        return
    
    # Tạo và chạy QR detector
    detector = QRDetector(video_source=args.video, display=not args.no_display)
    detector.run()

if __name__ == "__main__":
    main()
