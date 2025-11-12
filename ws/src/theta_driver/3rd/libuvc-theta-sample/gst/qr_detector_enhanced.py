#!/usr/bin/env python3
"""
Enhanced QR Code Detector cho Theta X Camera 360
Tối ưu hóa khả năng quét QR code cho camera 360 với fisheye correction và equirectangular processing
"""

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from pyzbar.pyzbar import Decoded
import time
import sys
import os
import threading
from datetime import datetime
from collections import deque
import math
from scipy import ndimage

class EnhancedQRDetector:
    def __init__(self, video_source="/dev/video0", display=True, camera_type="360"):
        """
        Khởi tạo Enhanced QR Detector cho camera 360
        
        Args:
            video_source: Nguồn video (mặc định /dev/video0)
            display: Có hiển thị video hay không
            camera_type: Loại camera ("360", "fisheye", "normal")
        """
        self.video_source = video_source
        self.display = display
        self.camera_type = camera_type
        self.cap = None
        self.running = False
        
        # Threading
        self.qr_thread = None
        self.qr_running = False
        self.current_frame = None
        self.frame_lock = threading.Lock()
        
        # Performance tracking
        self.fps_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        self.qr_detection_count = 0
        self.qr_detection_start_time = time.time()
        self.qr_detection_fps = 0
        
        # QR detection settings
        self.qr_detection_interval = 1  # Detect QR mỗi frame cho camera 360
        self.frame_count = 0
        self.last_qr_codes = []
        self.last_qr_time = 0
        self.qr_detection_history = deque(maxlen=20)
        
        # Image preprocessing settings
        self.enable_preprocessing = True
        self.enable_multiple_scales = True
        self.enable_rotation = True
        self.enable_fisheye_correction = True
        self.enable_equirectangular_processing = True
        self.enable_adaptive_threshold = True
        
        # Camera 360 specific settings
        self.fisheye_calibration = None
        self.equirectangular_regions = []
        self.detection_regions = []
        self.adaptive_contrast = True
        self.contrast_alpha = 1.5  # Tăng contrast cho 360
        self.brightness_beta = 20  # Tăng brightness cho 360
        
        # 360-specific detection settings
        self.enable_360_optimization = True
        self.crop_center_region = True  # Crop vùng trung tâm
        self.center_crop_ratio = 0.6  # Tỷ lệ crop vùng trung tâm
        self.enable_perspective_correction = True  # Sửa perspective
        
        # Advanced detection settings
        self.enable_contour_detection = True
        self.enable_template_matching = True
        self.enable_multi_angle_detection = True
        self.detection_confidence_threshold = 0.7
        
        # Threading locks
        self.qr_lock = threading.Lock()
        
        # Display settings
        self.display_width = 1280  # Width cho display
        self.display_height = 640  # Height cho display
        self.enable_resize = True  # Bật resize cho camera 360
        
        # Pan and zoom settings
        self.pan_x = 0  # Pan offset X
        self.pan_y = 0  # Pan offset Y
        self.zoom_level = 100  # Zoom level (100 = 100%)
        self.enable_pan = False  # Bật pan mode
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        # Initialize camera-specific parameters
        self._init_camera_parameters()
        
    def _init_camera_parameters(self):
        """Khởi tạo các tham số đặc thù cho camera 360"""
        if self.camera_type == "360":
            # Tham số fisheye cho Theta X
            self.fisheye_calibration = {
                'K': np.array([[800, 0, 960], [0, 800, 480], [0, 0, 1]], dtype=np.float32),
                'D': np.array([0.1, 0.05, 0, 0], dtype=np.float32)
            }
            
            # Các vùng quan trọng trong equirectangular image
            self.equirectangular_regions = [
                {'name': 'front', 'x': 0, 'y': 0, 'w': 480, 'h': 480},
                {'name': 'back', 'x': 1440, 'y': 0, 'w': 480, 'h': 480},
                {'name': 'left', 'x': 480, 'y': 0, 'w': 480, 'h': 480},
                {'name': 'right', 'x': 960, 'y': 0, 'w': 480, 'h': 480}
            ]
            
            # Các vùng detection ưu tiên
            self.detection_regions = [
                {'name': 'center', 'x': 720, 'y': 240, 'w': 480, 'h': 480},
                {'name': 'top', 'x': 720, 'y': 0, 'w': 480, 'h': 240},
                {'name': 'bottom', 'x': 720, 'y': 480, 'w': 480, 'h': 240}
            ]
            
        elif self.camera_type == "fisheye":
            # Tham số fisheye chung
            self.fisheye_calibration = {
                'K': np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32),
                'D': np.array([0.2, 0.1, 0, 0], dtype=np.float32)
            }
            
    def initialize_camera(self):
        """Khởi tạo camera với cài đặt tối ưu cho QR detection 360"""
        print(f"Đang kết nối với camera 360 tại {self.video_source}...")
        
        self.cap = cv2.VideoCapture(self.video_source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print(f"Lỗi: Không thể mở camera tại {self.video_source}")
            return False
        
        # Cấu hình camera tối ưu cho camera 360
        if self.camera_type == "360":
            # Cài đặt cho Theta X (3840x1920 equirectangular)
            # Theta X không hỗ trợ thay đổi resolution, sẽ resize trong code
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1920)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)  # Tăng buffer cho 360 video 4K
            # Không set FOURCC vì Theta X chỉ hỗ trợ YU12
            
            # Cài đặt đặc thù cho camera 360
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Tắt autofocus
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Bật auto exposure cho 360
            self.cap.set(cv2.CAP_PROP_EXPOSURE, -4)  # Exposure vừa phải
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 60)  # Tăng brightness cho 360
            self.cap.set(cv2.CAP_PROP_CONTRAST, 60)  # Tăng contrast
            self.cap.set(cv2.CAP_PROP_SATURATION, 50)  # Điều chỉnh saturation
            self.cap.set(cv2.CAP_PROP_HUE, 0)  # Reset hue
            
        else:
            # Cài đặt cho camera thường (HP HD Camera)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            
            # Cài đặt cho camera thường
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # Bật auto exposure cho HP camera
            self.cap.set(cv2.CAP_PROP_EXPOSURE, -6)
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)
            self.cap.set(cv2.CAP_PROP_CONTRAST, 50)
        
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        print(f"Camera {self.camera_type} FPS thực tế: {actual_fps}")
        print(f"Độ phân giải: {actual_width}x{actual_height}")
        print("Camera đã được khởi tạo thành công!")
        return True
        
    def qr_detection_thread(self):
        """Thread riêng cho QR detection với tối ưu hóa"""
        print("Enhanced QR Detection thread started")
        
        while self.qr_running:
            try:
                # Lấy frame từ shared frame
                with self.frame_lock:
                    if self.current_frame is None:
                        time.sleep(0.01)
                        continue
                    frame = self.current_frame.copy()
                
                # Detect QR mỗi N frame
                if self.frame_count % self.qr_detection_interval == 0:
                    qr_codes = self.detect_qr_codes_enhanced(frame)
                    
                    with self.qr_lock:
                        if qr_codes:
                            self.last_qr_codes = qr_codes
                            self.last_qr_time = time.time()
                            
                            # In QR code mới
                            for qr in qr_codes:
                                data = qr.data.decode('utf-8')
                                timestamp = datetime.now().strftime("%H:%M:%S")
                                print(f"[{timestamp}] QR: {data}")
                            
                            # Cập nhật detection history
                            self.qr_detection_history.append(time.time())
                        else:
                            # Nếu không detect được, thử với frame gốc
                            qr_codes_raw = self.detect_qr_codes_raw(frame)
                            if qr_codes_raw:
                                self.last_qr_codes = qr_codes_raw
                                self.last_qr_time = time.time()
                                
                                for qr in qr_codes_raw:
                                    data = qr.data.decode('utf-8')
                                    timestamp = datetime.now().strftime("%H:%M:%S")
                                    print(f"[{timestamp}] QR (raw): {data}")
                
                self.frame_count += 1
                self.qr_detection_count += 1
                
                # Tính QR detection FPS
                current_time = time.time()
                if current_time - self.qr_detection_start_time >= 1.0:
                    self.qr_detection_fps = self.qr_detection_count / (current_time - self.qr_detection_start_time)
                    self.qr_detection_count = 0
                    self.qr_detection_start_time = current_time
                
                # Ngủ ngắn để không chiếm CPU
                time.sleep(0.001)
                
            except Exception as e:
                print(f"Lỗi trong QR detection thread: {e}")
                
        print("Enhanced QR Detection thread stopped")
        
    def preprocess_image(self, image):
        """Preprocessing image tối ưu cho camera 360"""
        if not self.enable_preprocessing:
            return image
            
        # Chuyển sang grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 1. Fisheye correction cho camera 360
        if self.camera_type == "360" and self.enable_fisheye_correction and self.fisheye_calibration:
            gray = self._correct_fisheye_distortion(gray)
        
        # 2. Adaptive contrast enhancement cho 360 video
        if self.adaptive_contrast:
            gray = self._adaptive_contrast_enhancement(gray)
        
        # 3. CLAHE với tile size phù hợp cho 360
        if self.camera_type == "360":
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(16,16))
        else:
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        
        # 4. Bilateral filter để giảm noise nhưng giữ edge
        gray = cv2.bilateralFilter(gray, 9, 75, 75)
        
        # 5. Adaptive threshold với tham số tối ưu cho 360
        if self.enable_adaptive_threshold:
            if self.camera_type == "360":
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                             cv2.THRESH_BINARY, 15, 3)
            else:
                thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                             cv2.THRESH_BINARY, 11, 2)
        else:
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 6. Morphological operations tối ưu cho QR code
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 1))
        
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel_close)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel_open)
        
        return thresh
    
    def _correct_fisheye_distortion(self, image):
        """Sửa fisheye distortion cho camera 360"""
        if not self.fisheye_calibration:
            return image
            
        try:
            K = self.fisheye_calibration['K']
            D = self.fisheye_calibration['D']
            
            # Undistort image
            undistorted = cv2.fisheye.undistortImage(image, K, D, Knew=K)
            return undistorted
        except Exception as e:
            print(f"Lỗi fisheye correction: {e}")
            return image
    
    def _adaptive_contrast_enhancement(self, image):
        """Tăng cường contrast thích ứng cho 360 video"""
        # Tính toán histogram
        hist = cv2.calcHist([image], [0], None, [256], [0, 256])
        
        # Tìm percentile để điều chỉnh contrast
        total_pixels = image.shape[0] * image.shape[1]
        cumsum = np.cumsum(hist)
        
        # Tìm 5% và 95% percentile
        low_thresh = np.where(cumsum >= total_pixels * 0.05)[0][0]
        high_thresh = np.where(cumsum >= total_pixels * 0.95)[0][0]
        
        # Normalize image
        normalized = np.clip((image - low_thresh) * 255.0 / (high_thresh - low_thresh), 0, 255)
        
        # Apply contrast enhancement
        enhanced = cv2.convertScaleAbs(normalized, alpha=self.contrast_alpha, beta=self.brightness_beta)
        
        return enhanced.astype(np.uint8)
    
    def _resize_frame_for_display(self, frame):
        """Resize frame cho display với pan và zoom"""
        height, width = frame.shape[:2]
        
        # Tính tỷ lệ resize cơ bản
        scale_w = self.display_width / width
        scale_h = self.display_height / height
        base_scale = min(scale_w, scale_h)
        
        # Áp dụng zoom level
        zoom_scale = self.zoom_level / 100.0
        scale = base_scale * zoom_scale
        
        new_width = int(width * scale)
        new_height = int(height * scale)
        
        # Resize frame
        resized = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
        
        # Tạo canvas với kích thước cố định
        canvas = np.zeros((self.display_height, self.display_width, 3), dtype=np.uint8)
        
        # Tính offset với pan
        y_offset = (self.display_height - new_height) // 2 + self.pan_y
        x_offset = (self.display_width - new_width) // 2 + self.pan_x
        
        # Đảm bảo không vượt quá canvas
        y_offset = max(0, min(y_offset, self.display_height - new_height))
        x_offset = max(0, min(x_offset, self.display_width - new_width))
        
        # Đặt frame resized vào canvas
        if y_offset + new_height <= self.display_height and x_offset + new_width <= self.display_width:
            canvas[y_offset:y_offset+new_height, x_offset:x_offset+new_width] = resized
        else:
            # Nếu frame quá lớn, crop nó
            crop_h = min(new_height, self.display_height - y_offset)
            crop_w = min(new_width, self.display_width - x_offset)
            canvas[y_offset:y_offset+crop_h, x_offset:x_offset+crop_w] = resized[:crop_h, :crop_w]
        
        return canvas
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Mouse callback cho pan và zoom"""
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.enable_pan:
                self.last_mouse_x = x
                self.last_mouse_y = y
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.enable_pan and flags & cv2.EVENT_FLAG_LBUTTON:
                # Pan
                dx = x - self.last_mouse_x
                dy = y - self.last_mouse_y
                self.pan_x += dx
                self.pan_y += dy
                self.last_mouse_x = x
                self.last_mouse_y = y
        elif event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:  # Scroll up - zoom in
                self.zoom_level = min(self.zoom_level + 10, 300)
            else:  # Scroll down - zoom out
                self.zoom_level = max(self.zoom_level - 10, 25)
            print(f"Zoom: {self.zoom_level}%")
        
    def detect_qr_codes_enhanced(self, frame):
        """Detect QR codes với tối ưu hóa nâng cao cho camera 360"""
        height, width = frame.shape[:2]
        qr_codes = []
        
        if self.camera_type == "360":
            # Xử lý đặc thù cho camera 360
            qr_codes.extend(self._detect_qr_360_optimized(frame))
        else:
            # Xử lý cho camera thường
            qr_codes.extend(self._detect_qr_normal(frame))
        
        # Loại bỏ duplicate QR codes
        unique_qr_codes = self.remove_duplicate_qr_codes(qr_codes)
        
        return unique_qr_codes
    
    def _detect_qr_360_optimized(self, frame):
        """Detect QR codes tối ưu cho camera 360"""
        qr_codes = []
        height, width = frame.shape[:2]
        
        # 1. Crop vùng trung tâm nếu cần (tập trung vào vùng quan trọng)
        if self.crop_center_region and width > 2000:
            crop_w = int(width * self.center_crop_ratio)
            crop_h = int(height * self.center_crop_ratio)
            start_x = (width - crop_w) // 2
            start_y = (height - crop_h) // 2
            center_frame = frame[start_y:start_y+crop_h, start_x:start_x+crop_w]
            
            # Detect trên vùng trung tâm
            center_qr = self.detect_qr_codes_raw(center_frame)
            for qr in center_qr:
                new_qr = Decoded(
                    type=qr.type,
                    data=qr.data,
                    rect=type(qr.rect)(
                        qr.rect.left + start_x,
                        qr.rect.top + start_y,
                        qr.rect.width,
                        qr.rect.height
                    ),
                    polygon=qr.polygon,
                    quality=qr.quality,
                    orientation=qr.orientation
                )
                qr_codes.append(new_qr)
        
        # 2. Detect trên toàn bộ frame với resize thông minh
        if width > 1600:  # Resize cho 360 video
            scale = 1600 / width
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame_resized = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
        else:
            frame_resized = frame
            scale = 1.0
            new_width = width
            new_height = height
        
        # 3. Detect trên các vùng quan trọng của 360 video
        if self.enable_equirectangular_processing and self.equirectangular_regions:
            for region in self.equirectangular_regions:
                x, y, w, h = region['x'], region['y'], region['w'], region['h']
                if x + w <= width and y + h <= height:
                    region_frame = frame[y:y+h, x:x+w]
                    if region_frame.size > 0:
                        # Detect trên vùng này
                        region_qr = self.detect_qr_codes_raw(region_frame)
                        # Điều chỉnh tọa độ về frame gốc
                        for qr in region_qr:
                            # Tạo QR object mới với tọa độ đã điều chỉnh
                            new_qr = Decoded(
                                type=qr.type,
                                data=qr.data,
                                rect=type(qr.rect)(
                                    qr.rect.left + x,
                                    qr.rect.top + y,
                                    qr.rect.width,
                                    qr.rect.height
                                ),
                                polygon=qr.polygon,
                                quality=qr.quality,
                                orientation=qr.orientation
                            )
                            qr_codes.append(new_qr)
        
        # 4. Detect trên frame gốc
        qr_codes.extend(self.detect_qr_codes_raw(frame_resized))
        
        # 5. Detect với preprocessing tối ưu cho 360
        if self.enable_preprocessing:
            processed = self.preprocess_image(frame_resized)
            qr_codes.extend(self.detect_qr_codes_raw(processed))
            
            # Thêm preprocessing với tham số khác cho 360
            if self.enable_360_optimization:
                # CLAHE với tile size lớn hơn cho 360
                clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(32,32))
                if len(processed.shape) == 3:
                    gray = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)
                else:
                    gray = processed
                enhanced = clahe.apply(gray)
                qr_codes.extend(self.detect_qr_codes_raw(enhanced))
        
        # 6. Detect với multiple scales tối ưu cho 360
        if self.enable_multiple_scales:
            scales = [0.4, 0.6, 0.8, 1.2, 1.5, 2.0]  # Nhiều scale hơn cho 360
            for scale_factor in scales:
                if scale_factor != 1.0:
                    # Sử dụng kích thước của frame_resized thay vì new_width/new_height
                    h, w = frame_resized.shape[:2]
                    new_w = int(w * scale_factor)
                    new_h = int(h * scale_factor)
                    if new_w > 100 and new_h > 50:  # Kích thước tối thiểu nhỏ hơn
                        frame_scaled = cv2.resize(frame_resized, (new_w, new_h), interpolation=cv2.INTER_AREA)
                        qr_codes.extend(self.detect_qr_codes_raw(frame_scaled))
        
        # 7. Detect với rotation tối ưu cho 360
        if self.enable_rotation:
            angles = [30, 45, 60, 90, 120, 135, 150, 180, 210, 225, 240, 270, 300, 315, 330]  # Nhiều góc hơn
            for angle in angles:
                h, w = frame_resized.shape[:2]
                center = (w // 2, h // 2)
                rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
                frame_rotated = cv2.warpAffine(frame_resized, rotation_matrix, (w, h))
                qr_codes.extend(self.detect_qr_codes_raw(frame_rotated))
        
        # 8. Detect với contour-based method
        if self.enable_contour_detection:
            qr_codes.extend(self._detect_qr_by_contours(frame_resized))
        
        # Scale lại tọa độ nếu đã resize
        if scale != 1.0:
            scaled_qr_codes = []
            for qr in qr_codes:
                new_qr = Decoded(
                    type=qr.type,
                    data=qr.data,
                    rect=type(qr.rect)(
                        int(qr.rect.left / scale),
                        int(qr.rect.top / scale),
                        int(qr.rect.width / scale),
                        int(qr.rect.height / scale)
                    ),
                    polygon=qr.polygon,
                    quality=qr.quality,
                    orientation=qr.orientation
                )
                scaled_qr_codes.append(new_qr)
            qr_codes = scaled_qr_codes
        
        return qr_codes
    
    def _detect_qr_normal(self, frame):
        """Detect QR codes cho camera thường"""
        qr_codes = []
        height, width = frame.shape[:2]
        
        # Resize frame để tăng tốc
        if width > 1200:
            scale = 1200 / width
            new_width = int(width * scale)
            new_height = int(height * scale)
            frame_resized = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)
        else:
            frame_resized = frame
            scale = 1.0
            new_width = width
            new_height = height
        
        # 1. Detect với frame gốc
        qr_codes.extend(self.detect_qr_codes_raw(frame_resized))
        
        # 2. Detect với preprocessing
        if self.enable_preprocessing:
            processed = self.preprocess_image(frame_resized)
            qr_codes.extend(self.detect_qr_codes_raw(processed))
        
        # 3. Detect với multiple scales
        if self.enable_multiple_scales:
            scales = [0.8, 1.2, 1.5]
            for scale_factor in scales:
                if scale_factor != 1.0:
                    # Sử dụng kích thước của frame_resized thay vì new_width/new_height
                    h, w = frame_resized.shape[:2]
                    new_w = int(w * scale_factor)
                    new_h = int(h * scale_factor)
                    if new_w > 100 and new_h > 100:
                        frame_scaled = cv2.resize(frame_resized, (new_w, new_h), interpolation=cv2.INTER_AREA)
                        qr_codes.extend(self.detect_qr_codes_raw(frame_scaled))
        
        # 4. Detect với rotation
        if self.enable_rotation:
            angles = [90, 180, 270]
            for angle in angles:
                h, w = frame_resized.shape[:2]
                center = (w // 2, h // 2)
                rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
                frame_rotated = cv2.warpAffine(frame_resized, rotation_matrix, (w, h))
                qr_codes.extend(self.detect_qr_codes_raw(frame_rotated))
        
        # Scale lại tọa độ nếu đã resize
        if scale != 1.0:
            scaled_qr_codes = []
            for qr in qr_codes:
                new_qr = Decoded(
                    type=qr.type,
                    data=qr.data,
                    rect=type(qr.rect)(
                        int(qr.rect.left / scale),
                        int(qr.rect.top / scale),
                        int(qr.rect.width / scale),
                        int(qr.rect.height / scale)
                    ),
                    polygon=qr.polygon,
                    quality=qr.quality,
                    orientation=qr.orientation
                )
                scaled_qr_codes.append(new_qr)
            qr_codes = scaled_qr_codes
        
        return qr_codes
    
    def _detect_qr_by_contours(self, image):
        """Detect QR codes bằng contour analysis"""
        qr_codes = []
        
        try:
            # Chuyển sang grayscale
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # Tìm contours
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Lọc contours có thể là QR code
            for contour in contours:
                # Tính diện tích
                area = cv2.contourArea(contour)
                if area < 100:  # Bỏ qua contour quá nhỏ
                    continue
                
                # Tính tỷ lệ aspect ratio
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                
                # QR code thường có aspect ratio gần 1:1
                if 0.8 <= aspect_ratio <= 1.2:
                    # Cắt vùng có thể chứa QR code
                    roi = gray[y:y+h, x:x+w]
                    if roi.size > 0:
                        # Thử detect QR code trong ROI này
                        roi_qr = self.detect_qr_codes_raw(roi)
                        for qr in roi_qr:
                            # Tạo QR object mới với tọa độ đã điều chỉnh
                            new_qr = Decoded(
                                type=qr.type,
                                data=qr.data,
                                rect=type(qr.rect)(
                                    qr.rect.left + x,
                                    qr.rect.top + y,
                                    qr.rect.width,
                                    qr.rect.height
                                ),
                                polygon=qr.polygon,
                                quality=qr.quality,
                                orientation=qr.orientation
                            )
                            qr_codes.append(new_qr)
        
        except Exception as e:
            print(f"Lỗi contour detection: {e}")
        
        return qr_codes
        
    def detect_qr_codes_raw(self, image):
        """Detect QR codes với image gốc"""
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
            
        # Detect QR codes
        qr_codes = pyzbar.decode(gray)
        return qr_codes
        
    def remove_duplicate_qr_codes(self, qr_codes):
        """Loại bỏ duplicate QR codes"""
        unique_codes = []
        seen_data = set()
        
        for qr in qr_codes:
            data = qr.data.decode('utf-8')
            if data not in seen_data:
                seen_data.add(data)
                unique_codes.append(qr)
                
        return unique_codes
        
    def draw_qr_info(self, frame):
        """Vẽ thông tin QR code lên frame (không vẽ khung)"""
        with self.qr_lock:
            qr_codes = self.last_qr_codes
            
        for qr in qr_codes:
            rect = qr.rect
            data = qr.data.decode('utf-8')
            
            # Chỉ in thông tin QR code ra console, không vẽ gì lên frame
            # Text sẽ được hiển thị ở góc trên màn hình thay vì vẽ khung
            pass
        
        return frame
        
    def calculate_fps(self):
        """Tính FPS"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.fps_start_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.fps_start_time)
            self.fps_counter = 0
            self.fps_start_time = current_time
            
        return self.current_fps
        
    def run(self):
        """Chạy chương trình enhanced"""
        if not self.initialize_camera():
            return False
            
        self.running = True
        self.qr_running = True
        
        # Khởi động QR detection thread
        self.qr_thread = threading.Thread(target=self.qr_detection_thread, daemon=True)
        self.qr_thread.start()
        
        print("Enhanced QR Detection cho Camera 360 đã sẵn sàng!")
        print("Tối ưu hóa QR detection cho 360:")
        print("  - Fisheye distortion correction")
        print("  - Equirectangular region processing")
        print("  - Adaptive contrast enhancement")
        print("  - Multiple scales detection (0.6x - 1.5x)")
        print("  - Multi-angle rotation detection")
        print("  - Contour-based detection")
        print("  - Enhanced preprocessing cho 360 video")
        print("  - Duplicate removal")
        print()
        print("Phím tắt: 'q' = thoát, 's' = chụp ảnh, 'p' = toggle preprocessing")
        print("         'f' = toggle fisheye correction, 'r' = toggle regions")
        print("         'z' = toggle resize, '+' = tăng size, '-' = giảm size")
        print("         '1' = 1280x640, '2' = 1920x960, '3' = 2560x1280")
        print("         'space' = toggle pan mode, '0' = reset pan/zoom")
        print("         Mouse drag = pan (khi pan mode ON)")
        print("         Mouse wheel = zoom in/out")
        print("         'x' = toggle center crop, 'y' = toggle 360 optimization")
        
        try:
            while self.running:
                # Đọc frame cho display
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Lưu frame cho QR detection thread
                with self.frame_lock:
                    self.current_frame = frame.copy()
                
                # Resize frame cho display nếu cần
                if self.enable_resize and self.camera_type == "360":
                    display_frame = self._resize_frame_for_display(frame)
                else:
                    display_frame = frame.copy()
                
                # Vẽ QR info lên frame
                display_frame = self.draw_qr_info(display_frame)
                
                # Hiển thị frame
                if self.display:
                    # Thêm thông tin FPS và settings
                    fps = self.calculate_fps()
                    status_text = f"FPS: {fps:.1f} | QR FPS: {self.qr_detection_fps:.1f} | Frame: {self.frame_count}"
                    cv2.putText(display_frame, status_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # Hiển thị thông tin QR code được detect (thay vì vẽ khung)
                    qr_info_height = 0
                    with self.qr_lock:
                        if self.last_qr_codes:
                            y_offset = 60
                            for i, qr in enumerate(self.last_qr_codes):
                                data = qr.data.decode('utf-8')
                                qr_text = f"QR {i+1}: {data[:30]}{'...' if len(data) > 30 else ''}"
                                cv2.putText(display_frame, qr_text, (10, y_offset), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                y_offset += 25
                            qr_info_height = len(self.last_qr_codes) * 25
                    
                    # Thêm thông tin pan/zoom (điều chỉnh vị trí dựa trên số QR code)
                    if self.camera_type == "360":
                        pan_y_pos = 60 + qr_info_height
                        pan_zoom_text = f"Pan: ({self.pan_x}, {self.pan_y}) | Zoom: {self.zoom_level}% | Pan Mode: {'ON' if self.enable_pan else 'OFF'}"
                        cv2.putText(display_frame, pan_zoom_text, (10, pan_y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    # Hiển thị settings cho camera 360 (điều chỉnh vị trí dựa trên QR info)
                    if self.camera_type == "360":
                        settings_y_pos = 60 + qr_info_height + 30
                        settings_text = f"Preprocessing: {'ON' if self.enable_preprocessing else 'OFF'} | Fisheye: {'ON' if self.enable_fisheye_correction else 'OFF'} | Regions: {'ON' if self.enable_equirectangular_processing else 'OFF'}"
                        cv2.putText(display_frame, settings_text, (10, settings_y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                        
                        settings_text2 = f"Multi-scale: {'ON' if self.enable_multiple_scales else 'OFF'} | Rotation: {'ON' if self.enable_rotation else 'OFF'} | Contour: {'ON' if self.enable_contour_detection else 'OFF'}"
                        cv2.putText(display_frame, settings_text2, (10, settings_y_pos + 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    else:
                        settings_y_pos = 60 + qr_info_height + 30
                        settings_text = f"Preprocessing: {'ON' if self.enable_preprocessing else 'OFF'} | Multi-scale: {'ON' if self.enable_multiple_scales else 'OFF'}"
                        cv2.putText(display_frame, settings_text, (10, settings_y_pos), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    # Tạo window với mouse callback
                    cv2.namedWindow('Enhanced QR Detector', cv2.WINDOW_NORMAL)
                    cv2.setMouseCallback('Enhanced QR Detector', self._mouse_callback)
                    cv2.imshow('Enhanced QR Detector', display_frame)
                    
                    # Xử lý phím bấm
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("Thoát chương trình...")
                        break
                    elif key == ord('s'):
                        filename = f"enhanced_qr_360_{int(time.time())}.jpg"
                        cv2.imwrite(filename, frame)
                        print(f"Đã lưu ảnh: {filename}")
                    elif key == ord('p'):
                        self.enable_preprocessing = not self.enable_preprocessing
                        print(f"Preprocessing: {'ON' if self.enable_preprocessing else 'OFF'}")
                    elif key == ord('f'):
                        self.enable_fisheye_correction = not self.enable_fisheye_correction
                        print(f"Fisheye correction: {'ON' if self.enable_fisheye_correction else 'OFF'}")
                    elif key == ord('r'):
                        self.enable_equirectangular_processing = not self.enable_equirectangular_processing
                        print(f"Equirectangular regions: {'ON' if self.enable_equirectangular_processing else 'OFF'}")
                    elif key == ord('c'):
                        self.enable_contour_detection = not self.enable_contour_detection
                        print(f"Contour detection: {'ON' if self.enable_contour_detection else 'OFF'}")
                    elif key == ord('m'):
                        self.enable_multiple_scales = not self.enable_multiple_scales
                        print(f"Multiple scales: {'ON' if self.enable_multiple_scales else 'OFF'}")
                    elif key == ord('t'):
                        self.enable_rotation = not self.enable_rotation
                        print(f"Rotation detection: {'ON' if self.enable_rotation else 'OFF'}")
                    elif key == ord('z'):
                        self.enable_resize = not self.enable_resize
                        print(f"Resize display: {'ON' if self.enable_resize else 'OFF'}")
                    elif key == ord('+') or key == ord('='):
                        self.display_width = min(self.display_width + 160, 2560)
                        self.display_height = min(self.display_height + 80, 1280)
                        print(f"Display size: {self.display_width}x{self.display_height}")
                    elif key == ord('-'):
                        self.display_width = max(self.display_width - 160, 640)
                        self.display_height = max(self.display_height - 80, 320)
                        print(f"Display size: {self.display_width}x{self.display_height}")
                    elif key == ord('1'):
                        self.display_width, self.display_height = 1280, 640
                        print(f"Display size: {self.display_width}x{self.display_height}")
                    elif key == ord('2'):
                        self.display_width, self.display_height = 1920, 960
                        print(f"Display size: {self.display_width}x{self.display_height}")
                    elif key == ord('3'):
                        self.display_width, self.display_height = 2560, 1280
                        print(f"Display size: {self.display_width}x{self.display_height}")
                    elif key == ord(' '):  # Space key
                        self.enable_pan = not self.enable_pan
                        print(f"Pan mode: {'ON' if self.enable_pan else 'OFF'}")
                    elif key == ord('0'):
                        self.pan_x = 0
                        self.pan_y = 0
                        self.zoom_level = 100
                        print("Reset pan/zoom")
                    elif key == ord('x'):
                        self.crop_center_region = not self.crop_center_region
                        print(f"Center crop: {'ON' if self.crop_center_region else 'OFF'}")
                    elif key == ord('y'):
                        self.enable_360_optimization = not self.enable_360_optimization
                        print(f"360 optimization: {'ON' if self.enable_360_optimization else 'OFF'}")
                else:
                    # Nếu không hiển thị, chỉ in thông tin QR code
                    with self.qr_lock:
                        if self.last_qr_codes:
                            for qr in self.last_qr_codes:
                                data = qr.data.decode('utf-8')
                                print(f"QR Code: {data}")
                
        except KeyboardInterrupt:
            print("\nDừng chương trình...")
            self.running = False
            self.qr_running = False
            
        finally:
            self.cleanup()
            
        return True
        
    def cleanup(self):
        """Dọn dẹp tài nguyên"""
        self.running = False
        self.qr_running = False
        
        # Đợi QR thread kết thúc
        if self.qr_thread and self.qr_thread.is_alive():
            self.qr_thread.join(timeout=1)
            
        if self.cap:
            self.cap.release()
        if self.display:
            cv2.destroyAllWindows()
        print("Đã dọn dẹp tài nguyên")

def main():
    """Hàm main"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Enhanced QR Code Detector cho Camera 360')
    parser.add_argument('--video', '-v', default='/dev/video0', 
                       help='Nguồn video (mặc định: /dev/video0)')
    parser.add_argument('--no-display', action='store_true', 
                       help='Không hiển thị video (chỉ in kết quả)')
    parser.add_argument('--camera-type', '-t', choices=['360', 'fisheye', 'normal'], 
                       default='360', help='Loại camera (mặc định: 360)')
    parser.add_argument('--qr-interval', '-i', type=int, default=1,
                       help='Khoảng cách detect QR (mặc định: 1 frame cho 360)')
    parser.add_argument('--no-preprocessing', action='store_true',
                       help='Tắt image preprocessing')
    parser.add_argument('--no-multiscale', action='store_true',
                       help='Tắt multiple scales detection')
    parser.add_argument('--no-rotation', action='store_true',
                       help='Tắt rotation detection')
    parser.add_argument('--no-fisheye', action='store_true',
                       help='Tắt fisheye correction')
    parser.add_argument('--no-regions', action='store_true',
                       help='Tắt equirectangular region processing')
    parser.add_argument('--no-contour', action='store_true',
                       help='Tắt contour-based detection')
    parser.add_argument('--list-devices', action='store_true',
                       help='Liệt kê các thiết bị video có sẵn')
    
    args = parser.parse_args()
    
    if args.list_devices:
        print("Các thiết bị video có sẵn:")
        for i in range(10):
            device = f"/dev/video{i}"
            if os.path.exists(device):
                print(f"  {device}")
        return
    
    # Tạo và chạy QR detector
    detector = EnhancedQRDetector(
        video_source=args.video, 
        display=not args.no_display,
        camera_type=args.camera_type
    )
    detector.qr_detection_interval = args.qr_interval
    detector.enable_preprocessing = not args.no_preprocessing
    detector.enable_multiple_scales = not args.no_multiscale
    detector.enable_rotation = not args.no_rotation
    detector.enable_fisheye_correction = not args.no_fisheye
    detector.enable_equirectangular_processing = not args.no_regions
    detector.enable_contour_detection = not args.no_contour
    detector.run()

if __name__ == "__main__":
    main()
