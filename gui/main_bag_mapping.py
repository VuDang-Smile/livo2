import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
import time
from pathlib import Path
import sys
import subprocess
import os
import io
import threading
import signal
import zipfile
import json
import uuid
# Import numpy trước (an toàn hơn)
import numpy as np

# Delay import cv2 để tránh segfault - import với error handling tốt hơn
cv2 = None
try:
    # Thử import cv2 với error handling đầy đủ
    import cv2
    # Test import thành công bằng cách gọi một hàm đơn giản
    _ = cv2.__version__
except (ImportError, SystemError, OSError, AttributeError) as e:
    print(f"Warning: cv2 not available: {e}. Some features will be disabled.")
    cv2 = None
except Exception as e:
    # Catch tất cả các exception khác để tránh segfault
    print(f"Warning: cv2 import failed with unexpected error: {e}. Some features will be disabled.")
    cv2 = None

from concurrent.futures import ThreadPoolExecutor
import contextlib
import io
import warnings
import ctypes
import sys

# Import Translator với error handling tốt hơn
try:
    sys.path.insert(0, str(Path(__file__).parent.parent / "languages"))
    from translate_engine import Translator
except (ImportError, SystemError, OSError) as e:
    print(f"Error: Cannot import Translator: {e}")
    # Fallback Translator class
    class Translator:
        def __init__(self, lang='en'):
            self.lang = lang
        def get(self, key, default=None):
            return default or key
        def switch_language(self, lang):
            self.lang = lang
    Translator = Translator

# Import comparison utils cho tính năng so sánh bản đồ thiết kế
COMPARISON_UTILS_AVAILABLE = False
try:
    # Thêm đường dẫn scripts để import pcd_comparison_utils
    scripts_dir = Path(__file__).parent.parent / "scripts"
    if str(scripts_dir) not in sys.path:
        sys.path.insert(0, str(scripts_dir))

    from pcd_comparison_utils import (
        load_pcd,
        load_obj_as_pointcloud,
        preprocess_pcd,
        compute_similarity_metrics,
        calculate_similarity_percentage,
        detect_drift,
        DEFAULT_ICP_MAX_ITERATIONS,
        DEFAULT_ICP_THRESHOLD_MULTIPLIER,
    )
    COMPARISON_UTILS_AVAILABLE = True
except (ImportError, SystemError, OSError, SystemExit) as e:
    # Thiếu open3d hoặc file utils – chỉ log cảnh báo, không làm crash GUI
    print(f"Warning: pcd_comparison_utils not available: {e}. Design map comparison will be disabled.")
    COMPARISON_UTILS_AVAILABLE = False

# Try to import pyzbar for QR code scanning với error handling tốt hơn
PYZBAR_AVAILABLE = False
QRCODE_SYMBOLS = None
pyzbar = None
try:
    from pyzbar import pyzbar
    from pyzbar.pyzbar import ZBarSymbol
    PYZBAR_AVAILABLE = True
    # Chỉ decode QRCODE để tránh warnings từ DataBar decoder
    QRCODE_SYMBOLS = [ZBarSymbol.QRCODE]
except (ImportError, SystemError, OSError, AttributeError) as e:
    PYZBAR_AVAILABLE = False
    QRCODE_SYMBOLS = None
    pyzbar = None
    print(f"Warning: pyzbar not available: {e}. QR code scanning will be disabled.")

# Import geometry utils với error handling tốt hơn
GEOMETRY_UTILS_AVAILABLE = False
get_camera_matrix = None
get_extrinsic_matrix = None
camera_to_world = None
cartesian_to_spherical = None
spherical2equirect = None
try:
    from geometry_utils import (
        get_camera_matrix, get_extrinsic_matrix, 
        camera_to_world, cartesian_to_spherical, spherical2equirect
    )
    GEOMETRY_UTILS_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError) as e:
    GEOMETRY_UTILS_AVAILABLE = False
    print(f"Warning: geometry_utils not available: {e}. QR code scanning will be disabled.")

# Try to import requests for backend upload
REQUESTS_AVAILABLE = False
requests = None
try:
    import requests
    REQUESTS_AVAILABLE = True
except (ImportError, SystemError, OSError):
    REQUESTS_AVAILABLE = False
    requests = None

# Try to import ROS2 với error handling tốt hơn
ROS2_AVAILABLE = False
rclpy = None
Node = None
SingleThreadedExecutor = None
Image = None
Odometry = None
CvBridge = None
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError) as e:
    ROS2_AVAILABLE = False
    print(f"Warning: ROS2 not available: {e}. QR code scanning from bag will be disabled.")

def _decode_qr_silently(gray_image):
    """Wrapper function để decode QRCODE với suppress warnings hoàn toàn"""
    if not PYZBAR_AVAILABLE or pyzbar is None:
        return []
    
    decoded_objects = []
    
    # Suppress warnings ở nhiều mức
    if sys.platform == 'linux':
        # Linux: redirect stderr ở OS level
        saved_stderr = None
        devnull_fd = None
        try:
            devnull_fd = os.open(os.devnull, os.O_WRONLY)
            stderr_fd = sys.stderr.fileno()
            saved_stderr = os.dup(stderr_fd)
            os.dup2(devnull_fd, stderr_fd)
            os.close(devnull_fd)
            devnull_fd = None
            
            # Decode chỉ QRCODE
            try:
                decoded_objects = pyzbar.decode(gray_image, symbols=QRCODE_SYMBOLS)
            except (TypeError, AttributeError):
                # Fallback: decode tất cả rồi filter
                decoded_objects = pyzbar.decode(gray_image)
                decoded_objects = [obj for obj in decoded_objects if obj.type == 'QRCODE']
        except Exception:
            pass
        finally:
            # Restore stderr
            if saved_stderr is not None:
                try:
                    stderr_fd = sys.stderr.fileno()
                    os.dup2(saved_stderr, stderr_fd)
                    os.close(saved_stderr)
                except:
                    pass
            if devnull_fd is not None:
                try:
                    os.close(devnull_fd)
                except:
                    pass
    else:
        # Windows/Mac: dùng redirect_stderr
        stderr_buffer = io.StringIO()
        with contextlib.redirect_stderr(stderr_buffer):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                try:
                    decoded_objects = pyzbar.decode(gray_image, symbols=QRCODE_SYMBOLS)
                except (TypeError, AttributeError):
                    decoded_objects = pyzbar.decode(gray_image)
                    decoded_objects = [obj for obj in decoded_objects if obj.type == 'QRCODE']
                except Exception:
                    decoded_objects = []
    
    return decoded_objects


class QRScanner:
    """Class để quét QR code từ ảnh equirectangular"""
    
    def __init__(self, perspec_size=600, max_workers=4):
        self.perspec_size = perspec_size
        self.max_workers = max_workers
        self._remap_cache = {}
        
        # Kiểm tra tất cả dependencies trước khi enable
        if not PYZBAR_AVAILABLE or not GEOMETRY_UTILS_AVAILABLE or cv2 is None:
            self.enabled = False
        else:
            self.enabled = True
    
    def Equirec2Perspec(self, img: np.ndarray,
                     FOV: float,
                     THETA: float,
                     PHI: float,
                     height: int,
                     width: int) -> np.ndarray:
        """
        Convert equirectangular image to perspective view with caching for speed.
        """
        if cv2 is None or not GEOMETRY_UTILS_AVAILABLE:
            raise RuntimeError("cv2 or geometry_utils not available")
        
        key = (THETA, PHI, FOV, height, width)
        if key in self._remap_cache:
            XY = self._remap_cache[key]
        else:
            # Convert angles to radians
            FOV_rad = np.deg2rad(FOV)
            THETA_rad = np.deg2rad(THETA)
            PHI_rad = np.deg2rad(PHI)

            img_height, img_width = img.shape[:2]
            K = get_camera_matrix(FOV_rad, width, height)
            R = get_extrinsic_matrix(THETA_rad, PHI_rad)

            # Image grid
            x, y = np.meshgrid(np.arange(width), np.arange(height))
            z = np.ones_like(x)
            xyz = np.stack([x, y, z], axis=-1)

            world_coords = camera_to_world(xyz, K, R)
            sp_coords = cartesian_to_spherical(world_coords)
            XY = spherical2equirect(sp_coords, img_width, img_height)

            self._remap_cache[key] = XY  # cache for speed

        persp = cv2.remap(img, XY[..., 0], XY[..., 1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP)
        return persp

    def scan_qr_codes(self, frame):
        """Quét QR code từ ảnh equirectangular - Logic từ nhánh khanhbv/feature/merge_code_scan_qr"""
        if not self.enabled:
            return []
        
        try:
            # --- Vòng 1: 6 view, zoom cao để quét QR gần ---
            views_round1 = [
                (0, 0),      # front
                (180, 0),    # back
                (90, 0),     # right
                (-90, 0),    # left
                (0, 90),     # up
                (0, -90),    # down
            ]
            zoom_round1 = [90, 70, 50]

            # --- Vòng 2: 12 view, zoom thấp để quét QR xa ---
            views_round2 = [
                (0, 0), (180, 0), (90, 0), (-90, 0),
                (0, 90), (0, -90),
                (45, 0), (-45, 0), (0, 45), (0, -45),
                (135, 0), (-135, 0)
            ]
            zoom_round2 = [30, 15, 7, 5]

            rounds = [
                (views_round1, zoom_round1),
                (views_round2, zoom_round2)
            ]

            all_results = []
            debug_view = None

            for views, zoom_levels in rounds:
                for fov in zoom_levels:
                    if all_results:  # dừng nếu đã quét QR
                        break

                    def process_view(args):
                        theta, phi = args
                        if cv2 is None:
                            return [], None
                        
                        try:
                            persp = self.Equirec2Perspec(frame, fov, theta, phi,
                                                        self.perspec_size, self.perspec_size)
                            gray = cv2.cvtColor(persp, cv2.COLOR_BGR2GRAY)
                            # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
                            # gray = clahe.apply(gray)
                            # gray = cv2.GaussianBlur(gray, (3,3), 0)
                            
                            # Sử dụng wrapper function để suppress warnings hoàn toàn
                            decoded_objects = _decode_qr_silently(gray)
                            
                            # Tất cả decoded_objects đã là QRCODE
                            qr_codes = decoded_objects
                            
                            results = []
                            debug_img = None

                            for qr in qr_codes:
                                results.append({
                                    "data": qr.data.decode('utf-8'),
                                    "rect": {
                                        "left": qr.rect.left,
                                        "top": qr.rect.top,
                                        "width": qr.rect.width,
                                        "height": qr.rect.height
                                    },
                                    "view": (theta, phi),
                                    "zoom": fov
                                })

                                # Tạo debug image nếu cần (giống logic từ nhánh)
                                x, y, w, h = qr.rect.left, qr.rect.top, qr.rect.width, qr.rect.height
                                debug_img = persp.copy()
                                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                                cv2.putText(debug_img, qr.data.decode('utf-8'), (x, y - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            return results, debug_img  # trả về debug image
                        except Exception as e:
                            # Tránh crash nếu có lỗi trong quá trình xử lý
                            return [], None

                    with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
                        futures = executor.map(process_view, views)
                        for r, dbg in futures:
                            all_results.extend(r)
                            if dbg is not None and debug_view is None:
                                debug_view = dbg  # main thread quyết định hiển thị

                    # Hiển thị GUI chỉ trong main thread (có thể bật nếu cần debug)
                    # if debug_view is not None:
                    #      cv2.imshow("QR Debug View", debug_view)
                    #      cv2.waitKey(1)

                if all_results:  # dừng vòng nếu đã quét QR
                    break

            return all_results

        except Exception as e:
            print(f'Error scanning QR codes: {str(e)}')
            import traceback
            traceback.print_exc()
            return []


class QRImageSubscriberNode(Node):
    """ROS2 Node để subscribe image topic và odometry cho QR scanning"""
    
    def __init__(self, image_topic, odom_topic, image_callback, odom_callback, node_name='qr_image_subscriber'):
        super().__init__(node_name)
        
        # Đảm bảo topic name có / prefix
        if not image_topic.startswith('/'):
            image_topic = '/' + image_topic
        if not odom_topic.startswith('/'):
            odom_topic = '/' + odom_topic
        
        # Subscribe image topic
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        # Subscribe odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.image_callback_func = image_callback
        self.odom_callback_func = odom_callback
        self.image_topic_name = image_topic
        self.odom_topic_name = odom_topic
        self.get_logger().info(f'QR Scanner subscribed to image topic {image_topic} and odom topic {odom_topic}')
    
    def image_callback(self, msg):
        """Callback when receiving image"""
        try:
            if cv2 is None or CvBridge is None:
                return
            
            # Handle JPEG compressed images
            if msg.encoding.lower() in ['jpeg', 'jpg']:
                # Decode JPEG data directly from compressed format
                jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)
                if cv_image is None:
                    raise ValueError("Failed to decode JPEG image")
                # Convert BGR to RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                # Use cv_bridge for standard encodings
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Gọi callback với ảnh đã decode
            if self.image_callback_func:
                self.image_callback_func(cv_image)
            
        except Exception as e:
            error_msg = f'Error processing image from {self.image_topic_name}: {e}'
            if hasattr(self, 'get_logger'):
                self.get_logger().error(error_msg)
            print(f'[QRImageSubscriberNode] ERROR: {error_msg}')
    
    def odom_callback(self, msg):
        """Callback when receiving odometry"""
        try:
            # Extract position [x, y, z]
            position = [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]
            
            # Gọi callback với position
            if self.odom_callback_func:
                self.odom_callback_func(position)
            
        except Exception as e:
            error_msg = f'Error processing odometry from {self.odom_topic_name}: {e}'
            self.get_logger().error(error_msg)
            print(f'[QRImageSubscriberNode] ERROR: {error_msg}')


class BagMappingInterface:
    def __init__(self, root):
        self.root = root
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
        self.bag_path = None
        self.config_path = None
        
        self.mapping_process = None
        self.is_mapping_running = False
        # Flag để phân biệt crash thực sự và graceful shutdown từ STOP button
        self.is_stopping = False
        # Thêm process cho ros2 bag play
        self.bag_process = None
        self.is_bag_playing = False
        self.use_rviz = False
        # Bag rate mặc định, dùng cho ros2 bag play
        self.bag_rate = 0.5
        # Backend URL for map upload
        self.backend_base_url = os.environ.get("LIVO_BACKEND_URL", "http://backend.lidar.tm")
        # Thư mục lưu PCD convert từ OBJ (bản thiết kế)
        self.obj_converted_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "obj_converted_pcd"
        # Cấu hình so sánh bản đồ thiết kế
        self.design_file_type = "pcd"  # 'pcd' hoặc 'obj'
        self.design_file_path = None
        self.comparison_result = None
        self.comparison_threshold = 90.0  # % khớp tối thiểu khuyến nghị
        
        # QR code scanning - chỉ khởi tạo nếu tất cả dependencies có sẵn
        try:
            if PYZBAR_AVAILABLE and GEOMETRY_UTILS_AVAILABLE and cv2 is not None:
                self.qr_scanner = QRScanner()
            else:
                self.qr_scanner = None
                if not PYZBAR_AVAILABLE:
                    print("Warning: pyzbar not available. QR scanning disabled.")
                if not GEOMETRY_UTILS_AVAILABLE:
                    print("Warning: geometry_utils not available. QR scanning disabled.")
                if cv2 is None:
                    print("Warning: cv2 not available. QR scanning disabled.")
        except Exception as e:
            print(f"Warning: Failed to initialize QRScanner: {e}. QR scanning disabled.")
            self.qr_scanner = None
        self.detected_qr_codes = []  # List of detected QR codes
        self.qr_codes_lock = threading.Lock()  # Lock for thread-safe access
        
        # ROS2 subscriber for QR scanning during bag playback
        self.qr_scan_enabled = True  # Enable QR scanning during bag playback
        self.qr_scan_frame_interval = 10  # Scan QR every N frames (default: every 10 frames)
        self.qr_image_subscriber = None
        self.qr_ros_node = None
        self.qr_ros_executor = None
        self.qr_ros_thread = None
        self.qr_frame_count = 0
        
        # Current device position for QR detection
        self.current_position = None  # [x, y, z]
        self.position_lock = threading.Lock()  # Lock for thread-safe access
        self.qr_detect_json_path = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "QR_detect.json"
        
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        # Queue system cho smooth updates
        self.log_queue = []
        self.progress_queue = None  # (target_value, current_value)
        self.log_update_lock = threading.Lock()
        self.log_batch_size = 5  # Batch log updates để mượt hơn
        self.progress_animation_speed = 2  # Tốc độ animation progress bar (giá trị mỗi frame)
        
        # Start log update thread
        self.log_update_thread_running = True
        threading.Thread(target=self._log_update_worker, daemon=True).start()
        threading.Thread(target=self._progress_update_worker, daemon=True).start()
        
        self.setup_language_button()
        self.update_ui_texts()
        
        self.root.geometry("1150x850")

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Workspace ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 1. Card Control (Start Mapping)
        self.setup_control_card()

        # 2. Card Visualization (Preview / RViz) - Bao gồm cả progress và log section
        self.setup_preview_rviz_section()
        
        # Set default config
        self.set_default_config()
    
    def setup_language_button(self):
        lang_frame = tk.Frame(self.root)
        lang_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button = tk.Button(
            lang_frame,
            text=f"🌐 {lang_names.get(self.current_lang, 'English')}",
            font=("Arial", 9, "bold"),
            fg="#0066cc",
            bg="#e8e8e8",
            activebackground="#d0d0d0",
            activeforeground="#0066cc",
            relief=tk.RAISED,
            bd=1,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.toggle_language
        )
        self.lang_button.pack(side=tk.RIGHT)
    
    def toggle_language(self):
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        self.root.title(self.translator.get('title.bag_mapping_system', 'Bag Mapping System'))
        
        if hasattr(self, 'control_card'):
            self.control_card.config(text=self.translator.get('label.execution_control', 'Execution Control'))
        
        if hasattr(self, 'select_bag_label'):
            self.select_bag_label.config(text=self.translator.get('label.select_bag_file', 'Select Bag File:'))
        
        if hasattr(self, 'browse_btn'):
            self.browse_btn.config(text=self.translator.get('button.browse', 'Browse'))
        
        if hasattr(self, 'btn_stop'):
            self.btn_stop.config(text=self.translator.get('button.stop_process', '■ STOP'))
        
        if hasattr(self, 'status_label'):
            current_status = self.status_label.cget('text')
            if 'Mapping node running' in current_status or 'マッピングノード実行中' in current_status:
                self.status_label.config(text=self.translator.get('label.status_mapping_running', 'Status: 📡 Mapping node running'))
            elif 'Mapping' in current_status and 'node' not in current_status:
                self.status_label.config(text=self.translator.get('label.status_mapping', 'Status: Mapping...'))
            elif 'Map Created' in current_status or 'マップ作成済み' in current_status:
                self.status_label.config(text=self.translator.get('label.status_map_created', 'Status: Map Created'))
            elif 'Stopped' in current_status or '停止' in current_status:
                self.status_label.config(text=self.translator.get('label.status_stopped', 'Status: Stopped'))
            else:
                self.status_label.config(text=self.translator.get('label.status_ready', 'Status: Ready'))
        
        if hasattr(self, 'btn_start'):
            self.btn_start.config(text=self.translator.get('button.start_mapping', '🚀 START MAPPING'))
        
        if hasattr(self, 'preview_card'):
            self.preview_card.config(text=self.translator.get('label.visualization_preview', 'Visualization Preview'))
        
        if hasattr(self, 'rviz_check'):
            self.rviz_check.config(text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'))
        
        if hasattr(self, 'btn_open_rviz'):
            self.btn_open_rviz.config(text=self.translator.get('button.launch_rviz2', 'Launch RViz2'))
        
        if hasattr(self, 'preview_label'):
            current_text = self.preview_label.cget('text')
            if 'RViz2' in current_text:
                self.preview_label.config(text=self.translator.get('label.rviz2_redirect_active', 'RViz2 REDIRECT ACTIVE'))
            else:
                self.preview_label.config(text=self.translator.get('label.internal_viewport_active', 'INTERNAL VIEWPORT ACTIVE'))
        
        if hasattr(self, 'btn_upload'):
            self.btn_upload.config(text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'))
        
        if hasattr(self, 'upload_stat'):
            current_text = self.upload_stat.cget('text')
            if 'Ready' in current_text:
                self.upload_stat.config(text=self.translator.get('label.ready_to_upload', 'Ready to upload'))
            elif 'Done' in current_text or '100%' in current_text:
                self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            else:
                self.upload_stat.config(text=self.translator.get('label.waiting', 'Waiting...'))
        
        if hasattr(self, 'system_log_label'):
            self.system_log_label.config(text=self.translator.get('label.system_log', 'SYSTEM LOG'))
        
        if hasattr(self, 'clear_log_btn'):
            self.clear_log_btn.config(text=self.translator.get('button.clear_log', 'Clear Log'))
        
        if hasattr(self, 'qr_listbox'):
            # QR codes section sẽ được cập nhật tự động khi có QR codes mới
            pass
        
        # Update Design Map Comparison section
        if hasattr(self, 'comparison_frame'):
            self.comparison_frame.config(
                text=self.translator.get('label.design_map_comparison', 'Design Map Comparison')
            )
        
        if hasattr(self, 'design_file_type_label'):
            self.design_file_type_label.config(
                text=self.translator.get('label.design_file_type', 'Design file type:')
            )
        
        if hasattr(self, 'design_file_label'):
            self.design_file_label.config(
                text=self.translator.get('label.design_file', 'Design file:')
            )
        
        if hasattr(self, 'design_browse_btn'):
            self.design_browse_btn.config(
                text=self.translator.get('button.browse', 'Browse')
            )
        
        # Update comparison status label if exists (only if default text)
        if hasattr(self, 'comparison_status_label'):
            current_text = self.comparison_status_label.cget('text')
            # Only update if it's the default "not configured" text
            if 'not configured' in current_text.lower() or '未設定' in current_text:
                self.comparison_status_label.config(
                    text=self.translator.get('label.design_comparison_not_configured', 
                        'Design comparison: not configured')
                )

    def setup_control_card(self):
        self.control_card = tk.LabelFrame(self.workspace, text=self.translator.get('label.execution_control', 'Execution Control'), padx=15, pady=10)
        self.control_card.pack(fill=tk.X, pady=(0, 10))
        
        # File Path
        self.select_bag_label = tk.Label(self.control_card, text=self.translator.get('label.select_bag_file', 'Select Bag File:'))
        self.select_bag_label.pack(anchor="w")
        path_frame = tk.Frame(self.control_card)
        path_frame.pack(fill=tk.X, pady=5)
        self.bag_path_var = tk.StringVar()
        tk.Entry(path_frame, textvariable=self.bag_path_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.browse_btn = tk.Button(path_frame, text=self.translator.get('button.browse', 'Browse'), command=self.browse_file)
        self.browse_btn.pack(side=tk.LEFT, padx=5)

        # Map metadata inputs (Map Name & Vehicle Info)
        meta_frame = tk.Frame(self.control_card)
        meta_frame.pack(fill=tk.X, pady=(5, 5))

        # Map Name
        map_name_label = tk.Label(meta_frame, text=self.translator.get('label.map_name', 'Map Name:'))
        map_name_label.grid(row=0, column=0, sticky="w", padx=(0, 5), pady=2)
        self.map_name_var = tk.StringVar()
        map_name_entry = tk.Entry(meta_frame, textvariable=self.map_name_var)
        map_name_entry.grid(row=0, column=1, sticky="we", padx=(0, 5), pady=2)

        # Vehicle info (Name - ID)
        vehicle_label = tk.Label(meta_frame, text=self.translator.get('label.vehicle_info', 'Vehicle (Name - ID):'))
        vehicle_label.grid(row=1, column=0, sticky="w", padx=(0, 5), pady=2)
        self.vehicle_info_var = tk.StringVar()
        vehicle_entry = tk.Entry(meta_frame, textvariable=self.vehicle_info_var)
        vehicle_entry.grid(row=1, column=1, sticky="we", padx=(0, 5), pady=2)

        # Auto-fill vehicle_id từ MAC local và đặt read-only nếu có
        self.local_vehicle_id = self._get_local_mac_no_colon()
        if self.local_vehicle_id:
            self.vehicle_info_var.set(self.local_vehicle_id)
            vehicle_entry.config(state="readonly")
        else:
            self.vehicle_info_var.set(self.translator.get('label.vehicle_mac_not_found', 'MAC not found'))

        meta_frame.columnconfigure(1, weight=1)

        # Control Buttons
        btn_row = tk.Frame(self.control_card)
        btn_row.pack(fill=tk.X, pady=5)
        self.btn_start = tk.Button(btn_row, text=self.translator.get('button.start_mapping', '🚀 START MAPPING'), width=20, command=self.start_mapping)
        self.btn_start.pack(side=tk.LEFT, padx=5)
        self.btn_stop = tk.Button(btn_row, text=self.translator.get('button.stop_process', '■ STOP'), width=10, command=self.stop_process)
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(btn_row, text=self.translator.get('label.status_ready', 'Status: Ready'), font=("Arial", 9, "bold"))
        self.status_label.pack(side=tk.RIGHT, padx=10)

    def setup_preview_rviz_section(self):
        self.preview_card = tk.LabelFrame(
            self.workspace,
            text=self.translator.get('label.visualization_preview', 'Visualization Preview'),
            padx=15,
            pady=10,
        )
        self.preview_card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # 1. RViz checkbox section - tách biệt với spacing
        rviz_frame = tk.Frame(self.preview_card)
        rviz_frame.pack(fill=tk.X, pady=(0, 15))

        self.rviz_var = tk.BooleanVar(value=False)
        self.rviz_check = tk.Checkbutton(
            rviz_frame,
            text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'),
            variable=self.rviz_var,
            command=self.update_preview_mode,
        )
        self.rviz_check.pack(side=tk.LEFT)
        
        # Separator để tách biệt
        separator1 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator1.pack(fill=tk.X, pady=(0, 10))
        
        # 2. Progress section (Server Upload) - đặt trước log
        progress_frame = tk.Frame(self.preview_card)
        progress_frame.pack(fill=tk.X, pady=(0, 10))
        
        progress_label = tk.Label(progress_frame, text=self.translator.get('label.server_synchronization', 'Server Synchronization'), font=("Arial", 9, "bold"))
        progress_label.pack(anchor=tk.W, pady=(0, 5))
        
        progress_controls = tk.Frame(progress_frame)
        progress_controls.pack(fill=tk.X)
        
        self.btn_upload = tk.Button(progress_controls, text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'), 
                                    state=tk.NORMAL, command=self.start_upload)
        self.btn_upload.pack(side=tk.LEFT, padx=5)
        
        self.progress = ttk.Progressbar(progress_controls, orient=tk.HORIZONTAL, mode='determinate')
        self.progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.upload_stat = tk.Label(progress_controls, text=self.translator.get('label.waiting', 'Waiting...'), font=("Arial", 8))
        self.upload_stat.pack(side=tk.RIGHT)

        # 2.2. Design map comparison section
        self.comparison_frame = tk.LabelFrame(
            self.preview_card,
            text=self.translator.get('label.design_map_comparison', 'Design Map Comparison'),
            padx=10,
            pady=8,
        )
        self.comparison_frame.pack(fill=tk.X, pady=(0, 10))

        # Loại file thiết kế: PCD / OBJ
        design_type_frame = tk.Frame(self.comparison_frame)
        design_type_frame.pack(fill=tk.X, pady=(0, 5))

        self.design_file_type_label = tk.Label(
            design_type_frame,
            text=self.translator.get('label.design_file_type', 'Design file type:'),
            font=("Arial", 9, "bold"),
        )
        self.design_file_type_label.pack(side=tk.LEFT)

        self.design_type_var = tk.StringVar(value=self.design_file_type)
        design_pcd_radio = tk.Radiobutton(
            design_type_frame,
            text="PCD",
            variable=self.design_type_var,
            value="pcd",
            command=self.update_design_type,
        )
        design_pcd_radio.pack(side=tk.LEFT, padx=5)

        design_obj_radio = tk.Radiobutton(
            design_type_frame,
            text="OBJ",
            variable=self.design_type_var,
            value="obj",
            command=self.update_design_type,
        )
        design_obj_radio.pack(side=tk.LEFT, padx=5)

        # Chọn file thiết kế
        design_file_frame = tk.Frame(self.comparison_frame)
        design_file_frame.pack(fill=tk.X, pady=(0, 5))

        self.design_file_label = tk.Label(
            design_file_frame,
            text=self.translator.get('label.design_file', 'Design file:'),
            font=("Arial", 9),
        )
        self.design_file_label.pack(side=tk.LEFT)

        self.design_path_var = tk.StringVar()
        design_entry = tk.Entry(design_file_frame, textvariable=self.design_path_var)
        design_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.design_browse_btn = tk.Button(
            design_file_frame,
            text=self.translator.get('button.browse', 'Browse'),
            command=self.browse_design_file,
        )
        self.design_browse_btn.pack(side=tk.LEFT, padx=5)

        # Nhãn hiển thị kết quả so sánh
        self.comparison_status_label = tk.Label(
            self.comparison_frame,
            text=self.translator.get('label.design_comparison_not_configured', 'Design comparison: not configured'),
            font=("Arial", 11, "bold"),
            fg="gray",
        )
        self.comparison_status_label.pack(fill=tk.X, pady=(2, 0))

        # Separator để tách biệt progress và log
        separator2 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator2.pack(fill=tk.X, pady=(0, 10))
        
        # 2.5. QR Code section - đặt giữa progress và log
        qr_frame = tk.Frame(self.preview_card)
        qr_frame.pack(fill=tk.X, pady=(0, 10))
        
        qr_label = tk.Label(qr_frame, text=self.translator.get('label.qr_codes_detected', 'QR Codes Detected'), font=("Arial", 9, "bold"))
        qr_label.pack(anchor=tk.W, pady=(0, 5))
        
        qr_controls = tk.Frame(qr_frame)
        qr_controls.pack(fill=tk.X)
        
        # QR codes listbox với scrollbar
        qr_list_frame = tk.Frame(qr_controls)
        qr_list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar_qr = tk.Scrollbar(qr_list_frame)
        scrollbar_qr.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.qr_listbox = tk.Listbox(qr_list_frame, height=4, yscrollcommand=scrollbar_qr.set, 
                                     font=("Consolas", 9))
        self.qr_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_qr.config(command=self.qr_listbox.yview)
        
        # Button to clear QR codes
        btn_clear_qr = tk.Button(qr_controls, text=self.translator.get('button.clear_qr_codes', 'Clear QR Codes'), 
                                 font=("Arial", 7), command=self.clear_qr_codes)
        btn_clear_qr.pack(side=tk.RIGHT, padx=5)
        
        # Separator để tách biệt QR codes và log
        separator3 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator3.pack(fill=tk.X, pady=(0, 10))
        
        # 3. Log section - đặt cuối cùng
        log_container = tk.Frame(self.preview_card)
        log_container.pack(fill=tk.BOTH, expand=True)
        
        header = tk.Frame(log_container)
        header.pack(fill=tk.X, pady=(0, 2))
        self.system_log_label = tk.Label(header, text=self.translator.get('label.system_log', 'SYSTEM LOG'), font=("Arial", 8, "bold"))
        self.system_log_label.pack(side=tk.LEFT)
        self.clear_log_btn = tk.Button(header, text=self.translator.get('button.clear_log', 'Clear Log'), font=("Arial", 7), command=self.clear_log)
        self.clear_log_btn.pack(side=tk.RIGHT)

        self.log_panel = tk.Text(log_container, height=8, bg="white", fg="black", 
                                 font=("Consolas", 10), state=tk.DISABLED, bd=1, relief=tk.SUNKEN,
                                 wrap=tk.WORD, padx=5, pady=5)
        # Thêm scrollbar cho log
        log_scrollbar = tk.Scrollbar(log_container, orient=tk.VERTICAL, command=self.log_panel.yview)
        self.log_panel.config(yscrollcommand=log_scrollbar.set)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)



    # --- Logic ---
    def cleanup_ros2_resources(self):
        """Dọn dẹp shared memory và các process ROS2 cũ để tránh lỗi FastRTPS SHM"""
        try:
            self.add_log(self.translator.get('log.cleanup_ros2_resources', '🧹 Cleaning up old ROS2 resources...'))
            
            # 1. Kill các ROS2 processes cũ (fastlivo, ros2 launch, etc.)
            try:
                # Tìm và kill các process liên quan đến fast_livo
                kill_cmd = "pkill -f 'fast_livo|fastlivo' 2>/dev/null || true"
                subprocess.run(kill_cmd, shell=True, timeout=3)
                
                # Kill các ros2 launch processes
                kill_cmd2 = "pkill -f 'ros2 launch.*fast_livo' 2>/dev/null || true"
                subprocess.run(kill_cmd2, shell=True, timeout=3)
                
                time.sleep(0.5)  # Đợi processes dừng
            except Exception as e:
                self.add_log(self.translator.get('log.error_kill_processes', '⚠️ Error killing processes: {error}').replace('{error}', str(e)))
            
            # 2. Cleanup shared memory segments (FastRTPS SHM)
            try:
                # Tìm và xóa shared memory segments của FastRTPS
                # FastRTPS thường tạo segments trong /dev/shm với prefix "fastrtps_"
                shm_cleanup_cmd = "find /dev/shm -name 'fastrtps_*' -type f -delete 2>/dev/null || true"
                result = subprocess.run(shm_cleanup_cmd, shell=True, capture_output=True, text=True, timeout=5)
                
                # Đếm số file đã xóa (nếu có)
                if result.returncode == 0:
                    self.add_log(self.translator.get('log.cleanup_shared_memory', '✅ Cleaned up shared memory segments'))
            except Exception as e:
                self.add_log(self.translator.get('log.error_cleanup_memory', '⚠️ Error cleaning up shared memory: {error}').replace('{error}', str(e)))
            
            # 3. Cleanup semaphores liên quan (nếu có)
            try:
                # Xóa semaphores cũ của FastRTPS
                sem_cleanup_cmd = "ipcrm -a 2>/dev/null || true"
                subprocess.run(sem_cleanup_cmd, shell=True, timeout=3)
            except Exception as e:
                # ipcrm có thể không có trên một số hệ thống, bỏ qua lỗi
                pass
            
            self.add_log(self.translator.get('log.cleanup_complete', '✅ Finished cleaning up ROS2 resources'))
            time.sleep(0.5)  # Đợi một chút trước khi tiếp tục
            
        except Exception as e:
            self.add_log(self.translator.get('log.warning_cleanup', '⚠️ Warning during cleanup: {error}').replace('{error}', str(e)))
            # Không dừng lại, tiếp tục chạy mapping

    def update_preview_mode(self):
        self.use_rviz = self.rviz_var.get()
        if self.use_rviz:
            self.add_log(self.translator.get('log.config_rviz2_mode', 'CONFIG: Visualization mode set to RViz2.'))
        else:
            self.add_log(self.translator.get('log.config_internal_mode', 'CONFIG: Visualization mode set to Internal.'))

    def start_mapping(self):
        if self.is_mapping_running:
            self.add_log(self.translator.get('log.mapping_already_running', '⚠️ Mapping is already running'))
            return
        
        # Yêu cầu chọn bag trước, vì nút này sẽ vừa start mapping vừa play bag
        if not self.bag_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_bag_first', 'Please select bag folder before starting mapping & playback.')
            )
            return
        
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"Workspace setup not found at: {ws_setup}\n"
                f"{self.translator.get('message.build_workspace_first', 'Please build workspace first.')}"
            )
            return
        
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()
        
        if not self.config_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_config_first', 'Please select config file before starting mapping.')
            )
            return
        
        config_path_obj = Path(self.config_path)
        if not config_path_obj.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"{self.translator.get('message.config_not_exists', 'Config file does not exist')}: {self.config_path}\n"
                f"{self.translator.get('message.select_config_again', 'Please select config file again.')}"
            )
            return
        
        # Pre-flight checks để ngăn chặn crash
        self.add_log(self.translator.get('log.preflight_checks', '🔍 Running pre-flight checks to prevent crashes...'))
        
        # 1. Validate config file (kiểm tra YAML hợp lệ)
        if not self._validate_config_file(config_path_obj):
            return
        
        # 2. Validate config parameters (kiểm tra các tham số quan trọng)
        if not self._validate_config_parameters(config_path_obj):
            return
        
        # 3. Check system resources
        if not self._check_system_resources():
            return
        
        # 4. Validate bag file (kiểm tra topics trong bag)
        if not self._validate_bag_file():
            return
        
        self.add_log(self.translator.get('log.preflight_checks_passed', '✅ All pre-flight checks passed!'))
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            rviz_arg = "True" if self.use_rviz else "False"
            launch_file = "mapping_mid360_equirectangular.launch.py"
            config_name = config_path_obj.name
            
            if use_drive_ws:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={self.config_path}"
                )
                self.add_log(self.translator.get('log.source_drive_ws', '✅ Will source: ROS2 base -> drive_ws -> ws'))
            else:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={self.config_path}"
                )
                self.add_log(self.translator.get('log.source_ws_only', '⚠️ Will source: ROS2 base -> ws (no drive_ws)'))
            
            # Cleanup ROS2 resources trước khi start để tránh lỗi FastRTPS SHM
            self.cleanup_ros2_resources()
            
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            # Disable FastRTPS Shared Memory Transport để tránh lỗi port conflict
            # Sử dụng UDP transport thay vì SHM (an toàn hơn, tránh conflict)
            env['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'
            
            self.add_log("=" * 60)
            self.add_log(self.translator.get('log.starting_mapping_node', '🚀 Starting Mapping Node'))
            self.add_log(f"{self.translator.get('log.config_file', '📋 Config file')}: {config_name}")
            self.add_log(f"{self.translator.get('log.config_path', '📁 Config path')}: {self.config_path}")
            self.add_log(f"{self.translator.get('log.launch_file', '🎯 Launch file')}: {launch_file}")
            rviz_text = self.translator.get('log.rviz_enabled', 'Yes') if self.use_rviz else self.translator.get('log.rviz_disabled', 'No')
            self.add_log(f"{self.translator.get('log.rviz2', '👁️ RViz2')}: {rviz_text}")
            self.add_log("=" * 60)
            
            self.add_log(self.translator.get('log.starting_mapping_node_process', '📡 Starting mapping node...'))
            self.mapping_process = subprocess.Popen(
                mapping_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            # Graceful startup với health check
            if not self._wait_for_process_health(max_wait_time=10):
                # Process đã crash hoặc không khởi động được
                if self.mapping_process:
                    if self.mapping_process.poll() is not None:
                        error_output = ""
                        try:
                            error_output = self.mapping_process.stdout.read() if self.mapping_process.stdout else self.translator.get('log.no_output', 'No output')
                        except:
                            pass
                        self.add_log(f"❌ {self.translator.get('log.mapping_process_exited', 'Mapping process exited immediately with code')}: {self.mapping_process.returncode}")
                        if error_output:
                            self.add_log(f"{self.translator.get('log.output', 'Output')}: {error_output[:500]}")
                        messagebox.showerror(
                            self.translator.get('dialog.error', 'Error'),
                            self.translator.get('message.mapping_exited_immediately', 'Mapping process exited immediately. Check log for details.')
                        )
                    self.mapping_process = None
                return
            
            self.is_mapping_running = True
            self.add_log(self.translator.get('log.mapping_node_started', '✅ Mapping node started successfully'))
            
            self.btn_start.config(state=tk.DISABLED)
            self.status_label.config(
                text=self.translator.get('label.status_mapping_running', 'Status: 📡 Mapping node running'),
                foreground="orange"
            )
            
            threading.Thread(target=self.monitor_mapping_process, daemon=True).start()
            
            self.add_log("=" * 60)
            self.add_log(self.translator.get('log.mapping_node_ready', '✅ Mapping node is ready!'))
            self.add_log("=" * 60)
            
            # Sau khi mapping node sẵn sàng thì tự động play bag
            self.add_log(self.translator.get('log.auto_start_bag', '▶️ Auto-starting bag playback...'))
            self.start_bag_playback()
            
        except Exception as e:
            error_msg = f"{self.translator.get('message.cannot_start_mapping', 'Cannot start mapping')}: {e}"
            self.add_log(f"❌ {error_msg}")
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            self.cleanup_processes()

    def start_bag_playback(self):
        """Bắt đầu play bag sau khi mapping node đã chạy"""
        if self.is_bag_playing:
            self.add_log(self.translator.get('log.bag_already_playing', '⚠️ Bag is already playing'))
            return
        
        if not self.is_mapping_running:
            self.add_log(self.translator.get('log.mapping_not_running', '⚠️ Mapping node is not running. Cannot start bag playback.'))
            return
        
        if not self.bag_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_bag_first', 'Please select bag folder before starting mapping & playback.')
            )
            return
        
        bag_path_obj = Path(self.bag_path)
        if not bag_path_obj.exists():
            error_msg = self.translator.get('message.bag_folder_not_exists', 'Bag folder does not exist: {path}').replace('{path}', self.bag_path)
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            return
        
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"Workspace setup not found at: {ws_setup}\n"
                f"{self.translator.get('message.build_workspace_first', 'Please build workspace first.')}"
            )
            return
        
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Dùng self.bag_rate (mặc định 0.5x)
            try:
                bag_rate_value = float(self.bag_rate)
            except ValueError:
                bag_rate_value = 0.5
                self.add_log(self.translator.get('log.invalid_bag_rate', '⚠️ Invalid bag rate, fallback to 0.5x'))
            
            bag_play_cmd = f"ros2 bag play {self.bag_path} --rate {bag_rate_value}"
            
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.add_log("=" * 60)
            self.add_log(self.translator.get('log.starting_bag_playback', '▶️ Starting Bag Playback'))
            self.add_log(self.translator.get('log.bag_name', '📁 Bag: {name}').replace('{name}', bag_path_obj.name))
            self.add_log(self.translator.get('log.bag_path', '📁 Bag path: {path}').replace('{path}', self.bag_path))
            self.add_log(self.translator.get('log.bag_rate', '⚡ Rate: {rate}x').replace('{rate}', str(bag_rate_value)))
            self.add_log("=" * 60)
            
            if use_drive_ws:
                bag_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"{bag_play_cmd}"
                )
            else:
                bag_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"{bag_play_cmd}"
                )
            
            self.add_log(self.translator.get('log.launching_bag_play', '▶️ Launching ros2 bag play...'))
            self.bag_process = subprocess.Popen(
                bag_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_bag_playing = True
            self.add_log(self.translator.get('log.bag_playback_started', '✅ Bag playback started'))
            
            threading.Thread(target=self.monitor_bag_process, daemon=True).start()
            
            # Start QR code scanning subscriber if enabled
            if self.qr_scan_enabled and self.qr_scanner:
                self.start_qr_scanning_subscriber()
            
        except Exception as e:
            error_msg = f"Cannot start bag playback: {e}"
            self.add_log(f"❌ {error_msg}")
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)

    def check_pcd_files(self):
        """Kiểm tra xem có file PCD trong thư mục Log/PCD không"""
        pcd_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "PCD"
        if not pcd_dir.exists():
            return False
        
        # Tìm các file PCD (bỏ qua file merged nếu có)
        pcd_files = [f for f in pcd_dir.glob("*.pcd") if f.name not in ["merged_all.pcd", "merge_all_hba.pcd"]]
        return len(pcd_files) > 0

    def _get_local_mac_no_colon(self) -> str:
        """Lấy địa chỉ MAC (không dấu :) từ máy cục bộ."""
        try:
            mac_int = uuid.getnode()
            if mac_int == 0:
                return ""
            return f"{mac_int:012x}"
        except Exception as e:
            self.add_log(f"⚠️ Cannot read local MAC: {e}")
            return ""
    
    def start_upload(self):
        """Chạy full pipeline: Merge -> HBA -> SC -> Floorplan -> Upload"""
        pcd_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "PCD"
        
        if not self.check_pcd_files():
            messagebox.showerror(
                self.translator.get('dialog.no_pcd_files_title', 'No PCD files found'),
                self.translator.get('dialog.no_pcd_files_message', 
                    'No PCD files found in directory:\n{path}\n\nPlease run mapping first to create PCD files.')
                    .replace('{path}', str(pcd_dir))
            )
            return

        map_name = (self.map_name_var.get() or "").strip()
        vehicle_id = self._get_local_mac_no_colon()
        if vehicle_id:
            # Cập nhật UI hiển thị để người dùng biết ID được tự lấy
            self.vehicle_info_var.set(vehicle_id)

        if not map_name:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('dialog.map_name_required', 'Please enter Map Name before uploading.')
            )
            return

        if not vehicle_id:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('dialog.vehicle_id_required', 'Cannot detect local MAC for Vehicle ID. Please check network interfaces.')
            )
            return

        # Kiểm tra bắt buộc phải chọn file thiết kế để so sánh
        if not self.design_file_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.design_file_required', 
                    'Please select a design map file (PCD or OBJ) before uploading.\n\nDesign map comparison is required to ensure map quality.')
            )
            return
        
        # Validate design file exists
        design_path = Path(self.design_file_path)
        if not design_path.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.design_file_not_exists', 
                    'Design map file does not exist:\n{path}\n\nPlease select a valid design map file.')
                    .replace('{path}', str(design_path))
            )
            return

        # Lưu lại để truyền vào thread upload
        self.current_map_name = map_name
        self.current_vehicle_id = vehicle_id
        self.current_vehicle_input = vehicle_id

        self.btn_upload.config(state=tk.DISABLED)
        self.add_log(self.translator.get('log.pipeline_start', '🚀 Starting map processing and upload pipeline...'))
        threading.Thread(
            target=self._run_upload_pipeline_thread,
            args=(map_name, vehicle_id),
            daemon=True
        ).start()
    
    def _run_upload_pipeline_thread(self, map_name: str, vehicle_id: str):
        """Thread thực hiện full pipeline và upload"""
        start_time = time.time()
        
        try:
            # 1. Merge PCD
            self.add_log(self.translator.get('log.step_merge_pcd', 'Step 1/6: Merging PCD files...'))
            if not self.merge_pcd_files(silent=True):
                self.add_log(self.translator.get('log.pipeline_stopped_merge', '❌ Pipeline stopped at Merge PCD step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.set_progress(20))
            
            # 2. HBA Optimize
            self.add_log(self.translator.get('log.step_hba', 'Step 2/6: Running HBA Optimize...'))
            if not self._run_hba_core():
                self.add_log(self.translator.get('log.pipeline_stopped_hba', '❌ Pipeline stopped at HBA step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.set_progress(40))
            
            # 3. SC Tile
            self.add_log(self.translator.get('log.step_sc', 'Step 3/6: Running ScanContext Tiling...'))
            if not self._run_sc_core():
                self.add_log(self.translator.get('log.pipeline_stopped_sc', '❌ Pipeline stopped at ScanContext step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.set_progress(60))
            
            # 4. Generate Floorplan
            self.add_log(self.translator.get('log.step_floorplan', 'Step 4/6: Generating floorplan...'))
            if not self._generate_floorplan():
                self.add_log(self.translator.get('log.pipeline_stopped_floorplan', '❌ Pipeline stopped at Generate Floorplan step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.set_progress(80))
            
            # 4.5. So sánh với bản thiết kế (PCD/OBJ) nếu người dùng đã chọn
            if self.design_file_path:
                self.add_log(self.translator.get('log.step_comparing_design_map', 
                    'Step 5/7: Comparing generated map with design map...'))
                generated_pcd_path = (
                    self.workspace_path
                    / "src"
                    / "FAST-LIVO2"
                    / "Log"
                    / "merged_map"
                    / "merged_all.pcd"
                )

                success, similarity = self.run_design_map_comparison(generated_pcd_path)
                if not success:
                    self.add_log(self.translator.get('log.design_map_comparison_failed', 
                        '❌ Design map comparison failed. Stopping pipeline before upload.'))
                    self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                    return

                # Cập nhật progress nhẹ sau khi so sánh
                self.root.after(0, lambda: self.set_progress(82))

                # Nếu % khớp thấp hơn ngưỡng, hỏi người dùng có tiếp tục upload không
                try:
                    threshold = float(self.comparison_threshold)
                except Exception:
                    threshold = 90.0
                if similarity < threshold:
                    warn_msg = (
                        self.translator.get('log.low_similarity_warning', 
                            '⚠️ Low similarity with design map: {similarity}% (threshold: {threshold}%)\n\nPossible drift detected. Do you want to continue upload anyway?')
                            .replace('{similarity}', f'{similarity:.1f}')
                            .replace('{threshold}', f'{threshold:.1f}')
                    )
                    self.add_log(
                        f"⚠️ Low similarity with design map: {similarity:.1f}% "
                        f"(threshold {threshold:.1f}%). Asking user..."
                    )
                    if not messagebox.askyesno(
                        self.translator.get('dialog.low_similarity_warning_title', 'Low similarity warning'), 
                        warn_msg
                    ):
                        self.add_log("⛔ User cancelled upload due to low similarity with design map.")
                        self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                        return
                    else:
                        self.add_log(self.translator.get('log.user_continue_upload_low_similarity', 
                            '➡ User chose to continue upload despite low similarity.'))
            
            # 5. Save QR codes to JSON
            self.add_log(self.translator.get('log.step_save_qr', 'Step 5/7: Saving QR codes...'))
            qr_json_path = self._save_qr_codes_to_json()
            if qr_json_path:
                self.add_log(self.translator.get('log.qr_codes_saved', '✅ QR codes saved to: {path}').replace('{path}', str(qr_json_path)))
            else:
                self.add_log(self.translator.get('log.no_qr_codes_to_save', '⚠️ No QR codes to save'))
            
            self.root.after(0, lambda: self.set_progress(75))
            
            # 6. Zip files
            self.add_log(self.translator.get('log.step_zip', 'Step 6/7: Compressing files (Zip)...'))
            zip_path = self._zip_map_folders()
            if not zip_path:
                self.add_log(self.translator.get('log.pipeline_stopped_zip', '❌ Pipeline stopped at Zip step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.set_progress(90))
            
            # 7. Upload to backend
            self.add_log(self.translator.get('log.step_upload', 'Step 7/7: Uploading to backend...'))
            upload_success, upload_info = self._upload_map_to_backend(zip_path, map_name, vehicle_id)
            if upload_success:
                if upload_info:
                    upload_msg = self.translator.get('log.upload_backend_success', '✅ Backend upload successful (upload_id: {upload_id})').replace('{upload_id}', str(upload_info))
                else:
                    upload_msg = "✅ Backend upload successful"
                self.add_log(upload_msg)
                # Enable lại nút upload khi thành công
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
            else:
                fail_reason = upload_info or "Unknown error"
                self.add_log(self.translator.get('log.upload_backend_failed', '⚠️ Backend upload failed: {reason}').replace('{reason}', str(fail_reason)))
                # Enable lại nút upload khi thất bại để có thể thử lại
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
            
            self.root.after(0, lambda: self.set_progress(100))
            self.root.after(0, lambda: self.upload_stat.config(text="100% - Done"))
            
            duration = time.time() - start_time
            self.add_log(self.translator.get('log.pipeline_complete', '🎉 Pipeline completed in {duration:.1f} seconds!').replace('{duration:.1f}', f'{duration:.1f}'))
            
            # Show success message
            upload_status = (
                f"Upload backend: OK (upload_id: {upload_info})" if upload_success and upload_info
                else "Upload backend: OK" if upload_success
                else f"Upload backend: LỖI ({upload_info})"
            )
            self.root.after(0, lambda: messagebox.showinfo(
                self.translator.get('dialog.pipeline_complete_title', 'Success'),
                self.translator.get('dialog.pipeline_complete_message', 
                    'Pipeline completed successfully!\n\nZip file: {filename}\n{upload_status}')
                    .replace('{filename}', zip_path.name)
                    .replace('{upload_status}', upload_status)
            ))
            
        except Exception as e:
            self.add_log(self.translator.get('log.error_in_pipeline', '❌ Error in pipeline: {error}').replace('{error}', str(e)))
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()}")
            self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
    
    def simulate_upload_progress(self, val):
        if val <= 100:
            self.set_progress(val)
            upload_text = self.translator.get('label.uploading', 'Uploading: {val}%').replace('{val}', str(val))
            self.upload_stat.config(text=upload_text)
            self.root.after(30, lambda: self.simulate_upload_progress(val + 2))
        else:
            self.add_log(self.translator.get('log.server_upload_success', 'SERVER: Upload successful. Data stored in cloud.'))
            self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            self.btn_upload.config(state=tk.NORMAL)
            messagebox.showinfo(
                self.translator.get('dialog.success', 'Success'),
                self.translator.get('message.all_map_data_uploaded', 'All map data has been uploaded to the server!')
            )

    def browse_file(self):
        initial_dir = "/media/an/01DC80D9DB838380/"
        bag_path = filedialog.askdirectory(
            title=self.translator.get('dialog.choose_bag_folder', 'Choose Bag Folder'),
            initialdir=initial_dir
        )
        
        if bag_path:
            bag_path_obj = Path(bag_path)
            if bag_path_obj.exists():
                self.bag_path_var.set(str(bag_path_obj))
                self.bag_path = str(bag_path_obj)
                self.add_log_success('message.bag_selected', name=bag_path_obj.name)
            else:
                error_msg = self.translator.get('message.bag_folder_not_exists', 'Bag folder does not exist: {path}').replace('{path}', bag_path)
                messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)

    def update_design_type(self):
        """Cập nhật loại file thiết kế (PCD / OBJ) khi người dùng chọn."""
        self.design_file_type = self.design_type_var.get()
        if self.design_file_type == "obj":
            msg = self.translator.get('label.design_type_obj', 
                'Design type: OBJ | voxel=0.15 | sampling=uniform | Match PCD density + Save converted PCD')
        else:
            msg = self.translator.get('label.design_type_pcd', 'Design type: PCD')
        if hasattr(self, "comparison_status_label"):
            self.comparison_status_label.config(text=msg, fg="blue", font=("Arial", 11, "bold"))
        self.add_log(self.translator.get('log.design_type_set', 
            'CONFIG: Design map type set to {type}').replace('{type}', self.design_file_type.upper()))

    def browse_design_file(self):
        """Chọn file bản thiết kế (PCD hoặc OBJ) để so sánh."""
        # Thư mục gợi ý: dùng đường dẫn hiện tại hoặc Log
        initial_dir = self.design_path_var.get()
        if not initial_dir:
            initial_dir = str(self.workspace_path / "src" / "FAST-LIVO2" / "Log")

        if self.design_file_type == "obj":
            filetypes = [("OBJ files", "*.obj"), ("All files", "*.*")]
            title = self.translator.get('dialog.choose_design_obj_file', 'Choose Design OBJ File')
        else:
            filetypes = [("PCD files", "*.pcd"), ("All files", "*.*")]
            title = self.translator.get('dialog.choose_design_pcd_file', 'Choose Design PCD File')

        filename = filedialog.askopenfilename(
            title=title,
            initialdir=initial_dir,
            filetypes=filetypes,
        )

        if filename:
            self.design_path_var.set(filename)
            self.design_file_path = filename
            file_ext = "OBJ" if self.design_file_type == "obj" else "PCD"
            self.add_log(self.translator.get('log.design_file_selected', 
                '✅ Design {file_ext} selected: {filename}').replace('{file_ext}', file_ext).replace('{filename}', Path(filename).name))
            if hasattr(self, "comparison_status_label"):
                status_text = self.translator.get('label.design_file_selected', 
                    'Design file: {filename}').replace('{filename}', Path(filename).name)
                self.comparison_status_label.config(
                    text=status_text,
                    fg="green",
                    font=("Arial", 11, "bold"),
                )

    def launch_rviz_cmd(self):
        self.add_log(self.translator.get('log.system_launching_rviz2', 'SYSTEM: Launching RViz2...'))

    def run_design_map_comparison(self, generated_pcd_path: Path):
        """
        So sánh bản đồ tạo ra (merged_all.pcd) với bản thiết kế (PCD/OBJ).
        
        Returns:
            (success: bool, similarity: float)
        """
        if not COMPARISON_UTILS_AVAILABLE:
            self.add_log("⚠️ Design comparison utils not available. Skipping comparison step.")
            return True, 100.0

        try:
            design_path_str = self.design_file_path
            if not design_path_str:
                self.add_log("⚠️ No design file selected. Skipping comparison.")
                return True, 100.0

            design_path = Path(design_path_str)
            if not design_path.exists():
                msg = f"Design file not found: {design_path}"
                self.add_log(f"❌ {msg}")
                messagebox.showerror("Design map comparison", msg)
                return False, 0.0

            if not generated_pcd_path.exists():
                msg = f"Generated map PCD not found: {generated_pcd_path}"
                self.add_log(f"❌ {msg}")
                messagebox.showerror(
                    self.translator.get('dialog.design_map_comparison', 'Design map comparison'), 
                    msg
                )
                return False, 0.0

            self.add_log(self.translator.get('log.running_design_map_comparison', 
                '🔍 Running design map comparison...'))
            self.add_log(f"   Design file: {design_path.name} ({self.design_file_type.upper()})")
            self.add_log(f"   Generated map: {generated_pcd_path.name}")

            # Load test PCD (bản đồ tạo ra)
            pcd_test = load_pcd(generated_pcd_path)
            test_points = len(pcd_test.points)

            # Load reference từ PCD hoặc OBJ
            if self.design_file_type == "obj":
                # Thiết lập sampling cho OBJ: uniform, match density, lưu PCD convert
                match_density = test_points if test_points > 0 else None
                self.add_log(
                    f"   OBJ settings: voxel=0.15, sampling=uniform, "
                    f"match_density={match_density}, save_converted_pcd=True"
                )

                # Tạo thư mục lưu PCD convert từ OBJ
                self.obj_converted_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                converted_pcd_path = self.obj_converted_dir / f"{design_path.stem}_converted_{timestamp}.pcd"

                # Convert OBJ -> point cloud
                pcd_ref = load_obj_as_pointcloud(
                    design_path,
                    sampling_method="uniform",
                    num_points=None,
                    match_density=match_density,
                )
                self.add_log(f"   Converted OBJ to PCD with {len(pcd_ref.points):,} points")
            else:
                converted_pcd_path = None
                pcd_ref = load_pcd(design_path)

            ref_points = len(pcd_ref.points)
            self.add_log(f"   Reference points: {ref_points:,}")
            self.add_log(f"   Test points: {test_points:,}")

            # Preprocess với voxel size = 0.15, remove outliers
            voxel_size = 0.15
            self.add_log(f"   Preprocessing with voxel size = {voxel_size}")

            pcd_ref = preprocess_pcd(
                pcd_ref,
                voxel_size=voxel_size,
                remove_outliers=True,
            )
            pcd_test = preprocess_pcd(
                pcd_test,
                voxel_size=voxel_size,
                remove_outliers=True,
            )

            self.add_log(f"   After preprocessing - Reference: {len(pcd_ref.points):,} points")
            self.add_log(f"   After preprocessing - Test: {len(pcd_test.points):,} points")

            # ICP threshold tự động theo voxel size
            icp_threshold = voxel_size * DEFAULT_ICP_THRESHOLD_MULTIPLIER
            self.add_log(f"   ICP threshold: {icp_threshold:.3f} m")

            # Nếu là OBJ, lưu PCD convert vào thư mục riêng
            save_pcd_path = converted_pcd_path if converted_pcd_path is not None else None
            if save_pcd_path is not None:
                self.add_log(f"   Will save converted design PCD to: {save_pcd_path}")

            metrics = compute_similarity_metrics(
                pcd_ref,
                pcd_test,
                perform_icp=True,
                icp_threshold=icp_threshold,
                icp_max_iterations=DEFAULT_ICP_MAX_ITERATIONS,
                save_obj_pcd=save_pcd_path,
            )

            if save_pcd_path is not None:
                self.add_log(f"✅ Converted design PCD saved to: {save_pcd_path}")

            similarity = calculate_similarity_percentage(metrics)
            drift_detected, drift_reasons = detect_drift(metrics, similarity)

            # Lưu kết quả vào state
            self.comparison_result = {
                "design_file": str(design_path),
                "generated_file": str(generated_pcd_path),
                "metrics": metrics,
                "similarity": similarity,
                "drift_detected": drift_detected,
                "drift_reasons": drift_reasons,
            }

            # Log tóm tắt
            self.add_log("=" * 60)
            self.add_log(self.translator.get('log.design_map_comparison_results', 
                'Design Map Comparison Results'))
            self.add_log(self.translator.get('label.similarity_percent', 
                '   Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}'))
            self.add_log(f"   ICP fitness: {metrics.get('icp_fitness', 0.0):.4f}")
            self.add_log(f"   Hausdorff distance: {metrics.get('hausdorff_distance', 0.0):.4f} m")
            self.add_log(f"   Chamfer distance: {metrics.get('chamfer_distance', 0.0):.4f} m")
            self.add_log(f"   BBox overlap: {metrics.get('bbox_overlap', 0.0):.4f}")
            self.add_log(f"   Centroid distance: {metrics.get('centroid_distance', 0.0):.4f} m")
            self.add_log(f"   Density ratio: {metrics.get('density_ratio', 0.0):.4f}")

            status_text = self.translator.get('label.similarity_percent', 
                'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}')
            if drift_detected:
                self.add_log("   Drift detected:")
                for reason in drift_reasons:
                    self.add_log(f"     - {reason}")
                status_text += " (" + self.translator.get('label.possible_drift', 'Possible drift') + ")"
            
            # Xác định màu sắc dựa trên % similarity
            if similarity >= 90:
                status_color = "green"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.no_significant_drift', 'No significant drift') + ")"
            elif similarity >= 50:
                status_color = "orange"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.moderate_match', 'Moderate match') + ")"
            else:
                status_color = "red"
                if not drift_detected:
                    status_text = self.translator.get('label.similarity_percent', 
                        'Similarity: {similarity}%').replace('{similarity}', f'{similarity:.2f}') + \
                        " (" + self.translator.get('label.low_match', 'Low match') + ")"

            self.add_log("=" * 60)

            # Cập nhật label UI trong main thread
            if hasattr(self, "comparison_status_label"):
                self.root.after(
                    0,
                    lambda txt=status_text, col=status_color: self.comparison_status_label.config(
                        text=txt, fg=col, font=("Arial", 11, "bold")
                    ),
                )

            return True, similarity

        except Exception as e:
            self.add_log(f"❌ Error in design map comparison: {e}")
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()[:500]}")
            try:
                messagebox.showerror(
                    self.translator.get('dialog.design_map_comparison', 'Design map comparison'), 
                    f"Error during comparison:\n{e}"
                )
            except Exception:
                pass
            return False, 0.0

    def clear_log(self):
        self.log_panel.config(state=tk.NORMAL)
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state=tk.DISABLED)
    
    def clear_qr_codes(self):
        """Xóa danh sách QR codes đã quét"""
        with self.qr_codes_lock:
            self.detected_qr_codes = []
        self.qr_listbox.delete(0, tk.END)
        self.add_log(self.translator.get('log.qr_codes_cleared', '✅ QR codes list cleared'))
    
    def scan_qr_from_image(self, cv_image):
        """Quét QR code từ một ảnh cụ thể (equirectangular format)"""
        if not self.qr_scanner:
            self.add_log(self.translator.get('log.qr_scanner_not_available', '⚠️ QR scanner is not available. Please install pyzbar and geometry_utils.'))
            return []
        
        if cv2 is None:
            self.add_log('⚠️ cv2 not available. Cannot scan QR codes.')
            return []
        
        try:
            # Chuyển đổi BGR sang RGB nếu cần
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                # Kiểm tra xem có phải BGR không (OpenCV default)
                # Nếu là RGB thì giữ nguyên, nếu là BGR thì convert
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = cv_image
            
            # Quét QR code
            qr_codes = self.qr_scanner.scan_qr_codes(rgb_image)
            
            # Xử lý kết quả
            if qr_codes:
                self.on_qr_codes_received(qr_codes)
            
            return qr_codes
        except Exception as e:
            self.add_log(self.translator.get('log.error_scanning_qr', '❌ Error scanning QR from image: {error}').replace('{error}', str(e)))
            return []
    
    def on_odometry_received(self, position):
        """Callback khi nhận được odometry - cập nhật vị trí hiện tại"""
        with self.position_lock:
            self.current_position = position
    
    def on_qr_codes_received(self, qr_codes):
        """Callback để nhận QR codes (có thể từ theta tab hoặc từ scan_qr_from_image)"""
        if not qr_codes:
            return
        
        # Lấy vị trí hiện tại
        with self.position_lock:
            current_pos = self.current_position.copy() if self.current_position else None
        
        with self.qr_codes_lock:
            for qr in qr_codes:
                qr_data = qr.get("data", "")
                
                # Kiểm tra xem QR code đã tồn tại chưa
                existing_qr = None
                for existing in self.detected_qr_codes:
                    if existing.get("data") == qr_data:
                        existing_qr = existing
                        break
                
                if existing_qr:
                    # QR đã tồn tại - replace với vị trí mới
                    existing_qr["position"] = current_pos if current_pos else [0.0, 0.0, 0.0]
                    existing_qr["timestamp"] = datetime.now().isoformat()
                    pos_str = f"[{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]" if current_pos else "unknown"
                    self.add_log(self.translator.get('log.qr_code_updated', '🔄 QR Code updated: {code}').replace('{code}', qr_data) + f" at position {pos_str}")
                else:
                    # QR mới - thêm vào danh sách
                    qr["position"] = current_pos if current_pos else [0.0, 0.0, 0.0]
                    qr["timestamp"] = datetime.now().isoformat()
                    self.detected_qr_codes.append(qr)
                    # Cập nhật UI trong main thread
                    self.root.after(0, lambda q=qr_data: self._add_qr_to_listbox(q))
                    pos_str = f"[{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]" if current_pos else "unknown"
                    self.add_log(self.translator.get('log.qr_code_detected', '✅ QR Code detected: {code}').replace('{code}', qr_data) + f" at position {pos_str}")
        
        # Lưu vào file QR_detect.json ngay lập tức
        self.save_qr_detect_json()
    
    def save_qr_detect_json(self):
        """Lưu QR codes và tọa độ vào file QR_detect.json"""
        try:
            # Tạo thư mục Log nếu chưa có
            log_dir = self.qr_detect_json_path.parent
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # Đọc file hiện tại nếu có
            qr_dict = {}
            if self.qr_detect_json_path.exists():
                try:
                    with open(self.qr_detect_json_path, 'r', encoding='utf-8') as f:
                        qr_dict = json.load(f)
                except Exception as e:
                    self.add_log(self.translator.get('log.error_reading_qr_json', '⚠️ Error reading QR_detect.json: {error}').replace('{error}', str(e)))
                    qr_dict = {}
            
            # Cập nhật với QR codes hiện tại
            with self.qr_codes_lock:
                for qr in self.detected_qr_codes:
                    qr_data = qr.get("data", "")
                    position = qr.get("position", [0.0, 0.0, 0.0])
                    
                    # Format: key là nội dung QR, value là tọa độ [x, y, z]
                    qr_dict[qr_data] = position
            
            # Lưu vào file
            with open(self.qr_detect_json_path, 'w', encoding='utf-8') as f:
                json.dump(qr_dict, f, indent=2, ensure_ascii=False)
            
            self.add_log(self.translator.get('log.qr_detect_json_saved', '💾 QR_detect.json saved: {count} QR codes').replace('{count}', str(len(qr_dict))))
            
        except Exception as e:
            self.add_log(self.translator.get('log.error_saving_qr_detect_json', '❌ Error saving QR_detect.json: {error}').replace('{error}', str(e)))
            import traceback
            traceback.print_exc()
    
    def _add_qr_to_listbox(self, qr_data):
        """Thêm QR code vào listbox"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.qr_listbox.insert(tk.END, f"[{timestamp}] {qr_data}")
        self.qr_listbox.see(tk.END)

    def add_log(self, message):
        """Thêm log message vào queue để update mượt hơn"""
        time_str = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{time_str}] {message}\n"
        
        with self.log_update_lock:
            self.log_queue.append(log_entry)
    
    def _log_update_worker(self):
        """Worker thread để batch update log panel mượt hơn"""
        while self.log_update_thread_running:
            try:
                # Lấy batch log entries từ queue
                batch = []
                with self.log_update_lock:
                    if self.log_queue:
                        batch = self.log_queue[:self.log_batch_size]
                        self.log_queue = self.log_queue[self.log_batch_size:]
                
                if batch:
                    # Update UI trong main thread
                    self.root.after(0, lambda: self._update_log_panel(batch))
                
                # Sleep ngắn để không chiếm CPU
                time.sleep(0.05)  # 50ms = 20 updates/second
            except Exception as e:
                # Nếu có lỗi, log trực tiếp để không mất thông tin
                try:
                    self.root.after(0, lambda: self._update_log_panel_direct(f"Error in log worker: {e}\n"))
                except:
                    pass
                time.sleep(0.1)
    
    def _update_log_panel(self, batch):
        """Update log panel với batch entries"""
        try:
            self.log_panel.config(state=tk.NORMAL)
            
            # Insert tất cả entries trong batch
            for entry in batch:
                self.log_panel.insert(tk.END, entry)
            
            # Smooth scroll đến cuối
            self.log_panel.see(tk.END)
            
            # Giới hạn số dòng để tránh memory leak (giữ tối đa 1000 dòng)
            line_count = int(self.log_panel.index('end-1c').split('.')[0])
            if line_count > 1000:
                # Xóa 200 dòng đầu
                self.log_panel.delete('1.0', '200.0')
            
            self.log_panel.config(state=tk.DISABLED)
        except Exception as e:
            # Fallback nếu có lỗi
            print(f"Error updating log panel: {e}")
    
    def _update_log_panel_direct(self, message):
        """Update log panel trực tiếp (fallback)"""
        try:
            self.log_panel.config(state=tk.NORMAL)
            self.log_panel.insert(tk.END, message)
            self.log_panel.see(tk.END)
            self.log_panel.config(state=tk.DISABLED)
        except:
            pass
    
    def _progress_update_worker(self):
        """Worker thread để animate progress bar mượt hơn"""
        while self.log_update_thread_running:
            try:
                if self.progress_queue is not None:
                    target_value, current_value = self.progress_queue
                    
                    # Tính toán giá trị mới với animation
                    if abs(target_value - current_value) < self.progress_animation_speed:
                        # Đã đạt target
                        new_value = target_value
                        self.progress_queue = None
                    else:
                        # Di chuyển về phía target
                        if target_value > current_value:
                            new_value = min(current_value + self.progress_animation_speed, target_value)
                        else:
                            new_value = max(current_value - self.progress_animation_speed, target_value)
                        self.progress_queue = (target_value, new_value)
                    
                    # Update UI trong main thread
                    self.root.after(0, lambda v=new_value: self._update_progress_bar(v))
                
                # Sleep ngắn để animation mượt
                time.sleep(0.03)  # ~33 FPS cho animation
            except Exception as e:
                time.sleep(0.1)
    
    def _update_progress_bar(self, value):
        """Update progress bar với giá trị mới"""
        try:
            self.progress['value'] = value
            # Update status text
            if value >= 100:
                self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            elif value > 0:
                self.upload_stat.config(text=self.translator.get('label.uploading', 'Uploading: {val}%').replace('{val}', str(int(value))))
        except:
            pass
    
    def set_progress(self, value):
        """Set progress bar với smooth animation"""
        try:
            current_value = self.progress['value']
            # Clamp value
            value = max(0, min(100, value))
            # Set vào queue để animate
            self.progress_queue = (value, current_value)
        except:
            # Fallback: set trực tiếp
            try:
                self.progress['value'] = value
            except:
                pass
    
    def add_log_success(self, message_key, **kwargs):
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"✅ {msg}")
    
    def add_log_warning(self, message_key, **kwargs):
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"⚠️ {msg}")

    def set_default_config(self):
        default_config = self.workspace_path / "src" / "FAST-LIVO2" / "config" / "mid360_equirectangular_stable.yaml"
        if default_config.exists():
            self.config_path = str(default_config)
            self.add_log_success('message.default_config_selected', name=default_config.name)
        else:
            self.add_log_warning('message.default_config_not_found', name='mid360_equirectangular_stable.yaml')
    
    def _validate_config_file(self, config_path_obj):
        """Validate YAML config file trước khi sử dụng"""
        try:
            import yaml
        except ImportError:
            # Nếu không có yaml, chỉ kiểm tra file tồn tại
            self.add_log(self.translator.get('log.yaml_not_available', '⚠️ PyYAML not available, skipping config validation'))
            return True
        
        try:
            with open(config_path_obj, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
            
            if config_data is None:
                error_msg = self.translator.get('message.config_file_empty', 'Config file is empty or invalid YAML format')
                self.add_log(f"❌ {error_msg}")
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    f"{error_msg}: {config_path_obj}"
                )
                return False
            
            # Kiểm tra các tham số quan trọng
            required_keys = ['common', 'lidar', 'imu']
            missing_keys = [key for key in required_keys if key not in config_data]
            
            if missing_keys:
                error_msg = self.translator.get('message.config_missing_keys', 
                    'Config file is missing required sections: {keys}').replace('{keys}', ', '.join(missing_keys))
                self.add_log(f"⚠️ {error_msg}")
                # Không block, chỉ cảnh báo
                self.add_log(self.translator.get('log.config_validation_warning', 
                    '⚠️ Config validation warning, but continuing...'))
            
            self.add_log(self.translator.get('log.config_validation_success', '✅ Config file validation passed'))
            return True
            
        except yaml.YAMLError as e:
            error_msg = self.translator.get('message.config_yaml_error', 
                'Config file has invalid YAML syntax: {error}').replace('{error}', str(e))
            self.add_log(f"❌ {error_msg}")
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"{error_msg}\n\nFile: {config_path_obj}"
            )
            return False
        except Exception as e:
            error_msg = self.translator.get('message.config_validation_error', 
                'Error validating config file: {error}').replace('{error}', str(e))
            self.add_log(f"⚠️ {error_msg}")
            # Không block, chỉ cảnh báo
            return True
    
    def _validate_config_parameters(self, config_path_obj):
        """Validate các tham số quan trọng trong config để tránh crash"""
        try:
            import yaml
        except ImportError:
            return True
        
        try:
            with open(config_path_obj, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
            
            if not config_data:
                return True
            
            errors = []
            warnings = []
            
            # Kiểm tra common section
            if 'common' in config_data:
                common = config_data['common']
                # Kiểm tra các tham số có thể gây crash nếu sai
                if 'lidar_topic' in common:
                    topic = common['lidar_topic']
                    if not topic or not isinstance(topic, str):
                        errors.append(self.translator.get('log.config_invalid_lidar_topic', 'Invalid lidar_topic in config'))
                
                if 'imu_topic' in common:
                    topic = common['imu_topic']
                    if not topic or not isinstance(topic, str):
                        errors.append(self.translator.get('log.config_invalid_imu_topic', 'Invalid imu_topic in config'))
            
            # Kiểm tra lidar section
            if 'lidar' in config_data:
                lidar = config_data['lidar']
                # Kiểm tra các tham số số học
                numeric_params = ['scan_line', 'blind', 'fov_degree', 'min_ring', 'max_ring']
                for param in numeric_params:
                    if param in lidar:
                        try:
                            value = float(lidar[param])
                            if value < 0 and param in ['scan_line', 'min_ring', 'max_ring']:
                                warnings.append(self.translator.get('log.config_negative_value', 
                                    f'Warning: {param} has negative value: {value}').replace('{param}', param).replace('{value}', str(value)))
                        except (ValueError, TypeError):
                            errors.append(self.translator.get('log.config_invalid_numeric', 
                                f'Invalid numeric value for {param}').replace('{param}', param))
            
            # Kiểm tra imu section
            if 'imu' in config_data:
                imu = config_data['imu']
                # Kiểm tra các tham số quan trọng
                if 'acc_n' in imu:
                    try:
                        acc_n = float(imu['acc_n'])
                        if acc_n <= 0:
                            warnings.append(self.translator.get('log.config_invalid_acc_n', 
                                'Warning: acc_n should be positive'))
                    except (ValueError, TypeError):
                        pass
            
            if errors:
                error_msg = '\n'.join(errors)
                self.add_log(f"❌ {self.translator.get('log.config_validation_errors', 'Config validation errors')}:")
                for err in errors:
                    self.add_log(f"   - {err}")
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    f"{self.translator.get('message.config_has_errors', 'Config file has errors that may cause crashes')}:\n\n{error_msg}"
                )
                return False
            
            if warnings:
                self.add_log(f"⚠️ {self.translator.get('log.config_validation_warnings', 'Config validation warnings')}:")
                for warn in warnings:
                    self.add_log(f"   - {warn}")
            
            return True
            
        except Exception as e:
            self.add_log(f"⚠️ {self.translator.get('log.config_parameter_validation_error', 'Error validating config parameters: {error}').replace('{error}', str(e))}")
            # Không block, chỉ cảnh báo
            return True
    
    def _check_system_resources(self):
        """Kiểm tra system resources để tránh crash do thiếu tài nguyên"""
        try:
            import psutil
        except ImportError:
            # Nếu không có psutil, bỏ qua check
            self.add_log(self.translator.get('log.psutil_not_available', '⚠️ psutil not available, skipping resource check'))
            return True
        
        try:
            # Kiểm tra memory
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            available_gb = memory.available / (1024**3)
            
            if memory_percent > 90:
                error_msg = self.translator.get('message.low_memory', 
                    'System memory is very low ({percent}% used, {available:.1f}GB available). This may cause crashes.').replace('{percent}', str(memory_percent)).replace('{available:.1f}', f'{available_gb:.1f}')
                self.add_log(f"⚠️ {error_msg}")
                if not messagebox.askyesno(
                    self.translator.get('dialog.warning', 'Warning'),
                    f"{error_msg}\n\n{self.translator.get('message.continue_anyway', 'Continue anyway?')}"
                ):
                    return False
            elif memory_percent > 75:
                self.add_log(self.translator.get('log.memory_usage_high', 
                    f'⚠️ Memory usage is high: {memory_percent:.1f}%').replace('{memory_percent:.1f}', f'{memory_percent:.1f}'))
            
            # Kiểm tra disk space
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            free_gb = disk.free / (1024**3)
            
            if disk_percent > 95:
                error_msg = self.translator.get('message.low_disk_space', 
                    'Disk space is very low ({percent}% used, {free:.1f}GB free). This may cause crashes.').replace('{percent}', str(disk_percent)).replace('{free:.1f}', f'{free_gb:.1f}')
                self.add_log(f"❌ {error_msg}")
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    error_msg
                )
                return False
            elif disk_percent > 85:
                self.add_log(self.translator.get('log.disk_usage_high', 
                    f'⚠️ Disk usage is high: {disk_percent:.1f}%').replace('{disk_percent:.1f}', f'{disk_percent:.1f}'))
            
            self.add_log(self.translator.get('log.resource_check_passed', 
                f'✅ Resource check passed (Memory: {memory_percent:.1f}%, Disk: {disk_percent:.1f}%)').replace('{memory_percent:.1f}', f'{memory_percent:.1f}').replace('{disk_percent:.1f}', f'{disk_percent:.1f}'))
            return True
            
        except Exception as e:
            self.add_log(f"⚠️ {self.translator.get('log.resource_check_error', 'Error checking system resources: {error}').replace('{error}', str(e))}")
            # Không block, chỉ cảnh báo
            return True
    
    def _validate_bag_file(self):
        """Kiểm tra bag file có chứa các topics cần thiết"""
        if not self.bag_path:
            return True
        
        try:
            bag_path_obj = Path(self.bag_path)
            if not bag_path_obj.exists():
                return True
            
            self.add_log(self.translator.get('log.checking_bag_topics', '🔍 Checking bag file for required topics...'))
            
            # Kiểm tra bag info
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            ws_setup = self.workspace_path / "install" / "setup.sh"
            
            if not ws_setup.exists():
                # Không thể check, bỏ qua
                return True
            
            cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag info {self.bag_path}"
            result = subprocess.run(
                cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode != 0:
                self.add_log(self.translator.get('log.cannot_check_bag', '⚠️ Cannot check bag info, continuing...'))
                return True
            
            bag_output = result.stdout.lower()
            
            # Kiểm tra các topics cần thiết
            required_topics = ['/livox/lidar', '/livox/imu', '/image_raw']
            missing_topics = []
            
            for topic in required_topics:
                # Kiểm tra cả với và không có leading slash
                topic_variants = [topic, topic[1:], topic.replace('/', '_')]
                if not any(variant in bag_output for variant in topic_variants):
                    missing_topics.append(topic)
            
            if missing_topics:
                error_msg = self.translator.get('message.bag_missing_topics', 
                    'Bag file is missing required topics: {topics}\n\nThis may cause the mapping process to crash.').replace('{topics}', ', '.join(missing_topics))
                self.add_log(f"❌ {error_msg}")
                if not messagebox.askyesno(
                    self.translator.get('dialog.warning', 'Warning'),
                    f"{error_msg}\n\n{self.translator.get('message.continue_anyway', 'Continue anyway?')}"
                ):
                    return False
            else:
                self.add_log(self.translator.get('log.bag_topics_ok', '✅ All required topics found in bag file'))
            
            return True
            
        except subprocess.TimeoutExpired:
            self.add_log(self.translator.get('log.bag_check_timeout', '⚠️ Bag check timeout, continuing...'))
            return True
        except Exception as e:
            self.add_log(f"⚠️ {self.translator.get('log.bag_check_error', 'Error checking bag file: {error}').replace('{error}', str(e))}")
            # Không block, chỉ cảnh báo
            return True
    
    def _wait_for_process_health(self, max_wait_time=10):
        """Đợi process khởi động và kiểm tra health"""
        check_interval = 0.5
        max_checks = int(max_wait_time / check_interval)
        
        for i in range(max_checks):
            time.sleep(check_interval)
            
            if not self.mapping_process:
                return False
            
            # Kiểm tra process có còn chạy không
            if self.mapping_process.poll() is not None:
                # Process đã dừng
                return False
            
            # Kiểm tra output để tìm dấu hiệu khởi động thành công hoặc lỗi
            try:
                # Đọc một phần output nếu có
                if hasattr(self.mapping_process, 'stdout') and self.mapping_process.stdout:
                    # Không block, chỉ peek
                    pass
            except:
                pass
            
            # Sau 3 giây đầu tiên, coi như process đã khởi động
            if i >= 6:  # 3 seconds
                return True
        
        # Kiểm tra lần cuối
        if self.mapping_process and self.mapping_process.poll() is None:
            return True
        
        return False

    def monitor_mapping_process(self):
        """Theo dõi mapping process và phát hiện crash"""
        if not self.mapping_process:
            return
        
        try:
            for line in iter(self.mapping_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_mapping_running:
                    line_lower = line.lower()
                    # Log tất cả các dòng quan trọng
                    if any(keyword in line_lower for keyword in ['error', 'warning', 'started', 'ready', 'failed', 'died', 'crash', 'segmentation', 'signal']):
                        self.add_log(f"[Mapping] {line.strip()}")
                    
                    # Phát hiện process crash
                    if 'process has died' in line_lower or 'process died' in line_lower:
                        self.add_log(f"❌ [Mapping] {line.strip()}")
                        # Đợi một chút để process hoàn toàn dừng
                        time.sleep(0.5)
                        # Kiểm tra exit code
                        if self.mapping_process.poll() is not None:
                            exit_code = self.mapping_process.returncode
                            self._handle_mapping_crash(exit_code, line)
                            break
                else:
                    break
            
            # Kiểm tra lại sau khi đọc hết output
            if self.is_mapping_running and self.mapping_process:
                if self.mapping_process.poll() is not None:
                    exit_code = self.mapping_process.returncode
                    # Chỉ xử lý như crash nếu không phải graceful shutdown
                    if exit_code != 0 and not (self.is_stopping or exit_code == -15):
                        self._handle_mapping_crash(exit_code, "Process exited unexpectedly")
                        
        except Exception as e:
            if self.is_mapping_running:
                self.add_log(f"⚠️ {self.translator.get('log.error_reading_mapping_output', 'Error reading mapping output')}: {e}")
                # Kiểm tra xem process có còn chạy không
                if self.mapping_process and self.mapping_process.poll() is not None:
                    self._handle_mapping_crash(self.mapping_process.returncode, str(e))
    
    def _handle_mapping_crash(self, exit_code, error_line):
        """Xử lý khi mapping process bị crash"""
        if not self.is_mapping_running:
            return
        
        # Kiểm tra nếu đang dừng từ STOP button (Signal 15 = SIGTERM)
        # Signal 15 là graceful shutdown, không phải crash
        if self.is_stopping or exit_code == -15:
            # Đây là graceful shutdown từ STOP button, không phải crash
            self.is_mapping_running = False
            self.add_log(self.translator.get('log.mapping_stopped_by_user', '✅ Mapping process stopped by user'))
            return
        
        self.is_mapping_running = False
        
        # Xác định loại lỗi dựa trên exit code
        error_type = "Unknown error"
        error_description = ""
        
        if exit_code == -11:
            error_type = "Segmentation Fault (SIGSEGV)"
            error_description = self.translator.get('log.crash_segmentation_fault', 
                'The mapping process crashed due to a segmentation fault. This usually indicates:\n'
                '- Memory access violation\n'
                '- Invalid pointer dereference\n'
                '- Stack overflow\n'
                '- Corrupted memory\n\n'
                'Possible solutions:\n'
                '- Check config file parameters\n'
                '- Verify sensor data topics are available\n'
                '- Check system resources (memory, CPU)\n'
                '- Rebuild the workspace')
        elif exit_code == -6:
            error_type = "Abort (SIGABRT)"
            error_description = self.translator.get('log.crash_abort',
                'The mapping process was aborted. This usually indicates:\n'
                '- Assertion failure\n'
                '- Fatal error in the code\n'
                '- Resource allocation failure\n\n'
                'Check the log above for more details.')
        elif exit_code < 0:
            error_type = f"Signal {abs(exit_code)}"
            signal_num = abs(exit_code)
            error_description = self.translator.get('log.crash_signal',
                'The mapping process was terminated by signal {signal}.\nThis usually indicates a crash or abnormal termination.').replace('{signal}', str(signal_num))
        else:
            error_type = f"Exit code {exit_code}"
            error_description = self.translator.get('log.crash_exit_code',
                'The mapping process exited with code {code}.\nCheck the log above for error details.').replace('{code}', str(exit_code))
        
        # Log chi tiết
        self.add_log("=" * 60)
        self.add_log(f"❌ {self.translator.get('log.mapping_process_crashed', 'MAPPING PROCESS CRASHED')}")
        self.add_log(f"   {self.translator.get('log.error_type', 'Error Type')}: {error_type}")
        self.add_log(f"   {self.translator.get('log.exit_code', 'Exit Code')}: {exit_code}")
        self.add_log(f"   {self.translator.get('log.error_line', 'Error Line')}: {error_line[:200]}")
        self.add_log("=" * 60)
        
        # Cập nhật UI
        self.root.after(0, lambda: self.status_label.config(
            text=self.translator.get('label.status_crashed', 'Status: ❌ Crashed'),
            foreground="red"
        ))
        self.root.after(0, lambda: self.btn_start.config(state=tk.NORMAL))
        
        # Dừng bag playback nếu đang chạy
        if self.is_bag_playing:
            self.add_log(self.translator.get('log.stopping_bag_due_to_crash', '⚠️ Stopping bag playback due to mapping crash...'))
            self.cleanup_processes()
        
        # Hiển thị thông báo lỗi
        self.root.after(0, lambda: messagebox.showerror(
            self.translator.get('dialog.mapping_crashed_title', 'Mapping Process Crashed'),
            f"{self.translator.get('dialog.mapping_crashed_message', 'The mapping process has crashed!')}\n\n"
            f"{self.translator.get('dialog.error_type', 'Error Type')}: {error_type}\n"
            f"{self.translator.get('dialog.exit_code', 'Exit Code')}: {exit_code}\n\n"
            f"{error_description}"
        ))
    
    def monitor_bag_process(self):
        """Theo dõi output của ros2 bag play"""
        if not self.bag_process:
            return
        
        try:
            for line in iter(self.bag_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_bag_playing:
                    if 'paused' in line.lower() or 'playing' in line.lower():
                        self.add_log(f"[Bag] {line.strip()}")
                else:
                    break
            
            if self.is_bag_playing:
                self.add_log(self.translator.get('log.bag_playback_finished', '✅ Bag playback finished'))
                self.is_bag_playing = False
                # Hiển thị thông báo khi bag playback kết thúc
                self.root.after(0, lambda: messagebox.showinfo(
                    self.translator.get('dialog.bag_playback_finished_title', 'Bag Playback Finished'),
                    self.translator.get('dialog.bag_playback_finished_message', 'Bag playback has completed successfully!')
                ))
        except Exception as e:
            if self.is_bag_playing:
                self.add_log(self.translator.get('log.error_reading_bag', '⚠️ Error reading bag output: {error}').replace('{error}', str(e)))
    
    def start_qr_scanning_subscriber(self):
        """Bắt đầu ROS2 subscriber để quét QR code từ ảnh trong bag"""
        if not ROS2_AVAILABLE:
            self.add_log(self.translator.get('log.ros2_not_available', '⚠️ ROS2 not available. QR scanning from bag disabled.'))
            return
        
        if not self.qr_scanner:
            self.add_log(self.translator.get('log.qr_scanner_not_available', '⚠️ QR scanner is not available. Please install pyzbar and geometry_utils.'))
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.add_log(self.translator.get('log.starting_qr_subscriber', '🔍 Starting QR code scanning subscriber...'))
            
            # Tạo ROS2 node cho QR scanning (subscribe cả image và odometry)
            self.qr_ros_node = QRImageSubscriberNode(
                '/image_raw',
                '/aft_mapped_to_init',
                self.on_qr_image_received,
                self.on_odometry_received,
                'bag_qr_scanner'
            )
            
            # Tạo executor
            self.qr_ros_executor = SingleThreadedExecutor()
            self.qr_ros_executor.add_node(self.qr_ros_node)
            
            # Khởi động ROS thread
            self.qr_ros_thread = threading.Thread(target=self.qr_ros_spin, daemon=True)
            self.qr_ros_thread.start()
            
            self.qr_frame_count = 0
            self.add_log(self.translator.get('log.qr_subscriber_started', '✅ QR scanning subscriber started'))
            
        except Exception as e:
            self.add_log(self.translator.get('log.error_starting_qr_subscriber', '❌ Error starting QR subscriber: {error}').replace('{error}', str(e)))
            import traceback
            traceback.print_exc()
    
    def qr_ros_spin(self):
        """Spin ROS node trong thread riêng cho QR scanning"""
        try:
            while rclpy.ok() and self.is_bag_playing:
                if self.qr_ros_executor is not None:
                    self.qr_ros_executor.spin_once(timeout_sec=0.1)
                else:
                    break
        except Exception as e:
            if self.is_bag_playing:
                self.add_log(self.translator.get('log.error_qr_ros_spin', '⚠️ Error in QR ROS spin: {error}').replace('{error}', str(e)))
    
    def on_qr_image_received(self, cv_image):
        """Callback khi nhận được ảnh từ bag để quét QR"""
        try:
            self.qr_frame_count += 1
            
            # Chỉ quét QR mỗi N frame để tối ưu performance
            if self.qr_frame_count % self.qr_scan_frame_interval == 0:
                # Quét QR code trong background thread để không block ROS callback
                threading.Thread(
                    target=self._scan_qr_in_background,
                    args=(cv_image.copy(),),
                    daemon=True
                ).start()
            
        except Exception as e:
            self.add_log(self.translator.get('log.error_qr_image_callback', '⚠️ Error in QR image callback: {error}').replace('{error}', str(e)))
    
    def _scan_qr_in_background(self, cv_image):
        """Quét QR code trong background thread"""
        try:
            qr_codes = self.scan_qr_from_image(cv_image)
            # scan_qr_from_image đã tự động gọi on_qr_codes_received nếu tìm thấy QR
        except Exception as e:
            # Lỗi đã được log trong scan_qr_from_image
            pass
    
    def stop_qr_scanning_subscriber(self):
        """Dừng ROS2 subscriber cho QR scanning"""
        if self.qr_ros_executor:
            try:
                self.qr_ros_executor.shutdown()
            except:
                pass
            self.qr_ros_executor = None
        
        if self.qr_ros_node:
            try:
                self.qr_ros_node.destroy_node()
            except:
                pass
            self.qr_ros_node = None
        
        self.qr_ros_thread = None
        self.qr_frame_count = 0
    
    def cleanup_processes(self):
        # Đánh dấu đang dừng để tránh hiển thị dialog crash
        self.is_stopping = True
        
        # Dừng QR scanning subscriber trước
        self.stop_qr_scanning_subscriber()
        # Dừng bag play trước
        if self.bag_process:
            try:
                self.add_log(self.translator.get('log.stopping_bag_playback', 'Stopping bag playback...'))
                if hasattr(os, 'setsid'):
                    try:
                        os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                else:
                    self.bag_process.terminate()
                
                try:
                    self.bag_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if hasattr(os, 'setsid'):
                        try:
                            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        self.bag_process.kill()
                    self.bag_process.wait()
            except Exception as e:
                self.add_log(self.translator.get('log.error_stopping_bag', '⚠️ Error stopping bag process: {error}').replace('{error}', str(e)))
            finally:
                self.bag_process = None
                self.is_bag_playing = False
        
        # Sau đó dừng mapping node
        if self.mapping_process:
            try:
                self.add_log(self.translator.get('log.stopping_mapping_node', 'Stopping mapping node...'))
                if hasattr(os, 'setsid'):
                    try:
                        os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                else:
                    self.mapping_process.terminate()
                
                try:
                    self.mapping_process.wait(timeout=10)
                    self.add_log(self.translator.get('log.mapping_node_stopped', '✅ Mapping node stopped gracefully'))
                except subprocess.TimeoutExpired:
                    if hasattr(os, 'setsid'):
                        try:
                            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        self.mapping_process.kill()
                    self.mapping_process.wait()
            except Exception as e:
                self.add_log(f"⚠️ {self.translator.get('log.error_stopping_mapping', 'Error stopping mapping process')}: {e}")
            finally:
                self.mapping_process = None
                self.is_mapping_running = False
        
        # Cleanup ROS2 resources sau khi dừng tất cả processes
        self.cleanup_ros2_resources()
        
        # Kiểm tra PCD files và cập nhật status
        if self.check_pcd_files():
            self.upload_stat.config(text="Ready to upload")
            self.add_log(self.translator.get('log.pcd_detected', '✅ PCD files detected, ready to upload'))
        else:
            self.upload_stat.config(text="Waiting for PCD files...")
        
        # Reset flag sau khi cleanup xong
        self.is_stopping = False
    
    # --- PCD Processing Functions ---
    def merge_pcd_files(self, silent=False):
        """Gộp tất cả các file PCD trong thư mục Log/PCD thành một file lớn"""
        pcd_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "PCD"
        
        if not pcd_dir.exists():
            if not silent:
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    self.translator.get('dialog.pcd_dir_not_exists', 'PCD directory does not exist: {path}').replace('{path}', str(pcd_dir))
                )
            self.add_log(self.translator.get('log.pcd_dir_not_exists', '❌ PCD directory does not exist: {path}').replace('{path}', str(pcd_dir)))
            return False
        
        # Tìm tất cả file PCD (bỏ qua file merged nếu có)
        pcd_files = sorted([f for f in pcd_dir.glob("*.pcd") if f.name != "merged_all.pcd"])
        
        if not pcd_files:
            if not silent:
                messagebox.showwarning(
                    self.translator.get('dialog.warning', 'Warning'),
                    self.translator.get('dialog.no_pcd_files_warning', 'No PCD files found in directory: {path}').replace('{path}', str(pcd_dir))
                )
            self.add_log(self.translator.get('log.no_pcd_files', '⚠️ No PCD files found in: {path}').replace('{path}', str(pcd_dir)))
            return False
        
        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.merge_start', '🔗 Starting to merge {count} PCD files...').replace('{count}', str(len(pcd_files))))
        self.add_log(self.translator.get('log.merge_directory', '📁 Directory: {path}').replace('{path}', str(pcd_dir)))
        
        # Disable button during merge
        if hasattr(self, 'btn_merge_pcd'):
            self.btn_merge_pcd.config(state=tk.DISABLED)
        
        try:
            # Kiểm tra open3d
            try:
                import open3d as o3d
            except ImportError:
                error_msg = self.translator.get('dialog.open3d_not_installed', 
                    'open3d library is not installed.\n\nPlease install with:\npip install open3d\n\nOr add to requirements.txt and reinstall.')
                if not silent:
                    messagebox.showerror(
                        self.translator.get('dialog.library_missing', 'Missing Library'),
                        error_msg
                    )
                self.add_log(self.translator.get('log.open3d_not_installed', '❌ open3d is not installed. Please install: pip install open3d'))
                return False
            
            merged_cloud = None
            total_points = 0
            
            # Sử dụng open3d để merge
            for i, pcd_file in enumerate(pcd_files, 1):
                try:
                    self.add_log(self.translator.get('log.reading_pcd_file', '📖 Reading file {current}/{total}: {filename}').replace('{current}', str(i)).replace('{total}', str(len(pcd_files))).replace('{filename}', pcd_file.name))
                    cloud = o3d.io.read_point_cloud(str(pcd_file))
                    
                    if len(cloud.points) == 0:
                        self.add_log(self.translator.get('log.pcd_file_empty', '⚠️ File {filename} is empty, skipping').replace('{filename}', pcd_file.name))
                        continue
                    
                    if merged_cloud is None:
                        merged_cloud = cloud
                    else:
                        merged_cloud += cloud
                    
                    total_points += len(cloud.points)
                    self.add_log(self.translator.get('log.points_added', '   ✓ Added {count:,} points from {filename}').replace('{count:,}', f'{len(cloud.points):,}').replace('{filename}', pcd_file.name))
                    
                except Exception as e:
                    self.add_log(self.translator.get('log.error_reading_pcd', '❌ Error reading {filename}: {error}').replace('{filename}', pcd_file.name).replace('{error}', str(e)))
                    continue
            
            if merged_cloud is None or len(merged_cloud.points) == 0:
                if not silent:
                    messagebox.showerror(
                        self.translator.get('dialog.error', 'Error'),
                        self.translator.get('dialog.merge_no_valid_points', 'Cannot merge PCD files. No valid points.')
                    )
                self.add_log(self.translator.get('log.no_points_to_merge', '❌ No points to merge'))
                return False
            
            # Tạo folder riêng cho merged map
            merged_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "merged_map"
            merged_map_dir.mkdir(parents=True, exist_ok=True)
            
            # Lưu file merged vào folder riêng
            output_file = merged_map_dir / "merged_all.pcd"
            self.add_log(self.translator.get('log.saving_merged_file', '💾 Saving merged file: {filename}').replace('{filename}', output_file.name))
            self.add_log(f"📁 Output folder: {merged_map_dir}")
            o3d.io.write_point_cloud(str(output_file), merged_cloud)
            
            # Kiểm tra kết quả
            if output_file.exists():
                size_mb = output_file.stat().st_size / (1024 * 1024)
                self.add_log("=" * 60)
                self.add_log(self.translator.get('log.merge_success', '✅ Merge PCD successful!'))
                self.add_log(self.translator.get('log.merge_output_file', '📁 Output file: {filename}').replace('{filename}', output_file.name))
                self.add_log(self.translator.get('log.merge_total_points', '📊 Total points: {count:,}').replace('{count:,}', f'{total_points:,}'))
                self.add_log(self.translator.get('log.merge_file_size', '💾 File size: {size:.2f} MB').replace('{size:.2f}', f'{size_mb:.2f}'))
                self.add_log("=" * 60)
                if not silent:
                    messagebox.showinfo(
                        self.translator.get('dialog.merge_success_title', 'Success'),
                        self.translator.get('dialog.merge_success_message', 
                            'Successfully merged {count} PCD files!\n\nOutput file: {filename}\nTotal points: {points:,}\nFile size: {size:.2f} MB')
                            .replace('{count}', str(len(pcd_files)))
                            .replace('{filename}', output_file.name)
                            .replace('{points:,}', f'{total_points:,}')
                            .replace('{size:.2f}', f'{size_mb:.2f}')
                    )
                return True
            else:
                raise Exception("File output không được tạo")
                
        except Exception as e:
            error_msg = f"Error merging PCD files: {e}"
            self.add_log(f"❌ {error_msg}")
            if not silent:
                messagebox.showerror(
                    self.translator.get('dialog.error', 'Error'),
                    error_msg
                )
            return False
        finally:
            pass  # Button được quản lý bởi upload pipeline
    
    def _run_hba_core(self):
        """Core logic của HBA optimization (đồng bộ) - tạo folder riêng"""
        pcd_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "PCD"
        merged_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "merged_map"
        hba_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "hba_map"
        hba_script = Path(__file__).parent.parent / "scripts" / "normalize_map_hba.py"
        
        if not hba_script.exists():
            self.add_log(self.translator.get('log.hba_script_not_found', '❌ HBA script not found at: {path}').replace('{path}', str(hba_script)))
            return False

        # scans_pos.json luôn nằm trong PCD gốc
        pose_file = pcd_dir / "scans_pos.json"
        if not pose_file.exists():
            self.add_log(self.translator.get('log.scans_pos_missing', '❌ Missing scans_pos.json in {path}').replace('{path}', str(pcd_dir)))
            return False

        # Kiểm tra input: ưu tiên merged_map, fallback về PCD
        merged_pcd = merged_map_dir / "merged_all.pcd"
        if merged_pcd.exists():
            input_dir = merged_map_dir
            input_pcd = merged_pcd
        else:
            # Nếu không có merged_map, tìm file PCD đầu tiên trong PCD gốc
            pcd_files = sorted([f for f in pcd_dir.glob("*.pcd") if f.name not in ["merged_all.pcd", "merge_all_hba.pcd"]])
            if not pcd_files:
                self.add_log(self.translator.get('log.no_pcd_input', '❌ No PCD input files found'))
                return False
            input_dir = pcd_dir
            input_pcd = pcd_files[0]
        
        if not input_dir.exists():
            self.add_log(self.translator.get('log.input_dir_not_exists', '❌ Input directory does not exist: {path}').replace('{path}', str(input_dir)))
            return False

        # Tạo folder riêng cho HBA output
        hba_map_dir.mkdir(parents=True, exist_ok=True)
        
        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.hba_start', '🪄 Starting map optimization with HBA Standalone...'))
        self.add_log(self.translator.get('log.hba_input', '📁 Input: {path}').replace('{path}', str(input_dir)))
        self.add_log(f"📁 Input PCD: {input_pcd.name}")
        self.add_log(self.translator.get('log.hba_output', '📁 Output: {path}').replace('{path}', str(hba_map_dir)))

        try:
            import shutil
            
            # Copy scans_pos.json vào input_dir nếu chưa có (HBA script cần file này)
            input_pose_file = input_dir / "scans_pos.json"
            pose_file_copied = False
            if not input_pose_file.exists() and pose_file.exists():
                shutil.copy2(pose_file, input_pose_file)
                pose_file_copied = True
                self.add_log(self.translator.get('log.copied_scans_pos', '📋 Copied scans_pos.json to {path}').replace('{path}', str(input_dir)))
            
            # Chạy HBA với input_dir (script sẽ tạo file merge_all_hba.pcd trong input_dir)
            cmd = f"python3 {hba_script} --input_dir {input_dir}"
            process = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.add_log(f"[HBA] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                # Script HBA tạo file merge_all_hba.pcd trong input_dir, di chuyển vào hba_map_dir
                source_file = input_dir / "merge_all_hba.pcd"
                if source_file.exists():
                    # Di chuyển file output vào folder riêng (không để trong merged_map)
                    output_file = hba_map_dir / "merge_all_hba.pcd"
                    shutil.move(source_file, output_file)
                    self.add_log(self.translator.get('log.moved_hba_file', '📦 Moved merge_all_hba.pcd from {from} to {to}').replace('{from}', str(input_dir)).replace('{to}', str(hba_map_dir)))
                    
                    # Copy scans_pos.json vào hba_map_dir
                    if input_pose_file.exists():
                        shutil.copy2(input_pose_file, hba_map_dir / "scans_pos.json")
                    
                    # Xóa scans_pos.json khỏi merged_map nếu đã copy tạm (giữ merged_map sạch, chỉ có merged_all.pcd)
                    if input_dir == merged_map_dir and pose_file_copied and input_pose_file.exists():
                        try:
                            input_pose_file.unlink()
                            self.add_log(self.translator.get('log.removed_temp_scans_pos', '🧹 Removed temporary scans_pos.json from {path}').replace('{path}', str(merged_map_dir)))
                        except Exception as e:
                            self.add_log(self.translator.get('log.cannot_remove_temp', '⚠️ Cannot remove temporary scans_pos.json: {error}').replace('{error}', str(e)))
                    
                    self.add_log(self.translator.get('log.hba_complete', '✅ HBA Optimization complete!'))
                    self.add_log(f"📁 Output: {output_file}")
                    self.add_log(self.translator.get('log.merged_map_contains', '📁 merged_map contains: merged_all.pcd'))
                    self.add_log(self.translator.get('log.hba_map_contains', '📁 hba_map contains: merge_all_hba.pcd + scans_pos.json'))
                    return True
                else:
                    self.add_log(self.translator.get('log.hba_no_output', '⚠️ HBA completed but output file not found.'))
                    self.add_log(self.translator.get('log.checked_output', '   Checked: {path}').replace('{path}', str(source_file)))
                    return False
            else:
                self.add_log(self.translator.get('log.hba_failed', '❌ HBA failed with code {code}').replace('{code}', str(process.returncode)))
                return False
        except Exception as e:
            self.add_log(self.translator.get('log.hba_error', '❌ HBA error: {error}').replace('{error}', str(e)))
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()}")
            return False
    
    def _run_sc_core(self):
        """Core logic của ScanContext tiling (đồng bộ) - sử dụng input từ hba_map với error handling tốt hơn"""
        hba_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "hba_map"
        merged_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "merged_map"
        sc_script = Path(__file__).parent.parent / "scripts" / "generate_fast_localization_map.py"
        output_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "fastloc_map"

        # Kiểm tra script tồn tại
        if not sc_script.exists():
            self.add_log(self.translator.get('log.sc_script_not_found', '❌ ScanContext script not found: {path}').replace('{path}', str(sc_script)))
            return False

        # Ưu tiên input từ hba_map, fallback về merged_map
        input_pcd = hba_map_dir / "merge_all_hba.pcd"
        if not input_pcd.exists():
            self.add_log(self.translator.get('log.hba_not_found_try_merged', '⚠️ HBA map not found, trying merged_all.pcd...'))
            input_pcd = merged_map_dir / "merged_all.pcd"
            if not input_pcd.exists():
                self.add_log(self.translator.get('log.no_merged_pcd_found', '❌ No merged PCD files found'))
                return False

        # Validate input file
        try:
            file_size = input_pcd.stat().st_size
            file_size_mb = file_size / (1024 * 1024)
            
            if file_size == 0:
                self.add_log(self.translator.get('log.sc_input_empty', '❌ Input PCD file is empty'))
                return False
            
            self.add_log(self.translator.get('log.sc_input_size', '📦 Input file size: {size:.2f} MB').replace('{size:.2f}', f'{file_size_mb:.2f}'))
            
            # Cảnh báo nếu file quá lớn (>1GB)
            if file_size > 1024 * 1024 * 1024:
                self.add_log(self.translator.get('log.sc_file_very_large', 
                    '⚠️ Warning: Input file is very large ({size:.1f}GB). Processing may take a long time and use significant memory.').replace('{size:.1f}', f'{file_size_mb/1024:.1f}'))
        except OSError as e:
            self.add_log(self.translator.get('log.sc_cannot_read_input', '❌ Cannot read input file: {error}').replace('{error}', str(e)))
            return False

        # Kiểm tra memory trước khi chạy
        try:
            import psutil
            memory = psutil.virtual_memory()
            available_gb = memory.available / (1024**3)
            
            # Ước tính memory cần thiết: khoảng 3-5x file size
            estimated_memory_gb = (file_size_mb / 1024) * 4
            
            if available_gb < estimated_memory_gb:
                self.add_log(self.translator.get('log.sc_low_memory', 
                    '⚠️ Warning: Available memory ({available:.1f}GB) may be insufficient for processing ({estimated:.1f}GB estimated).').replace('{available:.1f}', f'{available_gb:.1f}').replace('{estimated:.1f}', f'{estimated_memory_gb:.1f}'))
        except ImportError:
            # psutil không có, bỏ qua check
            pass
        except Exception as e:
            self.add_log(f"⚠️ Warning: Cannot check memory: {e}")

        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.sc_start', '🗺️ Preparing ScanContext map from {filename}...').replace('{filename}', input_pcd.name))
        self.add_log(f"📁 Input: {input_pcd.parent}")
        self.add_log(f"📁 Output: {output_dir}")

        # Tạo output directory trước
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.add_log(self.translator.get('log.sc_cannot_create_output', '❌ Cannot create output directory: {error}').replace('{error}', str(e)))
            return False

        process = None
        try:
            # Tính toán timeout động: ít nhất 300s (5 phút), thêm 60s cho mỗi 100MB
            base_timeout = 300
            timeout_per_100mb = 60
            dynamic_timeout = base_timeout + int((file_size_mb / 100) * timeout_per_100mb)
            # Giới hạn timeout tối đa 3600s (1 giờ)
            process_timeout = min(dynamic_timeout, 3600)
            
            self.add_log(self.translator.get('log.sc_timeout', '⏱️ Process timeout: {timeout}s').replace('{timeout}', str(process_timeout)))
            
            cmd = (
                f"python3 {sc_script} "
                f"--input_pcd {input_pcd} "
                f"--output_dir {output_dir} "
                f"--strip_color --voxel_size 0.2 --tile_size 50.0"
            )
            
            self.add_log(self.translator.get('log.sc_running', '▶️ Running ScanContext script...'))
            
            process = subprocess.Popen(
                cmd, 
                shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.STDOUT, 
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # Đọc output với timeout
            output_lines = []
            start_time = time.time()
            
            while True:
                # Kiểm tra timeout
                elapsed = time.time() - start_time
                if elapsed > process_timeout:
                    self.add_log(self.translator.get('log.sc_timeout_reached', '❌ Process timeout reached ({timeout}s). Terminating...').replace('{timeout}', str(process_timeout)))
                    try:
                        if hasattr(os, 'setsid'):
                            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        else:
                            process.terminate()
                    except:
                        pass
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try:
                            if hasattr(os, 'setsid'):
                                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                            else:
                                process.kill()
                        except:
                            pass
                    return False
                
                # Đọc line với timeout ngắn
                try:
                    line = process.stdout.readline()
                    if not line:
                        # Process đã kết thúc
                        break
                    if line.strip():
                        output_lines.append(line.strip())
                        self.add_log(f"[ScanContext] {line.strip()}")
                except Exception as e:
                    self.add_log(f"⚠️ Error reading output: {e}")
                    break
            
            # Đợi process kết thúc
            try:
                return_code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.add_log(self.translator.get('log.sc_process_hanging', '⚠️ Process seems to be hanging, forcing termination...'))
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    else:
                        process.kill()
                except:
                    pass
                return False
            
            # Kiểm tra exit code
            if return_code == 0:
                # Validate output
                index_file = output_dir / "index.json"
                if not index_file.exists():
                    self.add_log(self.translator.get('log.sc_no_index_file', '⚠️ Process completed but index.json not found'))
                    return False
                
                # Đếm số tile files - tiles được lưu trong thư mục con "pcd"
                tiles_dir = output_dir / "pcd"
                if tiles_dir.exists():
                    tile_files = list(tiles_dir.glob("*.pcd"))
                else:
                    # Fallback: tìm trong output_dir trực tiếp (cho tương thích ngược)
                    tile_files = list(output_dir.glob("*.pcd"))
                
                if len(tile_files) == 0:
                    self.add_log(self.translator.get('log.sc_no_tiles', '⚠️ Process completed but no tile files found'))
                    # Log thêm thông tin để debug
                    if tiles_dir.exists():
                        self.add_log(f"   Checked directory: {tiles_dir}")
                        all_files = list(tiles_dir.glob("*"))
                        if all_files:
                            self.add_log(f"   Found {len(all_files)} files in pcd directory (but no .pcd files)")
                    return False
                
                self.add_log("=" * 60)
                self.add_log(self.translator.get('log.sc_success', '✅ ScanContext Tiling successful!'))
                self.add_log(f"📁 Output: {output_dir}")
                self.add_log(self.translator.get('log.sc_tiles_created', '📊 Tiles created: {count}').replace('{count}', str(len(tile_files))))
                self.add_log("=" * 60)
                return True
            else:
                # Process failed
                error_msg = self.translator.get('log.sc_failed', '❌ Tiling failed with code {code}').replace('{code}', str(return_code))
                self.add_log(error_msg)
                
                # Log last few lines of output để debug
                if output_lines:
                    self.add_log(self.translator.get('log.sc_last_output', 'Last output lines:'))
                    for line in output_lines[-10:]:
                        self.add_log(f"   {line}")
                
                return False
                
        except subprocess.TimeoutExpired:
            self.add_log(self.translator.get('log.sc_timeout_error', '❌ Process timeout'))
            if process:
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    else:
                        process.kill()
                except:
                    pass
            return False
        except KeyboardInterrupt:
            self.add_log(self.translator.get('log.sc_interrupted', '⚠️ Process interrupted by user'))
            if process:
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    else:
                        process.terminate()
                except:
                    pass
            return False
        except Exception as e:
            error_msg = self.translator.get('log.sc_error', '❌ ScanContext error: {error}').replace('{error}', str(e))
            self.add_log(error_msg)
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()[:500]}")
            
            # Cleanup process nếu còn chạy
            if process:
                try:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    else:
                        process.terminate()
                except:
                    pass
            return False
    
    def _generate_floorplan(self):
        """Generate floorplan from PCD file using pcd_to_3_views - tạo folder riêng cho 2D"""
        hba_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "hba_map"
        merged_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "merged_map"
        
        # Prefer merge_all_hba.pcd từ hba_map, fallback to merged_all.pcd từ merged_map
        input_pcd = hba_map_dir / "merge_all_hba.pcd"
        if not input_pcd.exists():
            self.add_log(self.translator.get('log.floorplan_hba_not_found', '⚠️ merge_all_hba.pcd not found, trying merged_all.pcd...'))
            input_pcd = merged_map_dir / "merged_all.pcd"
            if not input_pcd.exists():
                self.add_log(self.translator.get('log.floorplan_no_pcd_found', '❌ Neither merge_all_hba.pcd nor merged_all.pcd found for floorplan generation'))
                return False
        
        # Tạo folder riêng cho floorplan (2D)
        floorplan_output_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "floorplan_2d"
        floorplan_output_dir.mkdir(parents=True, exist_ok=True)
        
        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.floorplan_generating', '🗺️ Generating floorplan from: {filename}').replace('{filename}', input_pcd.name))
        self.add_log(self.translator.get('log.floorplan_output_dir', '📁 Output directory: {path}').replace('{path}', str(floorplan_output_dir)))
        
        try:
            # Setup path to import pcd_to_floorplan từ thư mục scripts của project
            script_dir = Path(__file__).parent.parent / "scripts"
            if script_dir not in [Path(p) for p in sys.path]:
                sys.path.insert(0, str(script_dir))
            
            # Dynamic import using importlib to avoid linter warnings
            import importlib
            pcd_module = importlib.import_module("pcd_to_floorplan")
            pcd_to_3_views = pcd_module.pcd_to_3_views
            
            # Generate 3 views with same parameters as backend
            result = pcd_to_3_views(
                str(input_pcd),
                output_dir=str(floorplan_output_dir),
                resolution=0.05,
                colormap='binary',
                invert_colors=True,
                auto_crop=True,
                crop_margin=5,
                border_margin=20,
                outlier_filter=True,
                outlier_percentile=1.0
            )
            
            # Validate output files
            expected_files = [
                floorplan_output_dir / f"{input_pcd.stem}_top.png",
                floorplan_output_dir / f"{input_pcd.stem}_side_x.png",
                floorplan_output_dir / f"{input_pcd.stem}_side_y.png",
                floorplan_output_dir / f"{input_pcd.stem}_metadata.json"
            ]
            
            all_exist = all(f.exists() for f in expected_files)
            if not all_exist:
                missing = [f.name for f in expected_files if not f.exists()]
                self.add_log(self.translator.get('log.floorplan_missing_files', '❌ Missing floorplan files: {files}').replace('{files}', ', '.join(missing)))
                return False
            
            self.add_log("=" * 60)
            self.add_log(self.translator.get('log.floorplan_success', '✅ Floorplan generated successfully!'))
            self.add_log(f"📁 Directory: {floorplan_output_dir}")
            self.add_log(self.translator.get('log.floorplan_metadata', '📄 Metadata: {path}').replace('{path}', result.get('metadata_path', 'N/A')))
            self.add_log("=" * 60)
            return True
            
        except ImportError as e:
            self.add_log(self.translator.get('log.floorplan_import_error', '❌ Failed to import pcd_to_floorplan: {error}').replace('{error}', str(e)))
            self.add_log(self.translator.get('log.floorplan_check_path', '   Check path: {path}').replace('{path}', str(script_dir)))
            return False
        except Exception as e:
            self.add_log(self.translator.get('log.floorplan_generation_error', '❌ Error generating floorplan: {error}').replace('{error}', str(e)))
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()}")
            return False
    
    def _save_qr_codes_to_json(self):
        """Kiểm tra và đảm bảo QR_detect.json tồn tại"""
        try:
            # Kiểm tra file QR_detect.json có tồn tại và có dữ liệu không
            if self.qr_detect_json_path.exists():
                try:
                    with open(self.qr_detect_json_path, 'r', encoding='utf-8') as f:
                        qr_dict = json.load(f)
                    
                    if qr_dict:
                        self.add_log(f"✅ QR_detect.json exists with {len(qr_dict)} QR codes")
                        return self.qr_detect_json_path
                    else:
                        self.add_log(self.translator.get('log.no_qr_codes_to_save', '⚠️ No QR codes to save'))
                        return None
                except Exception as e:
                    self.add_log(self.translator.get('log.error_reading_qr_json', '⚠️ Error reading QR_detect.json: {error}').replace('{error}', str(e)))
                    return None
            else:
                # Nếu file không tồn tại, thử lấy từ memory và lưu
                with self.qr_codes_lock:
                    if not self.detected_qr_codes:
                        self.add_log(self.translator.get('log.no_qr_codes_to_save', '⚠️ No QR codes to save'))
                        return None
                    
                    # Convert từ memory format sang dict format và lưu
                    qr_dict = {}
                    for qr in self.detected_qr_codes:
                        qr_data = qr.get("data", "")
                        position = qr.get("position", [0.0, 0.0, 0.0])
                        qr_dict[qr_data] = position
                    
                    # Lưu vào file
                    log_dir = self.qr_detect_json_path.parent
                    log_dir.mkdir(parents=True, exist_ok=True)
                    
                    with open(self.qr_detect_json_path, 'w', encoding='utf-8') as f:
                        json.dump(qr_dict, f, indent=2, ensure_ascii=False)
                    
                    self.add_log(f"✅ QR_detect.json saved with {len(qr_dict)} QR codes")
                    return self.qr_detect_json_path
                    
        except Exception as e:
            self.add_log(self.translator.get('log.error_saving_qr', '❌ Error saving QR codes: {error}').replace('{error}', str(e)))
            return None
    
    def _zip_map_folders(self):
        """Zip các folder map: merged_map, hba_map, fastloc_map, floorplan_2d và file QR_detect.json"""
        log_root = self.workspace_path / "src" / "FAST-LIVO2" / "Log"
        
        # Các folder cần zip
        folders_to_zip = [
            "merged_map",
            "hba_map", 
            "fastloc_map",
            "floorplan_2d"
        ]
        
        # Kiểm tra xem có folder nào tồn tại không
        existing_folders = []
        for folder_name in folders_to_zip:
            folder_path = log_root / folder_name
            if folder_path.exists() and any(folder_path.iterdir()):
                existing_folders.append(folder_name)
        
        # Kiểm tra file QR_detect.json
        qr_detect_file = log_root / "QR_detect.json"
        has_qr_file = qr_detect_file.exists() and qr_detect_file.stat().st_size > 0
        
        if not existing_folders and not has_qr_file:
            self.add_log(self.translator.get('log.zip_no_folders', '❌ No folders found to zip'))
            return None
        
        # Tạo output directory
        base_output_dir = Path(__file__).parent.parent / "output"
        v_num = 1
        while (base_output_dir / f"version1.{v_num}").exists():
            v_num += 1
        
        output_root = base_output_dir / f"version1.{v_num}"
        output_root.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        zip_filename = f"map_v1.{v_num}_{timestamp}.zip"
        zip_path = output_root / zip_filename
        
        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.zip_creating', '📦 Creating zip file: {filename}').replace('{filename}', zip_path.name))
        zip_items = existing_folders.copy()
        if has_qr_file:
            zip_items.append("QR_detect.json")
        self.add_log(self.translator.get('log.zip_folders', '📁 Items to zip: {folders}').replace('{folders}', ', '.join(zip_items)))
        
        try:
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
                # Zip các folders
                for folder_name in existing_folders:
                    folder_path = log_root / folder_name
                    if folder_path.exists():
                        for root, dirs, files in os.walk(folder_path):
                            for file in files:
                                file_path = Path(root) / file
                                # Tạo arcname với folder name prefix
                                arcname = Path(folder_name) / file_path.relative_to(folder_path)
                                zipf.write(file_path, arcname)
                                self.add_log(self.translator.get('log.zip_file_added', '   ✓ Added: {path}').replace('{path}', str(arcname)))
                
                # Zip file QR_detect.json nếu có
                if has_qr_file:
                    zipf.write(qr_detect_file, "QR_detect.json")
                    self.add_log(self.translator.get('log.zip_file_added', '   ✓ Added: {path}').replace('{path}', 'QR_detect.json'))
            
            if zip_path.exists():
                size_mb = zip_path.stat().st_size / (1024 * 1024)
                self.add_log("=" * 60)
                self.add_log(self.translator.get('log.zip_success', '✅ Zip file created successfully!'))
                self.add_log(self.translator.get('log.zip_file', '📁 File: {path}').replace('{path}', str(zip_path)))
                self.add_log(self.translator.get('log.zip_size', '💾 Size: {size:.2f} MB').replace('{size:.2f}', f'{size_mb:.2f}'))
                self.add_log("=" * 60)
                return zip_path
            else:
                self.add_log(self.translator.get('log.zip_not_created', '❌ Zip file was not created'))
                return None
                
        except Exception as e:
            self.add_log(self.translator.get('log.zip_error', '❌ Error creating zip file: {error}').replace('{error}', str(e)))
            import traceback
            self.add_log(f"   Details: {traceback.format_exc()}")
            return None
    
    def _upload_map_to_backend(self, zip_path: Path, map_name: str, vehicle_id: str):
        """Upload file zip map lên backend với retry mechanism và error handling tốt hơn."""
        if not REQUESTS_AVAILABLE:
            msg = self.translator.get('log.upload_backend_requests_missing', 'Missing requests library. Install with: pip install requests')
            self.add_log(f"❌ {msg}")
            return False, msg
        
        zip_path = Path(zip_path)
        
        # Validate file tồn tại và có thể đọc được
        if not zip_path.exists():
            error_msg = self.translator.get('log.upload_backend_file_not_found', 'File not found: {path}').replace('{path}', str(zip_path))
            self.add_log(f"❌ {error_msg}")
            return False, error_msg
        
        # Kiểm tra file size để tránh upload file quá lớn hoặc rỗng
        try:
            file_size = zip_path.stat().st_size
            file_size_mb = file_size / (1024 * 1024)
            
            if file_size == 0:
                error_msg = self.translator.get('log.upload_file_empty', 'File is empty: {path}').replace('{path}', str(zip_path))
                self.add_log(f"❌ {error_msg}")
                return False, error_msg
            
            # Cảnh báo nếu file quá lớn (>500MB)
            if file_size > 500 * 1024 * 1024:
                self.add_log(self.translator.get('log.upload_file_large', 
                    '⚠️ Warning: File is very large ({size:.1f}MB). Upload may take a long time.').replace('{size:.1f}', f'{file_size_mb:.1f}'))
            
            self.add_log(self.translator.get('log.upload_file_size', 
                '📦 File size: {size:.2f} MB').replace('{size:.2f}', f'{file_size_mb:.2f}'))
        except OSError as e:
            error_msg = self.translator.get('log.upload_cannot_read_file', 
                'Cannot read file: {error}').replace('{error}', str(e))
            self.add_log(f"❌ {error_msg}")
            return False, error_msg
        
        upload_url = f"{self.backend_base_url.rstrip('/')}/api/v1/maps/upload"
        self.add_log(self.translator.get('log.upload_endpoint', '🌐 Endpoint: {url}').replace('{url}', upload_url))
        self.add_log(f"📝 Metadata: map_name='{map_name}', vehicle_id='{vehicle_id}'")
        
        # Tính toán timeout động: ít nhất 60s, thêm 30s cho mỗi 10MB
        base_timeout = 60
        timeout_per_10mb = 30
        dynamic_timeout = base_timeout + int((file_size_mb / 10) * timeout_per_10mb)
        # Giới hạn timeout tối đa 600s (10 phút) để tránh chờ quá lâu
        upload_timeout = min(dynamic_timeout, 600)
        self.add_log(self.translator.get('log.upload_timeout', 
            '⏱️ Upload timeout: {timeout}s').replace('{timeout}', str(upload_timeout)))
        
        # Retry configuration
        max_retries = 3
        retry_delays = [2, 5, 10]  # Exponential backoff: 2s, 5s, 10s
        
        # Tạo session để reuse connection và cải thiện performance
        session = None
        try:
            session = requests.Session()
            # Tăng connection pool size
            adapter = requests.adapters.HTTPAdapter(
                pool_connections=1,
                pool_maxsize=1,
                max_retries=0  # Tự quản lý retry
            )
            session.mount('http://', adapter)
            session.mount('https://', adapter)
            
            # Retry loop
            last_error = None
            for attempt in range(max_retries):
                if attempt > 0:
                    delay = retry_delays[min(attempt - 1, len(retry_delays) - 1)]
                    self.add_log(self.translator.get('log.upload_retry', 
                        '🔄 Retrying upload (attempt {attempt}/{max}) after {delay}s...').replace('{attempt}', str(attempt + 1)).replace('{max}', str(max_retries)).replace('{delay}', str(delay)))
                    time.sleep(delay)
                
                try:
                    # Sử dụng streaming upload để tránh load toàn bộ file vào memory
                    # Điều này giúp tránh crash với file lớn
                    self.add_log(self.translator.get('log.upload_starting', 
                        '📤 Starting upload (attempt {attempt}/{max})...').replace('{attempt}', str(attempt + 1)).replace('{max}', str(max_retries)))
                    
                    start_time = time.time()
                    
                    # Mở file với context manager để đảm bảo đóng đúng cách
                    with open(zip_path, "rb") as f:
                        files = {"file": (zip_path.name, f, "application/zip")}
                        
                        # Upload với timeout và streaming
                        response = session.post(
                            upload_url,
                            data={
                                "map_name": map_name,
                                "vehicle_id": vehicle_id
                            },
                            files=files,
                            timeout=(10, upload_timeout),  # (connect timeout, read timeout)
                            stream=False  # False để đảm bảo upload hoàn tất
                        )
                    
                    upload_duration = time.time() - start_time
                    upload_speed_mbps = (file_size_mb / upload_duration) if upload_duration > 0 else 0
                    
                    self.add_log(self.translator.get('log.upload_completed', 
                        '✅ Upload completed in {duration:.1f}s ({speed:.2f} MB/s)').replace('{duration:.1f}', f'{upload_duration:.1f}').replace('{speed:.2f}', f'{upload_speed_mbps:.2f}'))
                    
                    # Validate response
                    if response.status_code in (200, 201):
                        upload_id = None
                        try:
                            data = response.json()
                            upload_id = data.get("upload_id") or data.get("uploadId") or data.get("id")
                            
                            if upload_id:
                                self.add_log(self.translator.get('log.upload_success_with_id', 
                                    '✅ Upload successful! Upload ID: {id}').replace('{id}', str(upload_id)))
                            else:
                                self.add_log(self.translator.get('log.upload_success_no_id', 
                                    '✅ Upload successful! (No upload ID in response)'))
                        except (ValueError, KeyError) as e:
                            # Response không phải JSON hoặc không có upload_id
                            self.add_log(self.translator.get('log.upload_success_invalid_response', 
                                '⚠️ Upload successful but response format unexpected: {error}').replace('{error}', str(e)))
                            upload_id = None
                        
                        return True, upload_id
                    
                    # Non-200 response - không retry cho client errors (4xx)
                    if 400 <= response.status_code < 500:
                        error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                        error_msg = self.translator.get('log.upload_client_error', 
                            '❌ Client error (HTTP {code}): {error}').replace('{code}', str(response.status_code)).replace('{error}', error_text)
                        self.add_log(error_msg)
                        return False, error_text
                    
                    # Server errors (5xx) - có thể retry
                    if 500 <= response.status_code < 600:
                        error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                        last_error = f"Server error (HTTP {response.status_code}): {error_text}"
                        self.add_log(f"⚠️ {last_error}")
                        # Tiếp tục retry loop
                        continue
                    
                    # Unknown status code
                    error_text = response.text.strip()[:500] if response.text else f"HTTP {response.status_code}"
                    last_error = f"Unexpected status code {response.status_code}: {error_text}"
                    self.add_log(f"⚠️ {last_error}")
                    # Tiếp tục retry loop
                    continue
                
                except requests.exceptions.Timeout as e:
                    last_error = self.translator.get('log.upload_timeout_error', 
                        'Upload timeout after {timeout}s').replace('{timeout}', str(upload_timeout))
                    self.add_log(f"⚠️ {last_error}")
                    # Retry nếu chưa hết số lần thử
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except requests.exceptions.ConnectionError as e:
                    last_error = self.translator.get('log.upload_connection_error', 
                        'Connection error: {error}').replace('{error}', str(e))
                    self.add_log(f"⚠️ {last_error}")
                    # Retry nếu chưa hết số lần thử
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except requests.exceptions.RequestException as e:
                    last_error = self.translator.get('log.upload_request_error', 
                        'Request error: {error}').replace('{error}', str(e))
                    self.add_log(f"⚠️ {last_error}")
                    # Retry nếu chưa hết số lần thử
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, last_error
                
                except IOError as e:
                    # File I/O error - không retry
                    error_msg = self.translator.get('log.upload_io_error', 
                        'File I/O error: {error}').replace('{error}', str(e))
                    self.add_log(f"❌ {error_msg}")
                    return False, error_msg
                
                except Exception as e:
                    # Unexpected error
                    error_msg = self.translator.get('log.upload_unexpected_error', 
                        'Unexpected error: {error}').replace('{error}', str(e))
                    self.add_log(f"❌ {error_msg}")
                    import traceback
                    self.add_log(f"   Details: {traceback.format_exc()[:500]}")
                    # Retry nếu chưa hết số lần thử
                    if attempt < max_retries - 1:
                        continue
                    else:
                        return False, error_msg
            
            # Tất cả retry đã thất bại
            final_error = self.translator.get('log.upload_all_retries_failed', 
                'All retry attempts failed. Last error: {error}').replace('{error}', str(last_error))
            self.add_log(f"❌ {final_error}")
            return False, last_error or "Unknown error"
        
        finally:
            # Đảm bảo đóng session
            if session:
                try:
                    session.close()
                except:
                    pass
    
    def stop_process(self):
        # Đánh dấu đang dừng từ STOP button
        self.is_stopping = True
        self.cleanup_processes()
        self.btn_start.config(state=tk.NORMAL)
        self.set_progress(0)
        self.add_log(self.translator.get('log.process_terminated', 'PROCESS: Mapping and Upload terminated.'))
        # Flag sẽ được reset trong cleanup_processes()

if __name__ == "__main__":
    app = None
    try:
        # Khởi tạo root window trước
        root = tk.Tk()
        
        # Thử khởi tạo ứng dụng với error handling tốt hơn
        try:
            app = BagMappingInterface(root)
        except Exception as e:
            print(f"Error initializing BagMappingInterface: {e}")
            import traceback
            traceback.print_exc()
            
            # Hiển thị error dialog nếu có thể
            try:
                import tkinter.messagebox as mb
                mb.showerror(
                    "Initialization Error",
                    f"Failed to initialize application:\n\n{str(e)}\n\n"
                    "Some features may be disabled. Check console for details."
                )
            except:
                pass
            
            # Vẫn chạy mainloop để không crash hoàn toàn
            # Nhưng ứng dụng có thể không hoạt động đầy đủ
            root.destroy()
            sys.exit(1)
        
        # Cleanup function khi đóng window
        def on_closing():
            if app:
                app.log_update_thread_running = False
            root.destroy()
        
        root.protocol("WM_DELETE_WINDOW", on_closing)
        
        # Chạy main loop với error handling
        try:
            root.mainloop()
        except KeyboardInterrupt:
            print("\nApplication interrupted by user")
        except Exception as e:
            print(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Cleanup threads
            if app:
                app.log_update_thread_running = False
    
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)