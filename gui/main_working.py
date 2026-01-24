import tkinter as tk
from tkinter import scrolledtext, messagebox
from datetime import datetime
import uuid
import subprocess
import os
import sys
import requests
import threading
import time
import zipfile
import shutil
import json
import signal
from pathlib import Path
from datetime import timezone

# Thêm project root vào sys.path để có thể import các module
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from languages.translate_engine import Translator
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS, BACKEND_HOST

# Theta driver import (optional)
try:
    from gui.theta_logic import ThetaDriver
    THETA_DRIVER_AVAILABLE = True
except ImportError:
    THETA_DRIVER_AVAILABLE = False
    print("Warning: ThetaDriver not available. Theta camera features will be disabled.")

# Heartbeat configuration
HEARTBEAT_INTERVAL_SECONDS = 10

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image
    try:
        from cv_bridge import CvBridge
        CV_BRIDGE_AVAILABLE = True
    except ImportError:
        CV_BRIDGE_AVAILABLE = False
        print("Warning: cv_bridge not available. QR scanning from ROS topic will be disabled.")
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    CV_BRIDGE_AVAILABLE = False
    print("Warning: ROS2 not available. Pose publishing will be disabled.")

# QR Scanner imports (optional)
try:
    import cv2
    import numpy as np
    try:
        from pyzbar import pyzbar
        PYZBAR_AVAILABLE = True
    except ImportError:
        PYZBAR_AVAILABLE = False
        print("Warning: pyzbar not available. QR scanning will be disabled.")
    QR_SCANNER_AVAILABLE = True
except ImportError:
    QR_SCANNER_AVAILABLE = False
    PYZBAR_AVAILABLE = False
    print("Warning: cv2 or numpy not available. QR scanning will be disabled.")


class QRImageSubscriberNode(Node):
    """ROS2 Node để subscribe image topic từ theta driver cho QR scanning"""
    
    def __init__(self, image_topic, image_callback, node_name='qr_image_subscriber'):
        super().__init__(node_name)
        
        # Đảm bảo topic name có / prefix
        if not image_topic.startswith('/'):
            image_topic = '/' + image_topic
        
        # Subscribe image topic
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self.image_callback_func = image_callback
        self.image_topic_name = image_topic
        self.get_logger().info(f'QR Scanner subscribed to image topic {image_topic}')
    
    def image_callback(self, msg):
        """Callback when receiving image"""
        try:
            if cv2 is None or self.bridge is None:
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
            self.get_logger().error(error_msg)


class PoseSubscriber(Node):
    """ROS2 Node để subscribe pose và gửi lên backend"""
    
    def __init__(self, callback_func, node_name='pose_publisher_subscriber'):
        super().__init__(node_name)
        self.callback_func = callback_func
        
        # Subscribe vào topic /localization (Odometry từ transform_fusion)
        # Fallback về /Odometry nếu không có
        self.subscription = self.create_subscription(
            Odometry,
            '/localization',
            self.pose_callback,
            10
        )
        
        # Thử subscribe thêm /Odometry làm backup
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/Odometry',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Pose subscriber đã subscribe vào /localization và /Odometry')
    
    def pose_callback(self, msg):
        """Callback khi nhận được pose mới"""
        try:
            # Extract position và orientation
            position = {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z)
            }
            
            orientation = {
                "x": float(msg.pose.pose.orientation.x),
                "y": float(msg.pose.pose.orientation.y),
                "z": float(msg.pose.pose.orientation.z),
                "w": float(msg.pose.pose.orientation.w)
            }
            
            # Convert timestamp từ ROS2 time sang datetime object (UTC)
            ros_time = msg.header.stamp
            # ROS2 time có sec và nanosec
            timestamp_sec = ros_time.sec + ros_time.nanosec / 1e9
            timestamp_dt = datetime.fromtimestamp(timestamp_sec, tz=timezone.utc)
            # Backend expect ISO format string
            timestamp_iso = timestamp_dt.isoformat()
            
            # Gọi callback để gửi lên backend
            if self.callback_func:
                self.callback_func(position, orientation, timestamp_iso)
                
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý pose: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class WorkerInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Worker Interface | System Monitoring")
        self.root.geometry("1100x700")
        
        self.is_running = False
        self.auto_start_triggered = False  # Flag để tránh double-start
        
        # Paths
        self.workspace_path = Path(project_root) / "ws"
        self.log_path = self.workspace_path / "src" / "FAST-LIVO2" / "Log"
        self.default_map_root = self.log_path / "fastloc_map"
        self.qr_detect_path = self.log_path / "QR_detect.json"
        self.backend_base_url = BACKEND_HOST.rstrip("/")
        self.autostart_config_path = Path(project_root) / "autostart_config.json"
        self.install_autostart_script = Path(project_root) / "install_autostart.sh"
        
        # Localization process
        self.loc_process = None
        self.is_localization_running = False
        
        # Livox driver process
        self.livox_driver_process = None
        self.is_livox_driver_running = False
        
        # Livox device connection info
        self.livox_ip = self._get_livox_ip_from_config()  # IP từ config hoặc mặc định
        self.is_livox_device_connected = False  # Trạng thái kết nối thiết bị (ping)
        self.device_connection_timer_id = None
        self.device_connection_update_interval = 5000  # Cập nhật mỗi 5 giây
        
        # Pose publishing
        self.pose_subscriber = None
        self.pose_executor = None
        self.pose_thread = None
        self.vehicle_id = None
        self.is_pose_publishing = False
        self.pose_counter = 0  # Đếm số lần nhận pose
        self.pose_send_interval = 20  # Gửi lên backend mỗi 20 lần
        self.last_pose_received_time = None  # Thời gian nhận pose cuối cùng
        self.pose_first_received_logged = False  # Đã log lần đầu nhận pose chưa
        
        # Heartbeat
        self.heartbeat_timer_id = None
        self.heartbeat_logged = False  # Đã log heartbeat thành công chưa
        
        # QR Scanner (sử dụng ROS topic từ theta driver)
        self.is_qr_scanning = False
        self.qr_ros_node = None  # ROS node để subscribe image topic
        self.qr_ros_executor = None  # ROS executor cho QR scanning
        self.qr_ros_thread = None  # Thread để chạy ROS executor
        self.qr_detect_data = {}  # Dictionary chứa QR codes và tọa độ từ JSON
        self.last_detected_qr = None  # QR code cuối cùng được phát hiện
        self.qr_scan_frame_interval = 5  # Quét QR mỗi 5 frame để tối ưu performance
        self.qr_frame_count = 0
        self.qr_lock = threading.Lock()  # Lock để thread-safe khi truy cập shared data
        
        # Translator for multi-language support (cần khởi tạo trước khi load QR data)
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        # Theta driver (for QR scanning camera)
        self.theta_driver = None
        self.is_theta_connected = False
        if THETA_DRIVER_AVAILABLE:
            try:
                self.theta_driver = ThetaDriver(
                    log_callback=self.log,
                    update_ui_theta_connected=self.update_ui_theta_connected
                )
            except Exception as e:
                # Không thể log vì log_panel chưa được setup, dùng print
                print(f"⚠️ Error initializing ThetaDriver: {e}")
                self.theta_driver = None
        
        # Set title with translator
        self.root.title(self.translator.get('title.worker_interface', 'Worker Interface | System Monitoring'))
        
        # Setup language button FIRST (before main container)
        self.setup_language_button()

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        # Pack AFTER language button to ensure language button is on top
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- PHẦN 1: SIDEBAR (Bên trái) ---
        self.sidebar = tk.Frame(self.main_container, width=320, relief=tk.SUNKEN, bd=1)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sidebar.pack_propagate(False) # Giữ cố định độ rộng cho Sidebar

        # --- PHẦN 2: SYSTEM LOG (Bên phải - Chiếm trọn phần còn lại) ---
        self.log_container = tk.Frame(self.main_container, padx=5, pady=5)
        self.log_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Cài đặt các thành phần bên trong Sidebar
        self.setup_movement_control()
        self.setup_options()
        self.setup_vehicle_info()
        self.setup_map_info()
        self.setup_device_connection_info()
        self.setup_pose_display()

        # Cài đặt phần Log bên phải
        self.setup_log_area()
        
        # Load QR detect data (sau khi log_panel đã được setup)
        self.load_qr_detect_data()
        
        # Update UI texts after all components are set up
        self.update_ui_texts()
        
        # Kiểm tra và tự động đăng ký thiết bị nếu chưa có trên backend
        # Gọi trong thread để không block UI
        threading.Thread(target=self.check_and_auto_register, daemon=True).start()
        
        # Cập nhật trạng thái vehicle thành online khi khởi động
        # Gọi sau khi setup_vehicle_info để có vehicle_id
        if self.vehicle_id:
            self.update_vehicle_status("online", refresh_info=True)
            # Bắt đầu heartbeat tự động
            self.start_heartbeat()
        
        # Tự động tải map và QR khi khởi động
        self.download_map_and_qr()
        
        # Kiểm tra nếu map đã tồn tại sẵn (không cần download)
        if self.default_map_root.exists() and (self.default_map_root / "pose.json").exists():
            # Map đã có sẵn, enable nút và tự động start
            self.root.after(500, self._on_map_downloaded)  # Delay 0.5 giây để đảm bảo UI đã render xong
        
        # Bắt đầu monitoring kết nối thiết bị định kỳ
        self.start_device_connection_monitoring()
        
        # Tự động check Theta USB connection khi khởi động
        if self.theta_driver:
            self.check_theta_usb_on_startup()
        
        # Thêm handler khi đóng cửa sổ
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Kiểm tra và hiển thị thông báo autostart nếu cần (sau khi UI đã load, delay 5 giây để không gây khó chịu)
        self.root.after(5000, self.check_and_prompt_autostart)
    
    def setup_language_button(self):
        """Setup language toggle button"""
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
        """Toggle between English and Japanese"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """Change language and update UI"""
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        """Update all UI texts based on current language"""
        self.root.title(self.translator.get('title.worker_interface', 'Worker Interface | System Monitoring'))
        
        # Update movement control frame
        if hasattr(self, 'btn_start'):
            self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'))
        if hasattr(self, 'status_text'):
            current_text = self.status_text.cget('text')
            if 'Đang tải map' in current_text or 'Loading map' in current_text:
                self.status_text.config(text=self.translator.get('label.loading_map', 'System: Loading map...'))
            elif 'Đã dừng' in current_text or 'Stopped' in current_text:
                self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
            elif 'Localization đang chạy' in current_text or 'Localization running' in current_text:
                self.status_text.config(text=self.translator.get('label.localization_running', 'System: Localization running...'))
            elif 'Ready to start' in current_text or 'Sẵn sàng' in current_text:
                self.status_text.config(text=self.translator.get('label.system_ready', 'System: Ready to start'))
        
        # Update preview frame
        if hasattr(self, 'preview_frame'):
            self.preview_frame.config(text=self.translator.get('label.preview', 'Preview'))
        if hasattr(self, 'btn_rviz'):
            self.btn_rviz.config(text="📊 " + self.translator.get("button.launch_rviz2", "Launch RViz2"))
        
        # Update settings frame
        if hasattr(self, 'settings_frame'):
            self.settings_frame.config(text=self.translator.get('label.settings', 'Settings'))
        if hasattr(self, 'chk_autostart'):
            self.chk_autostart.config(text=self.translator.get('label.autostart', 'Auto-start on boot'))
        
        # Update log area
        if hasattr(self, 'log_title_label'):
            self.log_title_label.config(text=self.translator.get('label.system_monitoring_logs', 'SYSTEM MONITORING LOGS'))
        if hasattr(self, 'btn_clear_log'):
            self.btn_clear_log.config(text=self.translator.get('button.clear_log', 'Clear Log'))
        
        # Update vehicle info labels
        if hasattr(self, 'vehicle_info_label_widgets'):
            for key, label_widget in self.vehicle_info_label_widgets.items():
                if key == 'device_id':
                    label_widget.config(text=self.translator.get('label.device_id', 'Device Id') + ":")
                elif key == 'vehicle_id':
                    label_widget.config(text=self.translator.get('label.vehicle_id', 'Vehicle Id') + ":")
                elif key == 'name':
                    label_widget.config(text=self.translator.get('label.name', 'Name') + ":")
                elif key == 'type':
                    label_widget.config(text=self.translator.get('label.type', 'Type') + ":")
                elif key == 'status':
                    label_widget.config(text=self.translator.get('label.status', 'Status') + ":")
        
        # Refresh vehicle info to update translated values (not given, N/A)
        if hasattr(self, 'vehicle_info_labels'):
            self.refresh_vehicle_info()
        
        # Update map info (will be updated by update_map_info)
        if hasattr(self, 'map_info_label'):
            self.update_map_info()
        
        # Update pose display labels
        if hasattr(self, 'pos_label'):
            self.pos_label.config(text=self.translator.get('label.position', 'Position:'))
        if hasattr(self, 'orient_label'):
            self.orient_label.config(text=self.translator.get('label.orientation', 'Orientation:'))
        if hasattr(self, 'pose_status_label'):
            current_text = self.pose_status_label.cget('text')
            if 'Chưa có dữ liệu' in current_text or 'No data' in current_text:
                self.pose_status_label.config(text=self.translator.get('label.no_data', 'No data'))
        
        # Update frame titles
        if hasattr(self, 'movement_control_frame'):
            self.movement_control_frame.config(text=self.translator.get('label.movement_control', 'Movement Control'))
        if hasattr(self, 'preview_frame'):
            self.preview_frame.config(text=self.translator.get('label.preview', 'Preview'))
        if hasattr(self, 'settings_frame'):
            self.settings_frame.config(text=self.translator.get('label.settings', 'Settings'))
        if hasattr(self, 'chk_autostart'):
            self.chk_autostart.config(text=self.translator.get('label.autostart', 'Auto-start on boot'))
        if hasattr(self, 'vehicle_info_frame'):
            self.vehicle_info_frame.config(text=self.translator.get('label.vehicle_information', 'Vehicle Information'))
        if hasattr(self, 'map_info_frame'):
            self.map_info_frame.config(text=self.translator.get('label.map_information', 'Map Information'))
        if hasattr(self, 'device_connection_frame'):
            self.device_connection_frame.config(text=self.translator.get('label.device_connection', 'Device Connection'))
            # Cập nhật lại IP và trạng thái
            if hasattr(self, 'device_ip_label'):
                self.device_ip_label.config(text=f"IP: {self.livox_ip}")
            self.check_device_connection()
        if hasattr(self, 'pose_display_frame'):
            self.pose_display_frame.config(text=self.translator.get('label.pose_orientation', 'Pose & Orientation'))
        
        # Update QR scanning UI
        if hasattr(self, 'qr_scanning_status_label'):
            self.update_qr_scanning_ui()

    def setup_movement_control(self):
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.movement_control', 'Movement Control'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.movement_control_frame = frame  # Store reference for update_ui_texts

        self.btn_start = tk.Button(frame, text=self.translator.get('button.start_moving', '▶ START MOVING'), command=self.toggle_movement, state=tk.DISABLED)
        self.btn_start.pack(fill=tk.X, pady=2)

        self.status_text = tk.Label(frame, text=self.translator.get('label.loading_map', 'System: Loading map...'), font=("Arial", 9, "italic"))
        self.status_text.pack(pady=5)

    def setup_options(self):
        # Frame Preview
        preview_frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.preview', 'Preview'), padx=5, pady=5)
        preview_frame.pack(fill=tk.X, pady=5, padx=5)
        self.preview_frame = preview_frame  # Store reference for update_ui_texts

        self.btn_rviz = tk.Button(
            preview_frame, 
            text="📊 " + self.translator.get("button.launch_rviz2", "Launch RViz2"), 
            command=self.open_rviz,
            width=10
        )
        self.btn_rviz.pack(side="right", pady=10)
        
        # Frame Settings
        settings_frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.settings', 'Settings'), padx=5, pady=5)
        settings_frame.pack(fill=tk.X, pady=5, padx=5)
        self.settings_frame = settings_frame  # Store reference for update_ui_texts
        
        # Tạo checkbox cho autostart với giá trị mặc định False, sẽ update sau khi check xong
        self.autostart_var = tk.BooleanVar(value=False)
        self.chk_autostart = tk.Checkbutton(
            settings_frame,
            text=self.translator.get('label.autostart', 'Auto-start on boot'),
            variable=self.autostart_var,
            command=self.toggle_autostart
        )
        self.chk_autostart.pack(anchor="w", pady=5)
        
        # Check autostart status trong background thread để không block UI
        def check_status():
            try:
                enabled = self.check_autostart_enabled()
                # Update UI trong main thread
                self.root.after(0, lambda: self.autostart_var.set(enabled))
            except Exception as e:
                self.root.after(0, lambda: self.log(f"⚠️ Error checking autostart status: {e}"))
        
        threading.Thread(target=check_status, daemon=True).start()  
    
    def toggle_autostart(self):
        """Xử lý khi user bật/tắt autostart từ checkbox"""
        # Disable checkbox khi đang xử lý để tránh double-click
        self.chk_autostart.config(state=tk.DISABLED)
        current_state = self.autostart_var.get()
        
        if current_state:
            # User muốn bật autostart
            self.chk_autostart.config(text=self.translator.get('log.autostart_enabling', 'Enabling autostart...'))
            self.log(self.translator.get('log.autostart_enabling', '📋 Enabling autostart...'))
            self.install_autostart()
        else:
            # User muốn tắt autostart
            self.chk_autostart.config(text=self.translator.get('log.autostart_disabling', 'Disabling autostart...'))
            self.log(self.translator.get('log.autostart_disabling', '📋 Disabling autostart...'))
            self.disable_autostart()
    
    def open_rviz(self):
        """Mở RViz2 để xem bản đồ (không khởi động node)"""
        # Kiểm tra map có tồn tại không
        if not self.default_map_root.exists() or not (self.default_map_root / "pose.json").exists():
            self.log(self.translator.get('log.map_not_ready_wait', '⚠️ Map not ready. Please wait for map download to complete.'))
            return
        
        # Tìm rviz config file từ fast_lio_localization package
        rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz" / "fastlio_localization.rviz"
        
        # Nếu không tìm thấy, thử tìm trong rviz_cfg
        if not rviz_config_file.exists():
            rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz_cfg" / "localization.rviz"
        
        self.log(self.translator.get('log.opening_rviz2', '🚀 Opening RViz2...'))
        
        # Kiểm tra xem file có tồn tại không để tránh lỗi im lặng
        if not rviz_config_file.exists():
            self.log(self.translator.get('log.rviz_config_not_found', '⚠️ Warning: Config file not found at {path}. Will open RViz with default.').replace('{path}', str(rviz_config_file)))
            cmd = ['rviz2']
        else:
            cmd = ['rviz2', '-d', str(rviz_config_file)]
            self.log(self.translator.get('log.using_config', '📋 Using config: {name}').replace('{name}', rviz_config_file.name))

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Ẩn log của RViz để đỡ rối terminal
                stderr=subprocess.STDOUT
            )
            self.log(self.translator.get('log.rviz2_opened', '✅ RViz2 opened. Note: Need to start node (START MOVING) to display map.'))
            
        except Exception as e:
            self.log(self.translator.get('log.error_executing_rviz2', '❌ Error executing rviz2: {error}').replace('{error}', str(e)))
    
    def monitor_localization_process(self):
        """Theo dõi output của process localization"""
        proc = self.loc_process
        if not proc:
            return
            
        try:
            # Chỉ đợi process kết thúc, không log output
            proc.wait()
        except Exception as e:
            self.log(self.translator.get('log.error_monitor_localization', 'Error in monitor_localization_process: {error}').replace('{error}', str(e)))
        finally:
            self.root.after(0, self.on_localization_ended)
    
    def on_localization_ended(self):
        """Xử lý khi process localization kết thúc"""
        if self.is_localization_running:
            self.log(self.translator.get('log.localization_process_stopped', '⏹ Localization process stopped'))
            self.root.after(0, self._handle_localization_stopped)
    
    def _handle_localization_stopped(self):
        """Xử lý UI khi localization dừng"""
        self.is_localization_running = False
        self.is_running = False
        self.auto_start_triggered = False  # Reset flag khi stop
        self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'), state=tk.NORMAL)
        self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
    
    def stop_localization(self):
        """Dừng localization process và kill tất cả các node liên quan"""
        # Kill process group trước
        if self.loc_process:
            try:
                if hasattr(os, 'setsid'):
                    os.killpg(os.getpgid(self.loc_process.pid), signal.SIGTERM)
                else:
                    self.loc_process.terminate()
                self.loc_process.wait(timeout=5)
            except:
                if self.loc_process:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(self.loc_process.pid), signal.SIGKILL)
                    else:
                        self.loc_process.kill()
            finally:
                self.loc_process = None
        
        # Kill tất cả các ROS2 node liên quan đến localization
        self.log(self.translator.get('log.stopping_all_localization_nodes', '🛑 Stopping all localization nodes...'))
        try:
            # Kill node fastlio_mapping
            subprocess.run(
                ['pkill', '-f', 'fastlio_mapping'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill node rviz2 nếu đang chạy với config localization
            subprocess.run(
                ['pkill', '-f', 'fastlio_localization.rviz'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill ros2 launch process
            subprocess.run(
                ['pkill', '-f', 'localization.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill bằng ros2 node kill (nếu có ROS2 environment)
            try:
                # Source ROS2 và kill node
                ros2_setup = "/opt/ros/jazzy/setup.bash"
                ws_setup = str(self.workspace_path / "install" / "setup.sh")
                
                if os.path.exists(ros2_setup) and os.path.exists(ws_setup):
                    kill_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 node list | grep -E 'fastlio|localization' | xargs -r ros2 node kill"
                    subprocess.run(
                        kill_cmd,
                        shell=True,
                        executable="/bin/bash",
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=5
                    )
            except:
                pass  # Ignore nếu không có ROS2 environment
            
            self.log(self.translator.get('log.all_localization_nodes_stopped', '✅ All localization nodes stopped'))
            
        except Exception as e:
            self.log(self.translator.get('log.error_killing_nodes', '⚠️ Error killing nodes: {error}').replace('{error}', str(e)))
        
        self.is_localization_running = False
    
    def start_pose_publishing(self):
        """Khởi động ROS2 subscriber để publish pose lên backend"""
        if not ROS2_AVAILABLE:
            self.log(self.translator.get('log.ros2_not_available_pose', '⚠️ ROS2 not available, cannot publish pose'))
            return
        
        if self.is_pose_publishing:
            return
        
        if not self.vehicle_id:
            self.log(self.translator.get('log.vehicle_id_not_available', '⚠️ Vehicle ID not available, cannot publish pose'))
            return
        
        try:
            # Kiểm tra topics có tồn tại không trước khi subscribe
            self.log(self.translator.get('log.checking_pose_topics', '🔍 Checking pose topics...'))
            time.sleep(2)  # Đợi một chút để localization node khởi động
            
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            topics_found = []
            if result.returncode == 0:
                topics = result.stdout
                if '/localization' in topics or 'localization' in topics:
                    topics_found.append('/localization')
                    self.log(self.translator.get('log.topic_exists', '✓ Topic {topic} exists').replace('{topic}', '/localization'))
                if '/Odometry' in topics or 'Odometry' in topics:
                    topics_found.append('/Odometry')
                    self.log(self.translator.get('log.topic_exists', '✓ Topic {topic} exists').replace('{topic}', '/Odometry'))
                
                if not topics_found:
                    self.log(self.translator.get('log.topic_not_found_warning', '⚠️ Warning: Topics /localization or /Odometry not found'))
                    self.log(self.translator.get('log.localization_may_not_ready', '   Localization may not be ready, will try to subscribe and wait...'))
            
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                rclpy.init()
            
            # Tạo subscriber node
            self.pose_subscriber = PoseSubscriber(
                callback_func=self.pose_callback_handler,
                node_name='pose_publisher_node'
            )
            
            # Tạo executor
            self.pose_executor = SingleThreadedExecutor()
            self.pose_executor.add_node(self.pose_subscriber)
            
            # Chạy executor trong background thread
            self.is_pose_publishing = True
            self.pose_thread = threading.Thread(
                target=self._run_pose_executor,
                daemon=True
            )
            self.pose_thread.start()
            
            self.log(self.translator.get('log.pose_publisher_started', '✅ Pose publisher started'))
            self.log(self.translator.get('log.waiting_for_pose', '📡 Waiting for pose from /localization or /Odometry...'))
            
            # Bắt đầu timer để kiểm tra xem có nhận được pose không
            self.last_pose_received_time = None
            self.pose_first_received_logged = False
            self._start_pose_monitoring()
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_pose_publisher', '❌ Error starting pose publisher: {error}').replace('{error}', str(e)))
            self.is_pose_publishing = False
    
    def _run_pose_executor(self):
        """Chạy ROS2 executor trong background thread"""
        try:
            while self.is_pose_publishing and rclpy.ok():
                self.pose_executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.log(self.translator.get('log.error_pose_executor', '❌ Error in pose executor: {error}').replace('{error}', str(e)))
        finally:
            if self.pose_subscriber:
                self.pose_subscriber.destroy_node()
            if not rclpy.ok():
                rclpy.shutdown()
    
    def stop_pose_publishing(self):
        """Dừng pose publishing"""
        if not self.is_pose_publishing:
            return
        
        self.is_pose_publishing = False
        
        if self.pose_executor:
            try:
                if self.pose_subscriber:
                    self.pose_executor.remove_node(self.pose_subscriber)
                    self.pose_subscriber.destroy_node()
            except:
                pass
        
        if self.pose_thread and self.pose_thread.is_alive():
            # Đợi thread kết thúc
            self.pose_thread.join(timeout=2)
        
        self.log(self.translator.get('log.pose_publisher_stopped', '🛑 Pose publisher stopped'))
    
    def pose_callback_handler(self, position, orientation, timestamp):
        """Xử lý pose callback và gửi lên backend"""
        if not self.vehicle_id:
            return
        
        # Cập nhật thời gian nhận pose cuối cùng
        self.last_pose_received_time = time.time()
        
        # Log lần đầu nhận được pose (chỉ 1 lần)
        if not self.pose_first_received_logged:
            self.log(self.translator.get('log.pose_received', '✅ Pose received'))
            self.pose_first_received_logged = True
        
        # Cập nhật UI (chạy trong main thread) - luôn cập nhật để hiển thị real-time
        self.root.after(0, lambda: self.update_pose_display(position, orientation, timestamp))
        
        # Đếm số lần nhận pose
        self.pose_counter += 1
        
        # Chỉ gửi lên backend mỗi 20 lần
        if self.pose_counter >= self.pose_send_interval:
            self.pose_counter = 0  # Reset counter
            # Gửi lên backend trong background thread để không block
            threading.Thread(
                target=self.send_pose_to_backend,
                args=(position, orientation, timestamp),
                daemon=True
            ).start()
    
    def _start_pose_monitoring(self):
        """Bắt đầu monitoring để kiểm tra xem có nhận được pose không"""
        def check_pose():
            if not self.is_pose_publishing:
                return
            
            if self.last_pose_received_time is None:
                # Chưa nhận được pose, kiểm tra lại sau 5 giây
                self.root.after(5000, check_pose)
                return
            
            # Đã nhận được pose, không cần check nữa
            elapsed = time.time() - self.last_pose_received_time
            if elapsed > 10:
                # Không nhận được pose trong 10 giây
                self.log(self.translator.get('log.pose_not_received', '❌ No pose received in 10 seconds'))
                # Check lại sau 10 giây nữa
                self.root.after(10000, check_pose)
            else:
                # Vẫn nhận được pose, check lại sau 10 giây
                self.root.after(10000, check_pose)
        
        # Bắt đầu check sau 5 giây
        self.root.after(5000, check_pose)
    
    def send_pose_to_backend(self, position, orientation, timestamp):
        """Gửi pose lên backend API"""
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/pose"
            
            payload = {
                "position": position,
                "orientation": orientation,
                "timestamp": timestamp
            }
            
            # API sử dụng POST method, không phải PUT
            response = requests.post(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            if response.status_code == 200:
                # Không log mỗi lần thành công để tránh spam
                pass
            elif response.status_code == 405:
                # Method not allowed - log chi tiết để debug
                if not hasattr(self, '_last_pose_error_code') or self._last_pose_error_code != response.status_code:
                    try:
                        error_detail = response.text[:200]
                    except:
                        error_detail = "Không thể đọc error detail"
                    self.log(self.translator.get('log.error_http_405', '⚠️ HTTP 405 error (Method Not Allowed)'))
                    self.log(f"   URL: {url}")
                    self.log(f"   Method: POST")
                    self.log(f"   Response: {error_detail}")
                    self._last_pose_error_code = response.status_code
            else:
                # Chỉ log lỗi một lần để tránh spam
                if not hasattr(self, '_last_pose_error_code') or self._last_pose_error_code != response.status_code:
                    try:
                        error_detail = response.text[:200]
                    except:
                        error_detail = ""
                    self.log(self.translator.get('log.error_sending_pose_http', '⚠️ Error sending pose: HTTP {code} - {detail}').replace('{code}', str(response.status_code)).replace('{detail}', error_detail))
                    self._last_pose_error_code = response.status_code
                
        except requests.exceptions.RequestException as e:
            # Chỉ log lỗi, không spam log
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(self.translator.get('log.error_connection_sending_pose', '⚠️ Connection error when sending pose: {error}').replace('{error}', str(e)))
                self._last_pose_error = datetime.now()
        except Exception as e:
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(self.translator.get('log.error_sending_pose', '⚠️ Error sending pose: {error}').replace('{error}', str(e)))
                self._last_pose_error = datetime.now()
    
    def check_livox_driver_running(self):
        """Kiểm tra xem Livox driver có đang chạy không bằng cách kiểm tra topics"""
        try:
            # Kiểm tra topics /livox/lidar và livox/imu có tồn tại không
            # Lưu ý: Driver publish livox/imu (không có / ở đầu), nhưng ROS2 có thể normalize
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                topics = result.stdout
                # Kiểm tra cả /livox/lidar và livox/lidar (ROS2 có thể normalize)
                has_lidar = '/livox/lidar' in topics or 'livox/lidar' in topics
                # Kiểm tra cả /livox/imu và livox/imu (driver publish livox/imu)
                has_imu = '/livox/imu' in topics or 'livox/imu' in topics
                
                if has_lidar and has_imu:
                    lidar_topic = '/livox/lidar' if '/livox/lidar' in topics else 'livox/lidar'
                    imu_topic = '/livox/imu' if '/livox/imu' in topics else 'livox/imu'
                    self.log(self.translator.get('log.topics_found', '✓ Found topics: lidar={lidar}, imu={imu}').replace('{lidar}', lidar_topic).replace('{imu}', imu_topic))
                    return True
                else:
                    # Log chi tiết để debug (chỉ khi không tìm thấy)
                    if not has_lidar:
                        self.log(self.translator.get('log.topic_not_found', '⚠️ Topic {topic} not found').replace('{topic}', '/livox/lidar'))
                    if not has_imu:
                        self.log(self.translator.get('log.topic_not_found', '⚠️ Topic {topic} not found').replace('{topic}', '/livox/imu'))
                    # Log tất cả topics có chứa 'livox' để debug
                    livox_topics = [t.strip() for t in topics.split('\n') if 'livox' in t.lower()]
                    if livox_topics:
                        self.log(self.translator.get('log.livox_topics_found', '   Topics containing \'livox\': {topics}').replace('{topics}', ', '.join(livox_topics)))
            
            return False
        except Exception as e:
            self.log(self.translator.get('log.error_checking_driver', '⚠️ Error checking driver: {error}').replace('{error}', str(e)))
            return False
    
    def start_livox_driver(self):
        """Khởi động Livox driver"""
        if self.is_livox_driver_running:
            return True
        
        try:
            # Paths
            drive_ws_path = Path(project_root) / "dependencies" / "drive_ws"
            drive_ws_setup = drive_ws_path / "install" / "setup.sh"
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Kiểm tra setup scripts
            if not os.path.exists(ros2_setup):
                self.log(self.translator.get('log.ros2_setup_not_found', '❌ ROS2 setup not found at: {path}').replace('{path}', ros2_setup))
                return False
            
            if not drive_ws_setup.exists():
                self.log(self.translator.get('log.drive_ws_not_built', '❌ Drive workspace not built: {path}').replace('{path}', str(drive_ws_setup)))
                self.log(self.translator.get('log.please_build_drive_ws', 'Please build drive_ws before running driver'))
                return False
            
            # Launch file path
            launch_file = "msg_MID360_launch.py"
            
            # Build command
            cmd = f"source {ros2_setup} && source {drive_ws_setup} && ros2 launch livox_ros_driver2 {launch_file}"
            
            self.log(self.translator.get('log.starting_livox_driver', '🚀 Starting Livox driver...'))
            self.log(f"Command: {cmd}")
            
            # Start process
            self.livox_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_livox_driver_running = True
            threading.Thread(target=self.monitor_livox_driver_output, daemon=True).start()
            
            # Đợi một chút để driver khởi động (giống như recorder)
            time.sleep(2.0)
            
            # Kiểm tra process có còn chạy không (quan trọng hơn là check topics)
            if self.livox_driver_process and self.livox_driver_process.poll() is None:
                # Process đang chạy, đợi thêm một chút để topics xuất hiện
                time.sleep(2.0)
                
                # Kiểm tra lại topics (nhưng không bắt buộc phải có)
                if self.check_livox_driver_running():
                    self.log(self.translator.get('log.livox_driver_started', '✅ Livox driver started successfully'))
                    return True
                else:
                    # Process đang chạy nhưng topics chưa xuất hiện - có thể thiết bị chưa kết nối
                    # Nhưng vẫn return True để tiếp tục với localization (có thể đang replay bag)
                    self.log(self.translator.get('log.livox_driver_running_no_topics', '⚠️ Driver process is running but topics not yet available. Continuing with localization...'))
                    return True
            else:
                # Process đã dừng ngay sau khi start - có thể không có thiết bị
                self.log(self.translator.get('log.livox_driver_exited_immediately', '⚠️ Driver process exited immediately. May be no device or bag replay.'))
                self.is_livox_driver_running = False
                self.livox_driver_process = None
                return False
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_livox_driver', '❌ Error starting Livox driver: {error}').replace('{error}', str(e)))
            self.is_livox_driver_running = False
            self.livox_driver_process = None
            return False
    
    def monitor_livox_driver_output(self):
        """Monitor output của Livox driver process"""
        proc = self.livox_driver_process
        if not proc:
            return
        
        try:
            for line in iter(proc.stdout.readline, ''):
                if not line:
                    break
                line = line.strip()
                if line:
                    # Chỉ log lỗi và cảnh báo để tránh spam
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed', 'cannot', 'unable']):
                        self.log(f"❌ Driver ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️ Driver WARNING: {line}")
        except Exception as e:
            self.log(self.translator.get('log.error_reading_driver_output', 'Error reading driver output: {error}').replace('{error}', str(e)))
        finally:
            # Khi process kết thúc
            if proc.poll() is not None:
                self.root.after(0, self._handle_driver_stopped)
    
    def _handle_driver_stopped(self):
        """Xử lý khi driver dừng"""
        if self.is_livox_driver_running:
            self.log(self.translator.get('log.livox_driver_stopped', '⏹ Livox driver stopped'))
            self.is_livox_driver_running = False
            self.livox_driver_process = None
    
    def stop_livox_driver(self):
        """Dừng Livox driver"""
        if not self.is_livox_driver_running:
            return
        
        if self.livox_driver_process:
            try:
                self.log(self.translator.get('log.stopping_livox_driver', '🛑 Stopping Livox driver...'))
                if hasattr(os, 'setsid'):
                    os.killpg(os.getpgid(self.livox_driver_process.pid), signal.SIGTERM)
                else:
                    self.livox_driver_process.terminate()
                self.livox_driver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if self.livox_driver_process:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(self.livox_driver_process.pid), signal.SIGKILL)
                    else:
                        self.livox_driver_process.kill()
            except Exception as e:
                self.log(self.translator.get('log.error_stopping_driver', '⚠️ Error stopping driver: {error}').replace('{error}', str(e)))
            finally:
                self.livox_driver_process = None
        
        # Kill tất cả các node liên quan đến livox driver
        try:
            subprocess.run(
                ['pkill', '-f', 'livox_ros_driver2'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
        except:
            pass
        
        self.is_livox_driver_running = False
        self.log(self.translator.get('log.livox_driver_stopped', '⏹ Livox driver đã dừng'))
    
    def setup_vehicle_info(self):
        device_id = self.get_mac_address()
        self.vehicle_id = device_id.lower().replace(':', '')[:12]
        vehicle_id = self.vehicle_id
        print("Vehicle ID:", vehicle_id)

        self.vehicle_info_frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.vehicle_information', 'Vehicle Information'), padx=10, pady=10)
        self.vehicle_info_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Lưu reference đến các label để có thể cập nhật lại
        self.vehicle_info_labels = {}
        
        # Tạo các label ban đầu
        info_keys = ["device_id", "vehicle_id", "name", "type", "status"]
        self.vehicle_info_label_widgets = {}  # Store label widgets for translation
        for key in info_keys:
            row = tk.Frame(self.vehicle_info_frame)
            row.pack(fill=tk.X, pady=2)
            # Use translator for label text
            label_key = f'label.vehicle_{key}' if key != 'name' and key != 'type' and key != 'status' else f'label.vehicle_{key}'
            if key == 'device_id':
                label_text = self.translator.get('label.device_id', 'Device Id') + ":"
            elif key == 'vehicle_id':
                label_text = self.translator.get('label.vehicle_id', 'Vehicle Id') + ":"
            elif key == 'name':
                label_text = self.translator.get('label.name', 'Name') + ":"
            elif key == 'type':
                label_text = self.translator.get('label.type', 'Type') + ":"
            elif key == 'status':
                label_text = self.translator.get('label.status', 'Status') + ":"
            else:
                label_text = key.replace("_", " ").title() + ":"
            label_widget = tk.Label(row, text=label_text, fg="grey", font=("Arial", 8))
            label_widget.pack(side=tk.LEFT)
            self.vehicle_info_label_widgets[key] = label_widget
            value_label = tk.Label(row, text="--", font=("Arial", 9, "bold"))
            value_label.pack(side=tk.RIGHT)
            self.vehicle_info_labels[key] = value_label
        
        # Load thông tin ban đầu
        self.refresh_vehicle_info()
    
    def refresh_vehicle_info(self):
        """Refresh thông tin vehicle từ API và cập nhật UI"""
        if not self.vehicle_id:
            return
        
        try:
            url = f'{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}'
            headers = {
                'accept': 'application/json'
            }
            
            response = requests.get(url, headers=headers, timeout=5)
            if response.status_code == 200:
                data = response.json()
            else:
                # Nếu không tìm thấy, sử dụng giá trị mặc định
                data = {}
        except Exception as e:
            self.log(self.translator.get('log.error_getting_vehicle_info', '⚠️ Error getting vehicle info: {error}').replace('{error}', str(e)))
            data = {}

        # Xử lý name: nếu null hoặc None thì hiển thị "not given"
        name = data.get("name")
        if name is None or name == "":
            name = self.translator.get('label.not_given', 'not given')
        
        # Xử lý type: nếu null hoặc None thì hiển thị "not given"
        vehicle_type = data.get("vehicle_type") or data.get("type")
        if vehicle_type is None or vehicle_type == "":
            vehicle_type = self.translator.get('label.not_given', 'not given')
        
        # Xử lý status: lấy từ API, nếu không có thì dùng "N/A"
        status = data.get("status") or self.translator.get('label.not_available', 'N/A')
        
        # Lấy device_id
        device_id = self.get_mac_address()

        # Cập nhật các label
        if "device_id" in self.vehicle_info_labels:
            self.vehicle_info_labels["device_id"].config(text=device_id)
        if "vehicle_id" in self.vehicle_info_labels:
            self.vehicle_info_labels["vehicle_id"].config(text=self.vehicle_id)
        if "name" in self.vehicle_info_labels:
            self.vehicle_info_labels["name"].config(text=name)
        if "type" in self.vehicle_info_labels:
            self.vehicle_info_labels["type"].config(text=vehicle_type)
        if "status" in self.vehicle_info_labels:
            self.vehicle_info_labels["status"].config(text=status)
    
    def setup_map_info(self):
        """Thiết lập phần hiển thị thông tin bản đồ"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.map_information', 'Map Information'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.map_info_frame = frame  # Store reference for update_ui_texts
        
        self.map_info_label = tk.Label(
            frame,
            text=self.translator.get('label.checking_map', 'Checking map...'),
            foreground="gray",
            font=("Arial", 8),
            wraplength=280,
            justify=tk.LEFT
        )
        self.map_info_label.pack(anchor=tk.W, pady=2)
        
        # Hiển thị thông tin chi tiết
        self.map_detail_label = tk.Label(
            frame,
            text="",
            foreground="gray",
            font=("Arial", 7),
            wraplength=280,
            justify=tk.LEFT
        )
        self.map_detail_label.pack(anchor=tk.W, pady=2)
        
        # Cập nhật thông tin map ban đầu
        self.update_map_info()
    
    def update_map_info(self):
        """Kiểm tra và hiển thị thông tin bản đồ từ fastloc_map"""
        map_path = self.default_map_root
        
        if not map_path.exists():
            self.map_info_label.config(text=self.translator.get('label.map_directory_not_exists', '❌ Map directory does not exist'), foreground="red")
            self.map_detail_label.config(text="")
            return

        pose_json = map_path / "pose.json"
        pcd_dir = map_path / "pcd"
        index_json = map_path / "index.json"
        
        issues = []
        if not pose_json.exists():
            issues.append(self.translator.get('label.missing_pose_json', 'missing pose.json'))
        if not pcd_dir.exists() or not any(pcd_dir.glob("*.pcd")):
            issues.append(self.translator.get('label.missing_pcd_files', 'missing pcd files'))
            
        if issues:
            self.map_info_label.config(text=self.translator.get('label.map_invalid', '⚠️ Map invalid: {issues}').replace('{issues}', ', '.join(issues)), foreground="orange")
            self.map_detail_label.config(text="")
        else:
            # Đếm số tile từ PCD files
            num_tiles = len(list(pcd_dir.glob("*.pcd")))
            
            # Đọc thông tin từ index.json nếu có
            detail_text = ""
            if index_json.exists():
                try:
                    with open(index_json, 'r', encoding='utf-8') as f:
                        index_data = json.load(f)
                        num_tiles_from_index = index_data.get("num_tiles", num_tiles)
                        if num_tiles_from_index != num_tiles:
                            num_tiles = num_tiles_from_index
                        
                        # Hiển thị thông tin chi tiết
                        tiles = index_data.get("tiles", [])
                        if tiles:
                            total_points = sum(tile.get("num_points", 0) for tile in tiles)
                            detail_text = self.translator.get('label.total_points_tiles', 'Total points: {points:,} | Tiles: {tiles}').replace('{points:,}', f'{total_points:,}').replace('{tiles}', str(num_tiles))
                except Exception as e:
                    detail_text = self.translator.get('label.tiles_count', 'Tiles: {count}').replace('{count}', str(num_tiles))
            else:
                detail_text = self.translator.get('label.tiles_count', 'Tiles: {count}').replace('{count}', str(num_tiles))
            
            self.map_info_label.config(text=self.translator.get('label.map_valid', '✅ Map valid: {count} tiles').replace('{count}', str(num_tiles)), foreground="green")
            self.map_detail_label.config(text=detail_text)
    
    def _get_livox_ip_from_config(self):
        """Đọc IP của Livox MID360 từ config file"""
        try:
            config_path = Path(project_root) / "dependencies" / "drive_ws" / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    # Thử lấy từ lidar_configs trước
                    if 'lidar_configs' in config and len(config['lidar_configs']) > 0:
                        ip = config['lidar_configs'][0].get('ip')
                        if ip:
                            return ip
                    # Nếu không có, thử lấy từ host_net_info
                    if 'MID360' in config and 'host_net_info' in config['MID360']:
                        if isinstance(config['MID360']['host_net_info'], list) and len(config['MID360']['host_net_info']) > 0:
                            lidar_ip = config['MID360']['host_net_info'][0].get('lidar_ip')
                            if lidar_ip and isinstance(lidar_ip, list) and len(lidar_ip) > 0:
                                return lidar_ip[0]
        except Exception as e:
            pass  # Ignore errors, dùng IP mặc định
        
        # IP mặc định
        return "192.168.1.109"
    
    def setup_device_connection_info(self):
        """Thiết lập phần hiển thị thông tin kết nối thiết bị Livox"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.device_connection', 'Device Connection'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.device_connection_frame = frame  # Store reference for update_ui_texts
        
        # Label hiển thị IP
        self.device_ip_label = tk.Label(
            frame,
            text=f"IP: {self.livox_ip}",
            foreground="gray",
            font=("Arial", 8),
            wraplength=280,
            justify=tk.LEFT
        )
        self.device_ip_label.pack(anchor=tk.W, pady=2)
        
        # Label hiển thị trạng thái kết nối
        self.device_status_label = tk.Label(
            frame,
            text=self.translator.get('label.checking_connection', 'Checking connection...'),
            foreground="gray",
            font=("Arial", 9, "bold"),
            wraplength=280,
            justify=tk.LEFT
        )
        self.device_status_label.pack(anchor=tk.W, pady=2)
        
        # Theta Camera status (nếu có ThetaDriver)
        if THETA_DRIVER_AVAILABLE:
            self.theta_camera_status_label = tk.Label(
                frame,
                text=self.translator.get('label.theta_camera', 'Theta Camera: Checking...'),
                foreground="gray",
                font=("Arial", 8),
                wraplength=280,
                justify=tk.LEFT
            )
            self.theta_camera_status_label.pack(anchor=tk.W, pady=2)
        
        # Separator
        separator = tk.Frame(frame, height=1, bg="gray")
        separator.pack(fill=tk.X, pady=5)
        
        # QR Scanning status
        self.qr_scanning_status_label = tk.Label(
            frame,
            text=self.translator.get('label.qr_scanning_status', 'QR Scanning: Inactive'),
            foreground="gray",
            font=("Arial", 8),
            wraplength=280,
            justify=tk.LEFT
        )
        self.qr_scanning_status_label.pack(anchor=tk.W, pady=2)
        
        # Last detected QR
        self.last_qr_label = tk.Label(
            frame,
            text=self.translator.get('label.last_qr', 'Last QR: None'),
            foreground="gray",
            font=("Arial", 7),
            wraplength=280,
            justify=tk.LEFT
        )
        self.last_qr_label.pack(anchor=tk.W, pady=2)
        
        # Cập nhật trạng thái ban đầu
        self.check_device_connection()
        self.update_qr_scanning_ui()
    
    def update_ui_theta_connected(self, is_connected):
        """Callback để cập nhật UI khi Theta camera connection thay đổi"""
        self.is_theta_connected = is_connected
        if hasattr(self, 'theta_camera_status_label'):
            if is_connected:
                self.theta_camera_status_label.config(
                    text="● " + self.translator.get('label.theta_camera', 'Theta Camera') + ": " + self.translator.get('status.connected', 'Connected'),
                    foreground="green"
                )
            else:
                self.theta_camera_status_label.config(
                    text="● " + self.translator.get('label.theta_camera', 'Theta Camera') + ": " + self.translator.get('status.disconnected', 'Disconnected'),
                    foreground="red"
                )
    
    def check_theta_usb_on_startup(self):
        """Tự động check Theta USB connection khi khởi động"""
        if not self.theta_driver:
            return
        
        def check():
            try:
                self.theta_driver.check_theta_usb_connection(self.check_theta_usb_callback)
            except Exception as e:
                self.log(f"⚠️ Error checking Theta USB: {e}")
        
        # Chạy trong background thread sau một chút để không block UI
        threading.Thread(target=check, daemon=True).start()
    
    def check_theta_usb_callback(self, is_connected):
        """Callback khi check Theta USB connection xong"""
        self.root.after(0, lambda: self.update_ui_theta_connected(is_connected))
        # Chỉ cập nhật UI, không tự động launch theta driver
        # Theta driver sẽ được launch khi start localization
    
    def _launch_theta_driver_if_needed(self):
        """Launch theta driver nếu chưa chạy và camera đã kết nối"""
        if not self.theta_driver:
            return False
        
        try:
            # Kiểm tra xem theta driver đã chạy chưa
            if hasattr(self.theta_driver, 'theta_driver_process') and \
               self.theta_driver.theta_driver_process and \
               self.theta_driver.theta_driver_process.poll() is None:
                # Đã chạy rồi
                self.log(self.translator.get('log.theta_driver_already_running', '✅ Theta driver is already running'))
                return True
            
            # Launch theta driver
            self.log(self.translator.get('log.launching_theta_driver', '🚀 Launching Theta driver...'))
            self.theta_driver.launch_theta_driver()
            
            # Đợi một chút để theta driver khởi động
            time.sleep(0.5)
            
            # Kiểm tra lại xem đã start thành công chưa
            if hasattr(self.theta_driver, 'theta_driver_process') and \
               self.theta_driver.theta_driver_process and \
               self.theta_driver.theta_driver_process.poll() is None:
                self.log(self.translator.get('log.theta_driver_started', '✅ Theta driver started successfully'))
                return True
            else:
                self.log(self.translator.get('log.theta_driver_start_failed', '⚠️ Theta driver failed to start'))
                return False
            
        except Exception as e:
            self.log(f"⚠️ Error launching Theta driver: {e}")
            return False
    
    def update_qr_scanning_ui(self):
        """Cập nhật UI hiển thị trạng thái QR scanning"""
        if not hasattr(self, 'qr_scanning_status_label') or not hasattr(self, 'last_qr_label'):
            return
        
        # Cập nhật trạng thái QR scanning
        if self.is_qr_scanning:
            self.qr_scanning_status_label.config(
                text="● " + self.translator.get('label.qr_scanning_active', 'QR Scanning: Active'),
                foreground="green"
            )
        else:
            self.qr_scanning_status_label.config(
                text="● " + self.translator.get('label.qr_scanning_inactive', 'QR Scanning: Inactive'),
                foreground="gray"
            )
        
        # Cập nhật QR code cuối cùng được phát hiện
        with self.qr_lock:
            last_qr = self.last_detected_qr
        
        if last_qr:
            self.last_qr_label.config(
                text=self.translator.get('label.last_qr_detected', 'Last QR: {qr}').replace('{qr}', last_qr),
                foreground="blue"
            )
        else:
            self.last_qr_label.config(
                text=self.translator.get('label.last_qr_none', 'Last QR: None'),
                foreground="gray"
            )
    
    def check_device_connection(self):
        """Kiểm tra kết nối thiết bị Livox bằng ping"""
        if not self.livox_ip:
            return
        
        def check():
            try:
                # Ping nhanh (1 gói tin, chờ 1 giây)
                # Dùng 'n' cho Windows, 'c' cho Linux
                param = '-n' if os.name == 'nt' else '-c'
                result = subprocess.run(
                    ['ping', param, '1', '-W', '1', self.livox_ip],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                is_connected = (result.returncode == 0)
                
                # Cập nhật UI trong main thread
                self.root.after(0, lambda: self._update_device_connection_ui(is_connected))
                
            except Exception as e:
                # Nếu có lỗi, coi như không kết nối
                self.root.after(0, lambda: self._update_device_connection_ui(False))
        
        # Chạy trong thread riêng để không block UI
        threading.Thread(target=check, daemon=True).start()
    
    def _update_device_connection_ui(self, is_connected):
        """Cập nhật UI hiển thị trạng thái kết nối thiết bị"""
        if not hasattr(self, 'device_status_label'):
            return
        
        self.is_livox_device_connected = is_connected
        
        if is_connected:
            self.device_status_label.config(
                text="● " + self.translator.get('label.device_connected', 'Device: Connected'),
                foreground="green"
            )
        else:
            self.device_status_label.config(
                text="● " + self.translator.get('label.device_disconnected', 'Device: Disconnected'),
                foreground="red"
            )
    
    def start_device_connection_monitoring(self):
        """Bắt đầu monitoring kết nối thiết bị định kỳ"""
        def update_periodically():
            if hasattr(self, 'device_status_label'):
                self.check_device_connection()
            # Lên lịch cập nhật tiếp theo
            self.device_connection_timer_id = self.root.after(
                self.device_connection_update_interval,
                update_periodically
            )
        
        # Bắt đầu cập nhật ngay lập tức và sau đó định kỳ
        update_periodically()
    
    def stop_device_connection_monitoring(self):
        """Dừng monitoring kết nối thiết bị"""
        if self.device_connection_timer_id:
            self.root.after_cancel(self.device_connection_timer_id)
            self.device_connection_timer_id = None
    
    def setup_pose_display(self):
        """Thiết lập phần hiển thị pose và orientation"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.pose_orientation', 'Pose & Orientation'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.pose_display_frame = frame  # Store reference for update_ui_texts
        
        # Position section
        self.pos_label = tk.Label(frame, text=self.translator.get('label.position', 'Position:'), font=("Arial", 8, "bold"), fg="gray")
        self.pos_label.pack(anchor=tk.W, pady=(0, 2))
        
        self.pos_x_label = tk.Label(frame, text="X: --", font=("Arial", 8), fg="black")
        self.pos_x_label.pack(anchor=tk.W, padx=10)
        
        self.pos_y_label = tk.Label(frame, text="Y: --", font=("Arial", 8), fg="black")
        self.pos_y_label.pack(anchor=tk.W, padx=10)
        
        self.pos_z_label = tk.Label(frame, text="Z: --", font=("Arial", 8), fg="black")
        self.pos_z_label.pack(anchor=tk.W, padx=10)
        
        # Separator
        separator = tk.Frame(frame, height=1, bg="gray")
        separator.pack(fill=tk.X, pady=5)
        
        # Orientation section
        self.orient_label = tk.Label(frame, text=self.translator.get('label.orientation', 'Orientation:'), font=("Arial", 8, "bold"), fg="gray")
        self.orient_label.pack(anchor=tk.W, pady=(5, 2))
        
        self.orient_x_label = tk.Label(frame, text="X: --", font=("Arial", 8), fg="black")
        self.orient_x_label.pack(anchor=tk.W, padx=10)
        
        self.orient_y_label = tk.Label(frame, text="Y: --", font=("Arial", 8), fg="black")
        self.orient_y_label.pack(anchor=tk.W, padx=10)
        
        self.orient_z_label = tk.Label(frame, text="Z: --", font=("Arial", 8), fg="black")
        self.orient_z_label.pack(anchor=tk.W, padx=10)
        
        self.orient_w_label = tk.Label(frame, text="W: --", font=("Arial", 8), fg="black")
        self.orient_w_label.pack(anchor=tk.W, padx=10)
        
        # Status
        self.pose_status_label = tk.Label(
            frame,
            text=self.translator.get('label.no_data', 'No data'),
            font=("Arial", 7),
            fg="gray"
        )
        self.pose_status_label.pack(anchor=tk.W, pady=(5, 0))
    
    def update_pose_display(self, position, orientation, timestamp):
        """Cập nhật hiển thị pose và orientation"""
        try:
            # Cập nhật position
            self.pos_x_label.config(text=f"X: {position['x']:.3f}")
            self.pos_y_label.config(text=f"Y: {position['y']:.3f}")
            self.pos_z_label.config(text=f"Z: {position['z']:.3f}")
            
            # Cập nhật orientation
            self.orient_x_label.config(text=f"X: {orientation['x']:.4f}")
            self.orient_y_label.config(text=f"Y: {orientation['y']:.4f}")
            self.orient_z_label.config(text=f"Z: {orientation['z']:.4f}")
            self.orient_w_label.config(text=f"W: {orientation['w']:.4f}")
            
            # Cập nhật timestamp (chỉ hiển thị thời gian, không hiển thị ngày)
            try:
                dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                time_str = dt.strftime("%H:%M:%S.%f")[:-3]  # Hiển thị đến millisecond
                self.pose_status_label.config(
                    text=f"Cập nhật: {time_str}",
                    fg="green"
                )
            except:
                self.pose_status_label.config(
                    text="Đang cập nhật...",
                    fg="green"
                )
        except Exception as e:
            self.pose_status_label.config(
                text=f"Lỗi: {e}",
                fg="red"
            )
    
    def reset_pose_display(self):
        """Reset pose và orientation về giá trị mặc định"""
        try:
            # Reset position
            self.pos_x_label.config(text="X: --")
            self.pos_y_label.config(text="Y: --")
            self.pos_z_label.config(text="Z: --")
            
            # Reset orientation
            self.orient_x_label.config(text="X: --")
            self.orient_y_label.config(text="Y: --")
            self.orient_z_label.config(text="Z: --")
            self.orient_w_label.config(text="W: --")
            
            # Reset status
            self.pose_status_label.config(
                text=self.translator.get('label.no_data', 'No data'),
                fg="gray"
            )
        except Exception as e:
            pass  # Ignore errors khi reset

    def setup_log_area(self):
        # Tiêu đề của Log
        log_header = tk.Frame(self.log_container)
        log_header.pack(fill=tk.X, pady=(0, 5))
        
        self.log_title_label = tk.Label(log_header, text=self.translator.get('label.system_monitoring_logs', 'SYSTEM MONITORING LOGS'), font=("Arial", 9, "bold"))
        self.log_title_label.pack(side=tk.LEFT)
        
        self.btn_clear_log = tk.Button(log_header, text=self.translator.get('button.clear_log', 'Clear Log'), 
                              command=self.clear_log, 
                              font=("Arial", 7))
        self.btn_clear_log.pack(side=tk.RIGHT)

        # Log Panel chiếm toàn bộ không gian còn lại
        # Sử dụng scrolledtext để tự động có thanh cuộn nếu log quá dài
        self.log_panel = scrolledtext.ScrolledText(self.log_container, state='disabled', 
                                                   font=("Consolas", 10), relief=tk.SUNKEN, bd=1)
        self.log_panel.pack(fill=tk.BOTH, expand=True)
        
        self.log(self.translator.get('log.system_ready', '✅ System ready'))

    def get_mac_address(self):
        try:
            mac = uuid.getnode()
            return ':'.join(['{:02x}'.format((mac >> elements) & 0xff) for elements in range(40, -1, -8)])
        except: return "unknown"
    
    def is_device_registered(self, vehicle_id):
        """Kiểm tra thiết bị đã đăng ký trên backend chưa"""
        try:
            url = f'{VEHICLE_ENDPOINT}{vehicle_id}'
            headers = {
                'accept': 'application/json'
            }
            response = requests.get(url, headers=headers, timeout=5)
            if response.status_code == 200:
                data = response.json()
                if data.get("vehicle_id"):
                    return True
            return False
        except requests.exceptions.RequestException:
            return False
    
    def check_and_auto_register(self):
        """Kiểm tra và tự động đăng ký thiết bị nếu chưa có trên backend với vai trò worker"""
        try:
            # Lấy MAC address
            mac = self.get_mac_address()
            if mac == "unknown":
                self.log(self.translator.get('log.cannot_get_mac', '⚠️ Cannot get MAC address'))
                return
            
            # Tạo vehicle_id từ MAC (12 ký tự đầu, không có dấu :)
            vehicle_id = mac.lower().replace(':', '')[:12]
            
            # Kiểm tra đã đăng ký chưa
            if self.is_device_registered(vehicle_id):
                self.log(self.translator.get('log.device_already_registered', '✅ Device already registered on backend'))
                return
            
            # Thiết bị chưa đăng ký, tự động đăng ký
            self.log(self.translator.get('log.device_not_registered', '⚠️ Device not registered. Auto-registering as worker...'))
            
            # Fetch available categories và types từ backend
            available_categories = []
            available_types = []
            worker_category = None
            worker_type = None
            
            try:
                # Fetch Categories
                categories_url = f"{self.backend_base_url}/api/v1/vehicles/categories"
                response = requests.get(categories_url, headers=HEADERS, timeout=API_TIMEOUT)
                if response.status_code == 200:
                    data = response.json()
                    available_categories = data.get("categories", [])
                    # Tìm "worker" trong categories (case-insensitive)
                    for cat in available_categories:
                        if cat.lower() == "worker":
                            worker_category = cat
                            break
                
                # Fetch Types
                types_url = f"{self.backend_base_url}/api/v1/vehicles/types"
                response = requests.get(types_url, headers=HEADERS, timeout=API_TIMEOUT)
                if response.status_code == 200:
                    data = response.json()
                    available_types = data.get("types", [])
                    # Tìm "worker" trong types (case-insensitive)
                    for t_val in available_types:
                        if t_val.lower() == "worker":
                            worker_type = t_val
                            break
            except Exception as e:
                self.log(self.translator.get('log.error_fetching_enums', '⚠️ Error fetching categories/types: {error}').replace('{error}', str(e)))
            
            # Tạo payload đăng ký
            payload = {
                "vehicle_id": vehicle_id,
                "name": f"Worker-{vehicle_id}",
                "description": f"Auto-registered worker device (MAC: {mac})",
                "metadata": {}
            }
            
            # Thêm vehicle_category nếu tìm thấy worker
            if worker_category:
                payload["vehicle_category"] = worker_category
            elif available_categories:
                # Nếu không tìm thấy "worker", thử dùng category đầu tiên
                payload["vehicle_category"] = available_categories[0]
            
            # Thêm vehicle_type nếu tìm thấy worker
            if worker_type:
                payload["vehicle_type"] = worker_type
            elif available_types:
                # Nếu không tìm thấy "worker", thử dùng type đầu tiên
                payload["vehicle_type"] = available_types[0]
            
            # Gửi request đăng ký
            response = requests.post(VEHICLE_ENDPOINT, json=payload, headers=HEADERS, timeout=API_TIMEOUT)
            
            if response.status_code in [200, 201]:
                self.log(self.translator.get('log.auto_registration_success', '✅ Auto-registration successful! Device registered as worker'))
                # Cập nhật vehicle_id và refresh info
                self.vehicle_id = vehicle_id
                # Cập nhật UI trong main thread
                self.root.after(0, self._on_auto_registration_success)
            else:
                error_msg = response.text[:200] if response.text else "Unknown error"
                self.log(self.translator.get('log.auto_registration_failed', '❌ Auto-registration failed: {status} - {error}').replace('{status}', str(response.status_code)).replace('{error}', error_msg))
        
        except Exception as e:
            self.log(self.translator.get('log.auto_registration_error', '❌ Error during auto-registration: {error}').replace('{error}', str(e)))
    
    def _on_auto_registration_success(self):
        """Xử lý sau khi auto-registration thành công"""
        # Refresh vehicle info
        if hasattr(self, 'refresh_vehicle_info'):
            self.refresh_vehicle_info()
        # Cập nhật trạng thái vehicle thành online
        if self.vehicle_id:
            self.update_vehicle_status("online", refresh_info=True)
            # Bắt đầu heartbeat nếu chưa bắt đầu
            if not self.heartbeat_timer_id:
                self.start_heartbeat()
    
    def log(self, message):
        self.log_panel.config(state='normal')
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_panel.insert(tk.END, f"[{time_str}] {message}\n")
        self.log_panel.see(tk.END)
        self.log_panel.config(state='disabled')

    def clear_log(self):
        self.log_panel.config(state='normal')
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state='disabled')

    def on_rviz_toggle(self):
        status = "ENABLED" if self.rviz_var.get() else "DISABLED"
        self.log(f"CONFIG: RViz2 visualization {status}.")

    def toggle_movement(self):
        if not self.is_running:
            # Khởi động localization node trong thread để không block UI
            threading.Thread(target=self.start_localization, daemon=True).start()
        else:
            self.stop_system()
    
    def start_localization(self):
        """Khởi động localization node (chạy trong thread để không block UI)"""
        if self.is_localization_running:
            return
        
        # Kiểm tra map có tồn tại không
        if not self.default_map_root.exists() or not (self.default_map_root / "pose.json").exists():
            self.root.after(0, lambda: self.log(self.translator.get('log.map_not_ready_wait', '⚠️ Map not ready. Please wait for map download to complete.')))
            return
        
        # Luôn luôn thử start driver khi người dùng bấm START MOVING (giống như recorder)
        # Không check topics trước vì khi thiết bị chưa start thì không có topics
        self.root.after(0, lambda: self.log("=" * 60))
        
        # Kiểm tra và launch Theta driver trước (cần cho QR scanning)
        self.root.after(0, lambda: self.log(self.translator.get('log.checking_theta_driver', '🔍 Checking Theta driver...')))
        if self.theta_driver:
            try:
                # Check USB connection nếu chưa check
                if not hasattr(self, 'is_theta_connected') or self.is_theta_connected is None:
                    # Check USB trong thread với callback
                    self.theta_driver.check_theta_usb_connection(self.check_theta_usb_callback)
                    time.sleep(0.5)  # Đợi một chút để check hoàn thành
                
                # Launch theta driver nếu chưa chạy và camera đã kết nối
                if self.is_theta_connected:
                    self._launch_theta_driver_if_needed()
                else:
                    self.root.after(0, lambda: self.log(self.translator.get('log.theta_camera_not_connected', '⚠️ Theta camera not connected. QR scanning may not work.')))
            except Exception as e:
                self.root.after(0, lambda: self.log(f"⚠️ Error checking/launching Theta driver: {e}"))
        else:
            self.root.after(0, lambda: self.log(self.translator.get('log.theta_driver_not_available', '⚠️ Theta driver not available. QR scanning will be disabled.')))
        
        self.root.after(0, lambda: self.log(self.translator.get('log.checking_livox_driver', '🔍 Checking Livox driver...')))
        
        if not self.is_livox_driver_running:
            self.root.after(0, lambda: self.log(self.translator.get('log.livox_driver_not_running', '⚠️ Livox driver not running, starting...')))
            # Thử start driver (sẽ tự động detect nếu có thiết bị)
            driver_started = self.start_livox_driver()
            if driver_started:
                self.root.after(0, lambda: self.log(self.translator.get('log.livox_driver_started', '✅ Livox driver đã khởi động thành công')))
            else:
                # Driver không start được, có thể không có thiết bị hoặc đang replay bag
                # Vẫn tiếp tục với localization (có thể đang replay bag)
                self.root.after(0, lambda: self.log(self.translator.get('log.livox_driver_start_failed_continue', '⚠️ Cannot start Livox driver. Continuing with localization (may be bag replay)...')))
        else:
            self.root.after(0, lambda: self.log(self.translator.get('log.livox_driver_already_running', '✅ Livox driver is already running')))
        
        # Script helper
        run_script = Path(project_root) / "scripts" / "run_localization.sh"
        if not run_script.exists():
            self.root.after(0, lambda: self.log(self.translator.get('log.localization_script_not_found', '❌ Localization script not found at: {path}').replace('{path}', str(run_script))))
            return
        
        self.root.after(0, lambda: self.log("=" * 60))
        self.root.after(0, lambda: self.log(self.translator.get('log.starting_localization_node', '🚀 Starting Localization node...')))
        self.root.after(0, lambda: self.log(self.translator.get('log.map_path', '📁 Map: {path}').replace('{path}', str(self.default_map_root))))
        
        try:
            # Chạy localization script với RViz disabled (chỉ chạy node)
            map_root_str = str(self.default_map_root)
            cmd = f"{run_script} {map_root_str} False mid360.yaml"
            
            # Start process
            self.loc_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_localization_running = True
            self.is_running = True
            threading.Thread(target=self.monitor_localization_process, daemon=True).start()
            
            # Bắt đầu publish pose lên backend
            self.start_pose_publishing()
            
            # Bắt đầu QR scanning (cần theta driver đã chạy)
            # Đợi một chút để đảm bảo theta driver đã publish topic /image_raw
            time.sleep(1.0)
            self.start_qr_scanning()
            
            # Cập nhật UI trong main thread
            self.root.after(0, lambda: self.btn_start.config(text=self.translator.get('button.stop_movement', '■ STOP MOVEMENT'), state=tk.NORMAL))
            self.root.after(0, lambda: self.status_text.config(text=self.translator.get('label.localization_running', 'System: Localization running...')))
            self.root.after(0, lambda: self.log(self.translator.get('log.localization_node_started', '✅ Localization node started.')))
            
        except Exception as e:
            self.root.after(0, lambda: self.log(self.translator.get('log.error_starting_localization', '❌ Error starting localization: {error}').replace('{error}', str(e))))
            self.is_localization_running = False
            self.is_running = False

    def stop_system(self):
        """Dừng localization node"""
        # Dừng QR scanning trước
        self.stop_qr_scanning()
        self.stop_pose_publishing()
        self.stop_localization()
        
        # Dừng Theta driver nếu đang chạy
        if self.theta_driver:
            try:
                self.log(self.translator.get('log.stopping_theta_driver', '🛑 Stopping Theta driver...'))
                if hasattr(self.theta_driver, 'stop_all'):
                    self.theta_driver.stop_all()
                else:
                    # Fallback: kill process trực tiếp
                    if hasattr(self.theta_driver, 'theta_driver_process') and \
                       self.theta_driver.theta_driver_process:
                        try:
                            self.theta_driver.theta_driver_process.terminate()
                            self.theta_driver.theta_driver_process.wait(timeout=3)
                        except:
                            if self.theta_driver.theta_driver_process:
                                self.theta_driver.theta_driver_process.kill()
                    
                    # Kill camera_info_publisher nếu có
                    if hasattr(self.theta_driver, 'camera_info_publisher_process') and \
                       self.theta_driver.camera_info_publisher_process:
                        try:
                            self.theta_driver.camera_info_publisher_process.terminate()
                            self.theta_driver.camera_info_publisher_process.wait(timeout=3)
                        except:
                            if self.theta_driver.camera_info_publisher_process:
                                self.theta_driver.camera_info_publisher_process.kill()
                
                # Kill các ROS node của theta driver bằng pkill và ros2 node kill
                self.log(self.translator.get('log.killing_theta_nodes', '🔍 Killing theta driver ROS nodes...'))
                try:
                    # Kill bằng pkill
                    subprocess.run(
                        ['pkill', '-f', 'theta_driver_node'],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=3
                    )
                    
                    subprocess.run(
                        ['pkill', '-f', 'camera_info_publisher_node'],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=3
                    )
                    
                    # Kill bằng ros2 node kill (nếu có ROS2 environment)
                    try:
                        ros2_setup = "/opt/ros/jazzy/setup.bash"
                        ws_setup = str(self.workspace_path / "install" / "setup.sh")
                        
                        if os.path.exists(ros2_setup) and os.path.exists(ws_setup):
                            kill_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 node list | grep -E 'theta_driver|camera_info_publisher' | xargs -r ros2 node kill"
                            subprocess.run(
                                kill_cmd,
                                shell=True,
                                executable="/bin/bash",
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                                timeout=5
                            )
                    except:
                        pass  # Ignore nếu không có ROS2 environment
                    
                    self.log(self.translator.get('log.theta_nodes_killed', '✅ All theta driver nodes killed'))
                except Exception as e:
                    self.log(f"⚠️ Error killing theta nodes: {e}")
                
                self.log(self.translator.get('log.theta_driver_stopped', '✅ Theta driver stopped'))
            except Exception as e:
                self.log(f"⚠️ Error stopping Theta driver: {e}")
        
        # Dừng Livox driver nếu đã khởi động tự động
        if self.is_livox_driver_running:
            self.stop_livox_driver()
        self.is_running = False
        self.auto_start_triggered = False  # Reset flag khi stop
        self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'), state=tk.NORMAL)
        self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
        # Reset pose display
        self.reset_pose_display()
        self.log(self.translator.get('log.stop_localization_requested', 'COMMAND: Stop localization requested.'))
    
    def download_map_and_qr(self):
        """Tự động tải fastloc_map và QR từ backend"""
        self.log(self.translator.get('log.downloading_map_qr', '🌐 Loading map and QR from backend...'))
        threading.Thread(target=self._download_worker, daemon=True).start()
    
    def _download_worker(self):
        """Worker thread để tải map và QR từ backend"""
        try:
            base_url = self.backend_base_url
            
            # Tải fastloc_map
            fastloc_url = f"{base_url}/api/v1/maps/localization/fastloc/download"
            self.log(self.translator.get('log.downloading_fastloc_map', '⬇️ Downloading fastloc_map from backend...'))
            
            dest_root = self.log_path
            dest_root.mkdir(parents=True, exist_ok=True)
            dest_zip = dest_root / "fastloc_map.zip"
            
            try:
                with requests.get(fastloc_url, stream=True, timeout=120) as r:
                    if r.status_code != 200:
                        self.log(self.translator.get('log.fastloc_map_download_failed', '❌ Failed to download fastloc_map (HTTP {code})').replace('{code}', str(r.status_code)))
                        return
                    with open(dest_zip, "wb") as f:
                        for chunk in r.iter_content(chunk_size=8192):
                            if chunk:
                                f.write(chunk)
                
                self.log(self.translator.get('log.extracting_fastloc_map', '📦 Extracting fastloc_map...'))
                # Xóa map cũ để tránh lẫn file
                if self.default_map_root.exists():
                    shutil.rmtree(self.default_map_root, ignore_errors=True)
                
                # Giải nén vào temp folder trước
                temp_extract_dir = dest_root / "temp_extract"
                if temp_extract_dir.exists():
                    shutil.rmtree(temp_extract_dir, ignore_errors=True)
                temp_extract_dir.mkdir(parents=True, exist_ok=True)
                
                # Giải nén zip vào temp folder
                with zipfile.ZipFile(dest_zip, 'r') as zip_ref:
                    zip_ref.extractall(temp_extract_dir)
                
                # Tìm folder fastloc_map trong temp folder hoặc di chuyển nội dung
                extracted_fastloc = None
                for item in temp_extract_dir.iterdir():
                    if item.is_dir() and item.name == "fastloc_map":
                        extracted_fastloc = item
                        break
                
                if extracted_fastloc:
                    # Zip chứa folder fastloc_map, di chuyển vào đúng vị trí
                    shutil.move(str(extracted_fastloc), str(self.default_map_root))
                else:
                    # Zip chứa file trực tiếp, tạo folder fastloc_map và di chuyển nội dung vào
                    self.default_map_root.mkdir(parents=True, exist_ok=True)
                    for item in temp_extract_dir.iterdir():
                        dest_item = self.default_map_root / item.name
                        if item.is_dir():
                            if dest_item.exists():
                                shutil.rmtree(dest_item, ignore_errors=True)
                            shutil.move(str(item), str(dest_item))
                        else:
                            shutil.move(str(item), str(dest_item))
                
                # Xóa temp folder
                if temp_extract_dir.exists():
                    shutil.rmtree(temp_extract_dir, ignore_errors=True)
                
                # Xóa file zip tạm
                try:
                    dest_zip.unlink()
                    self.log(self.translator.get('log.deleted_temp_zip', '🧹 Deleted temporary zip file: {filename}').replace('{filename}', dest_zip.name))
                except Exception as e:
                    self.log(self.translator.get('log.cannot_delete_temp_zip', '⚠️ Cannot delete temporary zip file: {error}').replace('{error}', str(e)))
                
                # Kiểm tra lại folder fastloc_map
                if self.default_map_root.exists():
                    self.log(self.translator.get('log.fastloc_map_downloaded', '✅ fastloc_map downloaded and extracted successfully'))
                    # Cập nhật thông tin map sau khi tải xong
                    self.root.after(0, self._on_map_downloaded)
                else:
                    self.log(self.translator.get('log.fastloc_map_not_exists', '⚠️ fastloc_map folder does not exist after extraction'))
                
            except requests.exceptions.RequestException as e:
                self.log(self.translator.get('log.error_downloading_fastloc_map', '❌ Error downloading fastloc_map: {error}').replace('{error}', str(e)))
                return
            except Exception as e:
                self.log(self.translator.get('log.error_extracting_fastloc_map', '❌ Error extracting fastloc_map: {error}').replace('{error}', str(e)))
                return
            
            # Tải QR_detect.json
            qr_url = f"{base_url}/api/v1/maps/localization/qr"
            self.log(self.translator.get('log.downloading_qr_detect', '⬇️ Downloading QR_detect.json from backend...'))
            
            try:
                resp = requests.get(qr_url, timeout=30)
                if resp.status_code == 200:
                    with open(self.qr_detect_path, 'wb') as f:
                        f.write(resp.content)
                    self.log(self.translator.get('log.qr_detect_downloaded', '✅ QR_detect.json downloaded successfully'))
                    # Reload QR detect data sau khi download
                    self.root.after(0, self.reload_qr_detect_data)
                else:
                    self.log(self.translator.get('log.qr_detect_download_failed', '⚠️ Failed to download QR_detect.json (HTTP {code})').replace('{code}', str(resp.status_code)))
            except requests.exceptions.RequestException as e:
                self.log(self.translator.get('log.error_downloading_qr_detect', '⚠️ Error downloading QR_detect.json: {error}').replace('{error}', str(e)))
            
            # Map đã sẵn sàng, người dùng có thể bấm nút Preview để mở RViz
            self.log(self.translator.get('log.map_qr_ready', '✅ Map and QR are ready. Click Preview button to open RViz.'))
            
        except Exception as e:
            self.log(self.translator.get('log.unexpected_error_downloading', '❌ Unexpected error downloading map/QR: {error}').replace('{error}', str(e)))
    
    def _on_map_downloaded(self):
        """Xử lý sau khi tải map thành công"""
        self.update_map_info()
        # Enable nút START MOVING
        self.btn_start.config(state=tk.NORMAL)
        self.status_text.config(text=self.translator.get('label.system_ready', 'System: Ready to start'))
        
        # Tự động bấm nút START MOVING sau delay ngắn
        if not self.auto_start_triggered:
            self.auto_start_triggered = True
            self.log(self.translator.get('log.auto_starting', '⏱️ Auto-starting in 1 second...'))
            # Delay 1 giây (1000ms) trước khi tự động start
            self.root.after(1000, self._auto_start_movement)
    
    def _auto_start_movement(self):
        """Tự động bấm nút START MOVING"""
        if not self.is_running and self.btn_start['state'] == tk.NORMAL:
            self.log(self.translator.get('log.auto_start_triggered', '🚀 Auto-starting movement...'))
            self.toggle_movement()
        else:
            self.log(self.translator.get('log.auto_start_skipped', '⏭️ Auto-start skipped (system already running or button disabled)'))
    
    def update_vehicle_status(self, status, refresh_info=False):
        """Cập nhật trạng thái vehicle lên backend"""
        if not self.vehicle_id:
            return
        
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/status"
            payload = {
                "status": status
            }
            
            response = requests.patch(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            if response.status_code == 200:
                self.log(self.translator.get('log.vehicle_status_updated', '✅ Vehicle status updated: {status}').replace('{status}', status))
                # Reset heartbeat log flag khi status thay đổi (không phải online)
                if status != "online":
                    self.heartbeat_logged = False
                # Chỉ refresh vehicle info khi status là "online" và refresh_info=True
                if status == "online" and refresh_info:
                    self.root.after(0, self.refresh_vehicle_info)
            else:
                self.log(self.translator.get('log.cannot_update_vehicle_status', '⚠️ Cannot update vehicle status: HTTP {code}').replace('{code}', str(response.status_code)))
                
        except requests.exceptions.RequestException as e:
            self.log(self.translator.get('log.error_connection_updating_status', '⚠️ Connection error when updating status: {error}').replace('{error}', str(e)))
        except Exception as e:
            self.log(self.translator.get('log.error_updating_status', '⚠️ Error updating status: {error}').replace('{error}', str(e)))
    
    def start_heartbeat(self):
        """Bắt đầu heartbeat loop để gửi status online định kỳ"""
        if not self.vehicle_id:
            return
        
        # Dừng heartbeat cũ nếu có
        self.stop_heartbeat()
        
        # Bắt đầu heartbeat loop
        self._heartbeat_loop()
    
    def stop_heartbeat(self):
        """Dừng heartbeat loop"""
        if self.heartbeat_timer_id is not None:
            self.root.after_cancel(self.heartbeat_timer_id)
            self.heartbeat_timer_id = None
    
    def _heartbeat_loop(self):
        """Gửi heartbeat và schedule lần tiếp theo"""
        if not self.vehicle_id:
            return
        
        # Gửi heartbeat trong background thread để không block UI
        threading.Thread(target=self._send_heartbeat_async, daemon=True).start()
        
        # Schedule lần tiếp theo
        self.heartbeat_timer_id = self.root.after(HEARTBEAT_INTERVAL_SECONDS * 1000, self._heartbeat_loop)
    
    def _send_heartbeat_async(self):
        """Gửi heartbeat trong background thread để không block UI"""
        if not self.vehicle_id:
            return
        
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/status"
            payload = {
                "status": "online"
            }
            
            response = requests.patch(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            # Update UI trong main thread (thread-safe)
            if response.status_code == 200:
                # Chỉ log lần đầu tiên khi heartbeat thành công
                if not self.heartbeat_logged:
                    self.root.after(0, lambda: self.log(
                        self.translator.get('log.vehicle_status_updated', '✅ Vehicle status updated: {status}').replace('{status}', 'online')
                    ))
                    self.heartbeat_logged = True
            else:
                self.root.after(0, lambda: self.log(
                    self.translator.get('log.cannot_update_vehicle_status', '⚠️ Cannot update vehicle status: HTTP {code}').replace('{code}', str(response.status_code))
                ))
                
        except requests.exceptions.RequestException as e:
            # Log lỗi trong main thread (thread-safe)
            # Capture giá trị của e vào default parameter để tránh closure issue
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.log(
                self.translator.get('log.error_connection_updating_status', '⚠️ Connection error when updating status: {error}').replace('{error}', msg)
            ))
        except Exception as e:
            # Log lỗi trong main thread (thread-safe)
            # Capture giá trị của e vào default parameter để tránh closure issue
            error_msg = str(e)
            self.root.after(0, lambda msg=error_msg: self.log(
                self.translator.get('log.error_heartbeat', '⚠️ Heartbeat error: {error}').replace('{error}', msg)
            ))
    
    def load_qr_detect_data(self):
        """Load và parse file QR_detect.json"""
        try:
            if not self.qr_detect_path.exists():
                self.log(self.translator.get('log.qr_detect_file_not_found', '⚠️ QR_detect.json not found at: {path}').replace('{path}', str(self.qr_detect_path)))
                self.qr_detect_data = {}
                return
            
            with open(self.qr_detect_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self.qr_detect_data = data
                
            # Log số lượng QR codes đã load
            qr_count = len(self.qr_detect_data)
            if qr_count > 0:
                self.log(self.translator.get('log.qr_detect_loaded', '✅ Loaded {count} QR codes from QR_detect.json').replace('{count}', str(qr_count)))
            else:
                self.log(self.translator.get('log.qr_detect_empty', '⚠️ QR_detect.json is empty'))
                
        except json.JSONDecodeError as e:
            self.log(self.translator.get('log.qr_detect_json_error', '❌ Error parsing QR_detect.json: {error}').replace('{error}', str(e)))
            self.qr_detect_data = {}
        except Exception as e:
            self.log(self.translator.get('log.qr_detect_load_error', '❌ Error loading QR_detect.json: {error}').replace('{error}', str(e)))
            self.qr_detect_data = {}
    
    def reload_qr_detect_data(self):
        """Reload QR_detect.json (có thể gọi khi file được cập nhật)"""
        self.load_qr_detect_data()
    
    def start_qr_scanning(self):
        """Khởi động QR scanning từ ROS topic /image_raw (theta driver)"""
        if not QR_SCANNER_AVAILABLE or not PYZBAR_AVAILABLE:
            self.log(self.translator.get('log.qr_scanner_not_available', '⚠️ QR scanner not available (cv2 or pyzbar not installed)'))
            return
        
        if not ROS2_AVAILABLE or not CV_BRIDGE_AVAILABLE:
            self.log(self.translator.get('log.ros2_cvbridge_not_available', '⚠️ ROS2 or cv_bridge not available. QR scanning requires ROS topic /image_raw'))
            return
        
        if self.is_qr_scanning:
            return  # Đã đang quét
        
        if not self.is_localization_running:
            self.log(self.translator.get('log.qr_scanning_requires_localization', '⚠️ QR scanning requires localization to be running'))
            return
        
        try:
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                rclpy.init()
            
            # Tạo ROS node để subscribe image topic từ theta driver
            self.qr_ros_node = QRImageSubscriberNode(
                '/image_raw',  # Topic từ theta driver
                self.on_qr_image_received,
                'qr_image_subscriber_worker'
            )
            
            # Tạo executor
            self.qr_ros_executor = SingleThreadedExecutor()
            self.qr_ros_executor.add_node(self.qr_ros_node)
            
            # Kiểm tra topic có tồn tại không
            self.log(self.translator.get('log.checking_image_topic', '🔍 Checking /image_raw topic...'))
            time.sleep(1)  # Đợi một chút để topic có thể xuất hiện
            
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if '/image_raw' in result.stdout:
                self.log(self.translator.get('log.image_topic_exists', '✓ Topic /image_raw exists'))
            else:
                self.log(self.translator.get('log.image_topic_not_found_warning', '⚠️ Warning: Topic /image_raw not found. Theta driver may not be running.'))
                self.log(self.translator.get('log.will_try_subscribe_anyway', '   Will try to subscribe anyway and wait for topic...'))
            
            # Khởi động ROS executor trong background thread
            self.is_qr_scanning = True
            self.qr_frame_count = 0
            
            self.qr_ros_thread = threading.Thread(
                target=self._qr_ros_spin,
                daemon=True
            )
            self.qr_ros_thread.start()
            
            self.log(self.translator.get('log.qr_scanning_started', '✅ QR scanning started (subscribing to /image_raw)'))
            
            # Cập nhật UI
            self.root.after(0, self.update_qr_scanning_ui)
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_qr_scanning', '❌ Error starting QR scanning: {error}').replace('{error}', str(e)))
            self.is_qr_scanning = False
            if self.qr_ros_node:
                self.qr_ros_node.destroy_node()
                self.qr_ros_node = None
            if self.qr_ros_executor:
                self.qr_ros_executor = None
    
    def stop_qr_scanning(self):
        """Dừng QR scanning"""
        if not self.is_qr_scanning:
            return
        
        self.is_qr_scanning = False
        
        # Dừng ROS executor
        if self.qr_ros_executor:
            try:
                if self.qr_ros_node:
                    self.qr_ros_executor.remove_node(self.qr_ros_node)
                    self.qr_ros_node.destroy_node()
            except:
                pass
            self.qr_ros_executor = None
        
        # Đợi thread kết thúc
        if self.qr_ros_thread and self.qr_ros_thread.is_alive():
            self.qr_ros_thread.join(timeout=2)
        
        # Cleanup
        self.qr_ros_node = None
        self.qr_ros_executor = None
        self.qr_ros_thread = None
        
        self.log(self.translator.get('log.qr_scanning_stopped', '🛑 QR scanning stopped'))
        
        # Cập nhật UI
        self.root.after(0, self.update_qr_scanning_ui)
    
    def _qr_ros_spin(self):
        """Spin ROS executor trong background thread cho QR scanning"""
        try:
            while self.is_qr_scanning and self.is_localization_running and rclpy.ok():
                if self.qr_ros_executor is not None:
                    self.qr_ros_executor.spin_once(timeout_sec=0.1)
                else:
                    break
        except Exception as e:
            if self.is_qr_scanning:
                self.root.after(0, lambda: self.log(
                    self.translator.get('log.error_qr_ros_spin', '❌ Error in QR ROS spin: {error}').replace('{error}', str(e))
                ))
        finally:
            self.is_qr_scanning = False
    
    def on_qr_image_received(self, cv_image):
        """Callback khi nhận được ảnh từ ROS topic để quét QR"""
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
            # Log lỗi trong main thread
            self.root.after(0, lambda: self.log(
                self.translator.get('log.error_qr_image_callback', '⚠️ Error in QR image callback: {error}').replace('{error}', str(e))
            ))
    
    def _scan_qr_in_background(self, cv_image):
        """Quét QR code trong background thread"""
        try:
            # Chuyển sang grayscale để tăng tốc độ xử lý
            try:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            except:
                # Nếu frame không phải RGB, thử BGR
                try:
                    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                except:
                    # Nếu frame không phải color, dùng trực tiếp
                    if len(cv_image.shape) == 2:
                        gray = cv_image
                    else:
                        return
            
            # Resize frame lớn xuống để tăng tốc độ (camera 360 có thể rất lớn)
            height, width = gray.shape[:2]
            if width > 1600:
                scale = 1600 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                gray = cv2.resize(gray, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            # Detect QR codes
            try:
                qr_codes = pyzbar.decode(gray)
                
                if qr_codes:
                    for qr in qr_codes:
                        try:
                            qr_data = qr.data.decode('utf-8')
                            # Gọi callback để xử lý QR code
                            self.root.after(0, lambda data=qr_data: self.on_qr_detected(data))
                        except Exception as e:
                            # Ignore lỗi decode
                            pass
            except Exception as e:
                # Ignore lỗi khi decode QR
                pass
                
        except Exception as e:
            # Ignore lỗi trong background thread
            pass
    
    def on_qr_detected(self, qr_data):
        """Xử lý khi phát hiện QR code"""
        try:
            # Cập nhật QR code cuối cùng được phát hiện
            with self.qr_lock:
                self.last_detected_qr = qr_data
            
            # Cập nhật UI
            self.root.after(0, self.update_qr_scanning_ui)
            
            # Kiểm tra xem QR code có trong danh sách không
            if qr_data not in self.qr_detect_data:
                # QR code không khớp, chỉ log
                self.log(self.translator.get('log.qr_not_in_list', '📷 QR detected but not in list: {qr}').replace('{qr}', qr_data))
                return
            
            # QR code khớp, lấy tọa độ từ JSON
            qr_coords = self.qr_detect_data[qr_data]
            
            if not isinstance(qr_coords, list) or len(qr_coords) < 3:
                self.log(self.translator.get('log.qr_invalid_coords', '⚠️ Invalid coordinates for QR {qr}').replace('{qr}', qr_data))
                return
            
            x, y, z = qr_coords[0], qr_coords[1], qr_coords[2]
            
            # Tạo pose với tọa độ QR
            position = {
                "x": float(x),
                "y": float(y),
                "z": float(z)
            }
            
            # Sử dụng orientation mặc định (identity quaternion)
            # Có thể thay đổi để giữ orientation hiện tại từ localization nếu cần
            orientation = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
            
            # Tạo timestamp hiện tại (UTC)
            timestamp_iso = datetime.now(timezone.utc).isoformat()
            
            # Gửi pose lên backend
            self.log(f"✅ QR matched: {qr_data} -> Sending pose ({x:.2f}, {y:.2f}, {z:.2f})")
            
            # Gửi trong background thread để không block UI
            threading.Thread(
                target=self.send_pose_to_backend,
                args=(position, orientation, timestamp_iso),
                daemon=True
            ).start()
            
        except Exception as e:
            self.log(self.translator.get('log.error_processing_qr', '❌ Error processing QR code: {error}').replace('{error}', str(e)))
    
    def load_autostart_config(self):
        """Load cấu hình autostart từ file JSON"""
        default_config = {
            "autostart_enabled": False,
            "autostart_first_run": True  # True = lần đầu, False = từ lần sau
        }
        
        if self.autostart_config_path.exists():
            try:
                with open(self.autostart_config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    # Đảm bảo có tất cả các key cần thiết
                    for key in default_config:
                        if key not in config:
                            config[key] = default_config[key]
                    # Migration: nếu có autostart_prompt_shown cũ, chuyển sang autostart_first_run
                    if "autostart_prompt_shown" in config and "autostart_first_run" not in config:
                        # Nếu đã hỏi rồi (prompt_shown = True), thì không phải lần đầu nữa
                        config["autostart_first_run"] = not config.get("autostart_prompt_shown", False)
                    return config
            except Exception as e:
                self.log(f"⚠️ Error loading autostart config: {e}")
                return default_config
        return default_config
    
    def save_autostart_config(self, config):
        """Lưu cấu hình autostart vào file JSON"""
        try:
            with open(self.autostart_config_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
        except Exception as e:
            self.log(f"⚠️ Error saving autostart config: {e}")
    
    def check_autostart_enabled(self):
        """Kiểm tra xem autostart đã được bật trong systemd chưa"""
        try:
            result = subprocess.run(
                ['systemctl', '--user', 'is-enabled', 'livo2-working.service'],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0 and 'enabled' in result.stdout
        except Exception as e:
            self.log(f"⚠️ Error checking autostart status: {e}")
            return False
    
    def install_autostart(self):
        """Cài đặt autostart bằng cách chạy install_autostart.sh (chạy trong background thread)"""
        if not self.install_autostart_script.exists():
            self.log(f"❌ {self.translator.get('error.autostart_script_not_found', 'Autostart script not found')}: {self.install_autostart_script}")
            self._on_autostart_failed(True, self.translator.get('error.autostart_script_not_found', 'Autostart script not found'))
            return
        
        try:
            # Chạy script với echo "y" để tự động chấp nhận
            # Tuy nhiên script hiện tại yêu cầu input, nên ta sẽ chạy non-interactive
            # Hoặc có thể sửa script để nhận argument --yes
            # Tạm thời chạy với expect hoặc sửa script
            # Vì script yêu cầu input, ta sẽ chạy trong thread và hiển thị thông báo
            def run_install():
                try:
                    # Tạo một script wrapper để tự động trả lời yes cho cả 2 prompt
                    import tempfile
                    wrapper_script = tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False)
                    wrapper_script.write(f'''#!/bin/bash
cd "{Path(project_root)}"
# Trả lời yes cho cả 2 prompt (overwrite nếu có, và confirm)
printf "y\\ny\\n" | bash "{self.install_autostart_script}"
''')
                    wrapper_script.close()
                    os.chmod(wrapper_script.name, 0o755)
                    
                    result = subprocess.run(
                        ['bash', wrapper_script.name],
                        capture_output=True,
                        text=True,
                        timeout=60
                    )
                    
                    os.unlink(wrapper_script.name)
                    
                    # Kiểm tra xem service có được enable thành công không
                    # "Created symlink" là thông báo thành công từ systemctl, không phải lỗi
                    # Hoặc có thể script exit với code khác 0 nhưng service vẫn được enable
                    time.sleep(0.5)  # Đợi một chút để systemd cập nhật
                    
                    # Kiểm tra trực tiếp xem service có được enable không (source of truth)
                    if self.check_autostart_enabled():
                        # Cập nhật config
                        config = self.load_autostart_config()
                        config["autostart_enabled"] = True
                        self.save_autostart_config(config)
                        # Gọi callback trong main thread
                        self.root.after(0, lambda: self._on_autostart_enabled())
                    else:
                        # Service chưa được enable, kiểm tra output để xem có thông báo thành công không
                        output = (result.stdout or "") + (result.stderr or "")
                        if "Created symlink" in output or "Service enabled" in output or result.returncode == 0:
                            # Có thể service đang được enable nhưng chưa kịp cập nhật, thử lại
                            time.sleep(1)
                            if self.check_autostart_enabled():
                                config = self.load_autostart_config()
                                config["autostart_enabled"] = True
                                self.save_autostart_config(config)
                                self.root.after(0, lambda: self._on_autostart_enabled())
                            else:
                                error_msg = output[:300] if output else "Unknown error"
                                self.root.after(0, lambda: self._on_autostart_failed(True, error_msg))
                        else:
                            error_msg = output[:300] if output else f"Exit code: {result.returncode}"
                            self.root.after(0, lambda: self._on_autostart_failed(True, error_msg))
                except Exception as e:
                    self.root.after(0, lambda: self._on_autostart_failed(True, str(e)))
            
            threading.Thread(target=run_install, daemon=True).start()
        except Exception as e:
            self._on_autostart_failed(True, str(e))
    
    def disable_autostart(self):
        """Tắt autostart bằng cách disable systemd service (chạy trong background thread)"""
        def run_disable():
            try:
                result = subprocess.run(
                    ['systemctl', '--user', 'disable', 'livo2-working.service'],
                    capture_output=True,
                    text=True,
                    timeout=10
                )
                
                if result.returncode == 0:
                    # Cập nhật config
                    config = self.load_autostart_config()
                    config["autostart_enabled"] = False
                    self.save_autostart_config(config)
                    # Gọi callback trong main thread
                    self.root.after(0, lambda: self._on_autostart_disabled())
                else:
                    error_msg = result.stderr or result.stdout
                    self.root.after(0, lambda: self._on_autostart_failed(False, error_msg))
            except Exception as e:
                self.root.after(0, lambda: self._on_autostart_failed(False, str(e)))
        
        threading.Thread(target=run_disable, daemon=True).start()
    
    def _on_autostart_enabled(self):
        """Callback khi autostart được enable thành công"""
        self.chk_autostart.config(
            state=tk.NORMAL, 
            text=self.translator.get('label.autostart', 'Auto-start on boot')
        )
        self.autostart_var.set(True)
        self.log(self.translator.get('log.autostart_enabled', '✅ Autostart enabled successfully. Application will start automatically on boot.'))
    
    def _on_autostart_disabled(self):
        """Callback khi autostart được disable thành công"""
        self.chk_autostart.config(
            state=tk.NORMAL, 
            text=self.translator.get('label.autostart', 'Auto-start on boot')
        )
        self.autostart_var.set(False)
        self.log(self.translator.get('log.autostart_disabled', '✅ Autostart disabled successfully. Application will not start automatically on boot.'))
    
    def _on_autostart_failed(self, desired_state, error_msg=None):
        """Callback khi autostart operation fail"""
        # Revert checkbox state về trạng thái ban đầu
        self.autostart_var.set(not desired_state)
        self.chk_autostart.config(
            state=tk.NORMAL, 
            text=self.translator.get('label.autostart', 'Auto-start on boot')
        )
        
        # Hiển thị error message
        if error_msg:
            error_text = error_msg[:300] if len(error_msg) > 300 else error_msg
            self.log(f"❌ {self.translator.get('error.autostart_install_failed' if desired_state else 'error.autostart_disable_failed', 'Failed to change autostart status')}: {error_text}")
            messagebox.showerror(
                self.translator.get('title.error', 'Error'),
                self.translator.get(
                    'error.autostart_install_failed' if desired_state else 'error.autostart_disable_failed',
                    'Failed to change autostart status'
                ) + (f": {error_text}" if error_text else "")
            )
        else:
            self.log(f"❌ {self.translator.get('error.autostart_install_failed' if desired_state else 'error.autostart_disable_failed', 'Failed to change autostart status')}")
            messagebox.showerror(
                self.translator.get('title.error', 'Error'),
                self.translator.get(
                    'error.autostart_install_failed' if desired_state else 'error.autostart_disable_failed',
                    'Failed to change autostart status'
                )
            )
    
    def check_and_prompt_autostart(self):
        """Kiểm tra và hiển thị thông báo hỏi có muốn tự động khởi động khi mới lên"""
        config = self.load_autostart_config()
        
        # Lần đầu tiên: đánh dấu đã chạy lần đầu và không hiển thị thông báo
        if config.get("autostart_first_run", True):
            config["autostart_first_run"] = False
            self.save_autostart_config(config)
            return
        
        # Từ lần thứ 2 trở đi: kiểm tra và hiển thị thông báo nếu chưa bật autostart
        # Chạy check trong background thread để không block UI
        def check_and_show_prompt():
            try:
                if self.check_autostart_enabled():
                    # Đã bật rồi, không cần hỏi nữa
                    return
                
                # Hiển thị thông báo hỏi trong main thread
                self.root.after(0, lambda: self._show_autostart_prompt())
            except Exception as e:
                self.root.after(0, lambda: self.log(f"⚠️ Error checking autostart status: {e}"))
        
        threading.Thread(target=check_and_show_prompt, daemon=True).start()
    
    def _show_autostart_prompt(self):
        """Hiển thị dialog hỏi user có muốn enable autostart"""
        response = messagebox.askyesno(
            self.translator.get('title.autostart', 'Autostart'),
            self.translator.get('prompt.autostart_question', 
                              'Do you want to automatically start this application when the computer boots?')
        )
        
        if response:
            # User chọn Yes, cài đặt autostart
            self.log(self.translator.get('log.autostart_user_enable', '📋 User chose to enable autostart. Installing...'))
            self.install_autostart()
        else:
            # User chọn No
            config = self.load_autostart_config()
            config["autostart_enabled"] = False
            self.save_autostart_config(config)
            self.log(self.translator.get('log.autostart_user_disable', '📋 User chose not to enable autostart.'))
    
    def on_closing(self):
        """Xử lý khi đóng cửa sổ"""
        # Dừng QR scanning
        if self.is_qr_scanning:
            self.stop_qr_scanning()
        
        # Dừng Theta driver nếu đang chạy
        if self.theta_driver:
            try:
                if hasattr(self.theta_driver, 'stop_all'):
                    self.theta_driver.stop_all()
            except Exception as e:
                self.log(f"⚠️ Error stopping Theta driver: {e}")
        
        # Dừng monitoring kết nối thiết bị
        self.stop_device_connection_monitoring()
        
        # Dừng heartbeat
        self.stop_heartbeat()
        
        # Cập nhật trạng thái vehicle thành offline
        if self.vehicle_id:
            self.update_vehicle_status("offline")
        
        # Dừng tất cả các process đang chạy
        if self.is_running:
            self.stop_system()
        
        # Dừng Livox driver nếu đang chạy
        if self.is_livox_driver_running:
            self.stop_livox_driver()
        
        # Đóng cửa sổ
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkerInterface(root)
    root.mainloop()