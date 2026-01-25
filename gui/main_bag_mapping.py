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
SingleThreadedExecutor = None
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    ROS2_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError) as e:
    ROS2_AVAILABLE = False
    print(f"Warning: ROS2 not available: {e}. QR code scanning from bag will be disabled.")

# Import bag_mapping modules
try:
    from bag_mapping.config import BagMappingConfig
    from bag_mapping.qr_code import QRScanner, QRImageSubscriberNode, PYZBAR_AVAILABLE, GEOMETRY_UTILS_AVAILABLE, ROS2_AVAILABLE as QR_ROS2_AVAILABLE
    from bag_mapping.logging import LoggingManager
    from bag_mapping.progress import ProgressManager
    from bag_mapping.utils import get_local_mac_no_colon, cleanup_ros2_resources, check_pcd_files, set_default_config
    from bag_mapping.validation import validate_config_file, validate_config_parameters, check_system_resources, validate_bag_file
    from bag_mapping.language import LanguageManager
    from bag_mapping.ui_components import UIComponentsBuilder
    from bag_mapping.design_comparison import DesignMapComparison
    from bag_mapping.pcd_processing import PCDProcessor
    from bag_mapping.upload import UploadManager
    from bag_mapping.core_mapping import MappingCore
except ImportError as e:
    print(f"Error importing bag_mapping modules: {e}")
    raise


class BagMappingInterface:
    def __init__(self, root):
        self.root = root
        
        # Initialize config - tập trung tất cả constants và paths
        self.config = BagMappingConfig()
        self.workspace_path = self.config.workspace_path
        self.drive_ws_path = self.config.drive_ws_path
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
        # Bag rate từ config
        self.bag_rate = self.config.bag_rate
        # Backend URL từ config
        self.backend_base_url = self.config.backend_base_url
        # Thư mục lưu PCD convert từ OBJ (bản thiết kế)
        self.obj_converted_dir = self.config.obj_converted_dir
        # Cấu hình so sánh bản đồ thiết kế
        self.design_file_type = self.config.design_file_type
        self.design_file_path = None
        self.comparison_result = None
        self.comparison_threshold = self.config.comparison_threshold
        # File config để lưu đường dẫn design file mặc định
        self.design_config_path = self.config.design_config_path
        
        # QR code scanning - chỉ khởi tạo nếu tất cả dependencies có sẵn
        try:
            if PYZBAR_AVAILABLE and GEOMETRY_UTILS_AVAILABLE and cv2 is not None:
                self.qr_scanner = QRScanner(
                    perspec_size=self.config.qr_scanner_perspec_size,
                    max_workers=self.config.qr_scanner_max_workers
                )
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
        self.qr_scan_enabled = self.config.qr_scan_enabled
        self.qr_scan_frame_interval = self.config.qr_scan_frame_interval
        self.qr_image_subscriber = None
        self.qr_ros_node = None
        self.qr_ros_executor = None
        self.qr_ros_thread = None
        self.qr_frame_count = 0
        
        # Current device position for QR detection
        self.current_position = None  # [x, y, z]
        self.position_lock = threading.Lock()  # Lock for thread-safe access
        self.qr_detect_json_path = self.config.qr_detect_json_path
        
        self.translator = Translator(self.config.default_language)
        self.current_lang = self.config.default_language
        
        # Setup UI trước khi khởi tạo managers (cần log_panel và progress)
        self.setup_language_button()
        self.update_ui_texts()
        
        self.root.geometry(self.config.window_geometry)

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Workspace ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, **self.config.window_padding)

        # Setup UI components builder
        ui_callbacks = {
            'browse_file': self.browse_file,
            'start_mapping': self.start_mapping,
            'stop_process': self.stop_process,
            'start_upload': self.start_upload,
            'update_preview_mode': self.update_preview_mode,
            'clear_log': self.clear_log,
            'clear_qr_codes': self.clear_qr_codes,
            'update_design_type': self.update_design_type,
            'browse_design_file': self.browse_design_file,
        }
        self.ui_builder = UIComponentsBuilder(
            self.workspace,
            self.translator,
            self.config,
            ui_callbacks
        )
        
        # 1. Card Control (Start Mapping)
        self.ui_builder.setup_control_card(get_mac_callback=self._get_local_mac_no_colon)
        # Copy widget references từ builder
        self.control_card = self.ui_builder.control_card
        self.select_bag_label = self.ui_builder.select_bag_label
        self.bag_path_var = self.ui_builder.bag_path_var
        self.browse_btn = self.ui_builder.browse_btn
        self.vehicle_info_var = self.ui_builder.vehicle_info_var
        self.btn_start = self.ui_builder.btn_start
        self.btn_stop = self.ui_builder.btn_stop
        self.status_label = self.ui_builder.status_label

        # 2. Card Visualization (Preview / RViz) - Bao gồm cả progress và log section
        self.ui_builder.setup_preview_rviz_section(self.design_file_type)
        # Copy widget references từ builder
        self.preview_card = self.ui_builder.preview_card
        self.rviz_var = self.ui_builder.rviz_var
        self.rviz_check = self.ui_builder.rviz_check
        self.btn_upload = self.ui_builder.btn_upload
        self.progress = self.ui_builder.progress
        self.upload_stat = self.ui_builder.upload_stat
        self.comparison_frame = self.ui_builder.comparison_frame
        self.design_file_type_label = self.ui_builder.design_file_type_label
        self.design_type_var = self.ui_builder.design_type_var
        self.design_file_label = self.ui_builder.design_file_label
        self.design_path_var = self.ui_builder.design_path_var
        self.design_browse_btn = self.ui_builder.design_browse_btn
        self.comparison_status_label = self.ui_builder.comparison_status_label
        self.tunnel_entrance_x_var = self.ui_builder.tunnel_entrance_x_var
        self.tunnel_entrance_y_var = self.ui_builder.tunnel_entrance_y_var
        self.tunnel_entrance_z_var = self.ui_builder.tunnel_entrance_z_var
        self.qr_listbox = self.ui_builder.qr_listbox
        self.system_log_label = self.ui_builder.system_log_label
        self.clear_log_btn = self.ui_builder.clear_log_btn
        self.log_panel = self.ui_builder.log_panel
        
        # Initialize managers sau khi UI được setup
        self.logging_manager = LoggingManager(
            self.root, 
            self.log_panel, 
            self.translator, 
            self.config
        )
        self.progress_manager = ProgressManager(
            self.root,
            self.progress,
            self.upload_stat,
            self.translator,
            self.config
        )
        
        # Initialize language manager với UI updater callback
        self.language_manager = LanguageManager(
            self.root,
            self.translator,
            self.config,
            ui_updater=self.update_ui_texts
        )
        # Update language button reference
        if hasattr(self.language_manager, 'lang_button'):
            # Language button đã được setup trong setup_language_button, chỉ cần update reference
            pass
        
        # Initialize design map comparison
        self.design_comparison = DesignMapComparison(
            self.config,
            self.translator,
            self.add_log,
            ui_update_callback=None
        )
        self.design_comparison.design_file_path = self.design_file_path
        self.design_comparison.design_file_type = self.design_file_type
        
        # Initialize PCD processor
        self.pcd_processor = PCDProcessor(
            self.config,
            self.translator,
            self.add_log
        )
        
        # Initialize upload manager
        self.upload_manager = UploadManager(
            self.config,
            self.translator,
            self.add_log,
            self.set_progress,
            self.pcd_processor,
            self.design_comparison,
            qr_save_callback=self._save_qr_codes_to_json
        )
        
        # Initialize mapping core
        validation_funcs = {
            'validate_config_file': lambda path: validate_config_file(path, self.add_log, self.translator),
            'validate_config_parameters': lambda path: validate_config_parameters(path, self.add_log, self.translator),
            'check_system_resources': lambda: check_system_resources(self.add_log, self.translator, self.config),
            'validate_bag_file': lambda path, ws: validate_bag_file(path, ws, self.add_log, self.translator),
        }
        self.mapping_core = MappingCore(
            self.config,
            self.translator,
            self.add_log,
            validation_funcs,
            self.qr_scanner,
            self.qr_scan_enabled,
            self.qr_scan_frame_interval
        )
        # Sync initial state
        self.mapping_process = self.mapping_core.mapping_process
        self.bag_process = self.mapping_core.bag_process
        self.is_mapping_running = self.mapping_core.is_mapping_running
        self.is_bag_playing = self.mapping_core.is_bag_playing
        self.is_stopping = self.mapping_core.is_stopping
        
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
        """Update UI texts - delegate to UI builder"""
        self.root.title(self.translator.get('title.bag_mapping_system', 'Bag Mapping System'))
        
        # Update UI builder
        if hasattr(self, 'ui_builder'):
            self.ui_builder.update_ui_texts()
        
        # Note: Không gọi language_manager.change_language() ở đây vì nó sẽ gọi lại ui_updater()
        # và tạo vòng lặp đệ quy. language_manager.change_language() sẽ tự gọi ui_updater() khi cần.

    def setup_control_card(self):
        """Setup control card - delegate to UI builder"""
        # Đã được setup trong __init__ qua UIComponentsBuilder
        pass

    def setup_preview_rviz_section(self):
        """Setup preview/RViz section - delegate to UI builder"""
        # Đã được setup trong __init__ qua UIComponentsBuilder
        pass



    # --- Logic ---
    def cleanup_ros2_resources(self):
        """Dọn dẹp shared memory và các process ROS2 cũ để tránh lỗi FastRTPS SHM"""
        cleanup_ros2_resources(logger_callback=self.add_log)

    def update_preview_mode(self):
        self.use_rviz = self.rviz_var.get()
        if self.use_rviz:
            self.add_log(self.translator.get('log.config_rviz2_mode', 'CONFIG: Visualization mode set to RViz2.'))
        else:
            self.add_log(self.translator.get('log.config_internal_mode', 'CONFIG: Visualization mode set to Internal.'))

    def start_mapping(self):
        """Bắt đầu mapping - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            ui_callbacks = {
                'update_status_label': lambda text, color: self.status_label.config(text=text, foreground=color),
                'disable_start_button': lambda: [self.btn_start.config(state=tk.DISABLED), self.btn_stop.config(state=tk.NORMAL)],
                'enable_start_button': lambda: [self.btn_start.config(state=tk.NORMAL), self.btn_stop.config(state=tk.DISABLED)],
                'start_bag_playback': self.start_bag_playback,
                'start_qr_scanning': self.start_qr_scanning_subscriber,
            }
            
            success = self.mapping_core.start_mapping(
                self.bag_path,
                self.config_path,
                self.use_rviz,
                self.workspace_path,
                self.drive_ws_path,
                self.root,
                ui_callbacks
            )
            
            if success:
                # Sync process references
                self.mapping_process = self.mapping_core.mapping_process
                self.is_mapping_running = self.mapping_core.is_mapping_running
                # Disable START button, enable STOP button
                self.btn_start.config(state=tk.DISABLED)
                self.btn_stop.config(state=tk.NORMAL)
                # Auto-start bag playback
                self.start_bag_playback()
            else:
                self.cleanup_processes()

    def start_bag_playback(self):
        """Bắt đầu play bag - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            ui_callbacks = {
                'start_qr_scanning': self.start_qr_scanning_subscriber,
            }
            
            success = self.mapping_core.start_bag_playback(
                self.bag_path,
                self.bag_rate,
                self.workspace_path,
                self.drive_ws_path,
                self.root,
                ui_callbacks
            )
            
            if success:
                # Sync process references
                self.bag_process = self.mapping_core.bag_process
                self.is_bag_playing = self.mapping_core.is_bag_playing

    def check_pcd_files(self):
        """Kiểm tra xem có file PCD trong thư mục Log/PCD không"""
        return check_pcd_files(self.config.pcd_dir)

    def _get_local_mac_no_colon(self) -> str:
        """Lấy địa chỉ MAC (không dấu :) từ máy cục bộ."""
        return get_local_mac_no_colon()
    
    def start_upload(self):
        """Chạy full pipeline - delegate to upload manager"""
        if hasattr(self, 'upload_manager'):
            def btn_upload_callback(state='normal'):
                self.btn_upload.config(state=state)
            
            self.upload_manager.start_upload(
                self.vehicle_info_var,
                self.design_file_path,
                self.comparison_threshold,
                self.root,
                btn_upload_callback
            )
    
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
        """Cập nhật loại file thiết kế - delegate to design comparison"""
        if hasattr(self, 'design_comparison'):
            self.design_file_type = self.design_type_var.get()
            self.design_comparison.update_design_type(
                self.design_type_var,
                self.comparison_status_label
            )
            # Sync với design_comparison
            self.design_comparison.design_file_type = self.design_file_type

    def browse_design_file(self):
        """Chọn file bản thiết kế - delegate to design comparison"""
        if hasattr(self, 'design_comparison'):
            self.design_comparison.browse_design_file(
                self.design_path_var,
                self.workspace_path,
                self.comparison_status_label
            )
            # Sync với design_comparison
            self.design_file_path = self.design_comparison.design_file_path
            self.design_file_type = self.design_comparison.design_file_type

    def launch_rviz_cmd(self):
        self.add_log(self.translator.get('log.system_launching_rviz2', 'SYSTEM: Launching RViz2...'))

    def run_design_map_comparison(self, generated_pcd_path: Path):
        """So sánh bản đồ - delegate to design comparison module"""
        if hasattr(self, 'design_comparison'):
            # Lấy tọa độ đầu cửa hầm từ UI (mặc định luôn có)
            try:
                x = float(self.tunnel_entrance_x_var.get())
                y = float(self.tunnel_entrance_y_var.get())
                z = float(self.tunnel_entrance_z_var.get())
                tunnel_entrance_coords = (x, y, z)
            except (ValueError, AttributeError) as e:
                # Nếu không parse được hoặc không có UI, raise exception
                self.add_log(f"❌ Error getting tunnel entrance coordinates: {e}")
                raise ValueError("Tunnel entrance coordinates are required but not available")
            
            success, similarity = self.design_comparison.run_design_map_comparison(
                generated_pcd_path,
                self.comparison_status_label,
                self.root,
                tunnel_entrance_coords=tunnel_entrance_coords
            )
            # Sync comparison result
            self.comparison_result = self.design_comparison.comparison_result
            return success, similarity
        return True, 100.0

    def clear_log(self):
        """Clear log panel - delegate to logging manager"""
        if hasattr(self, 'logging_manager'):
            self.logging_manager.clear_log()
        else:
            # Fallback nếu manager chưa được khởi tạo
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
        """Thêm log message - delegate to logging manager"""
        if hasattr(self, 'logging_manager'):
            self.logging_manager.add_log(message)
        else:
            # Fallback nếu manager chưa được khởi tạo
            time_str = datetime.now().strftime("%H:%M:%S")
            log_entry = f"[{time_str}] {message}\n"
            try:
                self.log_panel.config(state=tk.NORMAL)
                self.log_panel.insert(tk.END, log_entry)
                self.log_panel.see(tk.END)
                self.log_panel.config(state=tk.DISABLED)
            except:
                print(log_entry.strip())
    
    def add_log_success(self, message_key, **kwargs):
        """Thêm success log message - delegate to logging manager"""
        if hasattr(self, 'logging_manager'):
            self.logging_manager.add_log_success(message_key, **kwargs)
        else:
            msg = self.translator.get(message_key, message_key)
            for key, value in kwargs.items():
                msg = msg.replace(f'{{{key}}}', str(value))
            self.add_log(f"✅ {msg}")
    
    def add_log_warning(self, message_key, **kwargs):
        """Thêm warning log message - delegate to logging manager"""
        if hasattr(self, 'logging_manager'):
            self.logging_manager.add_log_warning(message_key, **kwargs)
        else:
            msg = self.translator.get(message_key, message_key)
            for key, value in kwargs.items():
                msg = msg.replace(f'{{{key}}}', str(value))
            self.add_log(f"⚠️ {msg}")
    
    def set_progress(self, value):
        """Set progress bar - delegate to progress manager"""
        if hasattr(self, 'progress_manager'):
            self.progress_manager.set_progress(value)
        else:
            # Fallback nếu manager chưa được khởi tạo
            try:
                self.progress['value'] = value
            except:
                pass

    def set_default_config(self):
        """Set default config - sử dụng utility function"""
        class ConfigPathHolder:
            def __init__(self):
                self.value = None
        
        holder = ConfigPathHolder()
        set_default_config(
            self.workspace_path,
            holder,
            logger_callback=self.add_log,
            translator=self.translator
        )
        if holder.value:
            self.config_path = holder.value
        
        # Load design file config sau khi setup UI
        self._load_design_file_config()
    
    def _load_design_file_config(self):
        """Tải đường dẫn design file - delegate to design comparison"""
        if hasattr(self, 'design_comparison'):
            self.design_comparison.load_design_file_config(
                self.design_path_var,
                self.design_type_var,
                self.comparison_status_label
            )
            # Sync với design_comparison
            self.design_file_path = self.design_comparison.design_file_path
            self.design_file_type = self.design_comparison.design_file_type
    
    def _save_design_file_config(self, file_path, file_type):
        """Lưu đường dẫn design file - delegate to design comparison"""
        if hasattr(self, 'design_comparison'):
            self.design_comparison._save_design_file_config(file_path, file_type)
    
    def _validate_config_file(self, config_path_obj):
        """Validate YAML config file - delegate to validation module"""
        return validate_config_file(
            config_path_obj,
            logger_callback=self.add_log,
            translator=self.translator
        )
    
    def _validate_config_parameters(self, config_path_obj):
        """Validate config parameters - delegate to validation module"""
        return validate_config_parameters(
            config_path_obj,
            logger_callback=self.add_log,
            translator=self.translator
        )
    
    def _check_system_resources(self):
        """Kiểm tra system resources - delegate to validation module"""
        return check_system_resources(
            logger_callback=self.add_log,
            translator=self.translator,
            config=self.config
        )
    
    def _validate_bag_file(self):
        """Kiểm tra bag file - delegate to validation module"""
        return validate_bag_file(
            self.bag_path,
            self.workspace_path,
            logger_callback=self.add_log,
            translator=self.translator
        )
    
    def _wait_for_process_health(self, max_wait_time=10):
        """Đợi process khởi động và kiểm tra health - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            # Sync state trước khi check
            self.mapping_process = self.mapping_core.mapping_process
            return self.mapping_core._wait_for_process_health(max_wait_time)
        return False

    def monitor_mapping_process(self):
        """Theo dõi mapping process - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            ui_callbacks = {
                'update_status_label': lambda text, color: self.status_label.config(text=text, foreground=color),
                'enable_start_button': lambda: [self.btn_start.config(state=tk.NORMAL), self.btn_stop.config(state=tk.DISABLED)],
            }
            self.mapping_core.monitor_mapping_process(self.root, ui_callbacks)
            # Sync state
            self.is_mapping_running = self.mapping_core.is_mapping_running
            self.mapping_process = self.mapping_core.mapping_process
    
    def monitor_bag_process(self):
        """Theo dõi bag process - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            self.mapping_core.monitor_bag_process(self.root)
            # Sync state
            self.is_bag_playing = self.mapping_core.is_bag_playing
            self.bag_process = self.mapping_core.bag_process
    
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
        """Dọn dẹp processes - delegate to mapping core"""
        # Dừng QR scanning subscriber trước
        self.stop_qr_scanning_subscriber()
        
        if hasattr(self, 'mapping_core'):
            ui_callbacks = {
                'update_status_label': lambda text, color: self.status_label.config(text=text, foreground=color),
                'enable_start_button': lambda: [self.btn_start.config(state=tk.NORMAL), self.btn_stop.config(state=tk.DISABLED)],
            }
            self.mapping_core.cleanup_processes(self.root, ui_callbacks)
            # Sync state
            self.mapping_process = self.mapping_core.mapping_process
            self.bag_process = self.mapping_core.bag_process
            self.is_mapping_running = self.mapping_core.is_mapping_running
            self.is_bag_playing = self.mapping_core.is_bag_playing
            self.is_stopping = self.mapping_core.is_stopping
        
        # Enable START button, disable STOP button khi cleanup
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        
        # Kiểm tra PCD files và cập nhật status
        if self.check_pcd_files():
            self.upload_stat.config(text="Ready to upload")
            self.add_log(self.translator.get('log.pcd_detected', '✅ PCD files detected, ready to upload'))
        else:
            self.upload_stat.config(text="Waiting for PCD files...")
    
    # --- PCD Processing Functions ---
    def merge_pcd_files(self, silent=False):
        """Gộp PCD files - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.merge_pcd_files(silent=silent)
        return False
    
    def _run_hba_core(self):
        """HBA optimization - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.run_hba_core()
        return False
    
    def _compute_tunnel_direction(self):
        """Tính tunnel direction - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.compute_tunnel_direction()
        return False
    
    def _rotate_pcd_to_positive_x(self):
        """Xoay PCD về +x - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.rotate_pcd_to_positive_x()
        return False
    
    def _run_sc_core(self):
        """ScanContext tiling - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.run_sc_core()
        return False
    
    def _generate_floorplan(self):
        """Generate floorplan - delegate to PCD processor"""
        if hasattr(self, 'pcd_processor'):
            return self.pcd_processor.generate_floorplan()
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
    
    def _zip_map_folders(self, map_name: str = None, vehicle_id: str = None):
        """Zip map folders - delegate to upload manager"""
        if hasattr(self, 'upload_manager'):
            return self.upload_manager.zip_map_folders(map_name, vehicle_id)
        return None
    
    def _upload_map_to_backend(self, zip_path: Path, map_name: str, vehicle_id: str):
        """Upload map to backend - delegate to upload manager"""
        if hasattr(self, 'upload_manager'):
            return self.upload_manager.upload_map_to_backend(zip_path, map_name, vehicle_id)
        return False, "Upload manager not available"
    
    def _save_qr_codes_to_json(self):
        """Save QR codes to JSON - wrapper method"""
        return self.save_qr_detect_json()
    
    def stop_process(self):
        """Dừng process - delegate to mapping core"""
        if hasattr(self, 'mapping_core'):
            self.mapping_core.is_stopping = True
        self.cleanup_processes()
        # Enable START button, disable STOP button
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.set_progress(0)
        self.add_log(self.translator.get('log.process_terminated', 'PROCESS: Mapping and Upload terminated.'))

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