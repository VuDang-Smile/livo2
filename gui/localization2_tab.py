#!/usr/bin/env python3
"""
Localization2 Tab Module
Tab để điều khiển Localization2 system với FAST-LIVO2
"""

import threading
import subprocess
from pathlib import Path
from datetime import datetime
import os
import platform
import signal
import sys
import time
import tempfile
import queue
import re
import shutil
from functools import partial

# Multiprocessing for parallel downsampling
try:
    from multiprocessing import Pool, cpu_count
    HAS_MULTIPROCESSING = True
except ImportError:
    HAS_MULTIPROCESSING = False
    cpu_count = lambda: 1

# Open3D và scipy imports (optional, for map loading)
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    from scipy.spatial.transform import Rotation
    from scipy.spatial import cKDTree
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    cKDTree = None

# ROS2 imports with graceful fallback
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    print("⚠️  ROS2 not available. Localization2 tab will be limited.")
    ROS2_AVAILABLE = False
    # Create dummy classes for when ROS2 is not available
    class Node:
        def __init__(self, *args, **kwargs):
            pass
    class MultiThreadedExecutor:
        def __init__(self, *args, **kwargs):
            pass
    class Odometry:
        pass

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class LocalizationListenerNode(Node):
    """ROS2 node to listen to localization odometry topic"""
    
    def __init__(self, localization_tab):
        if not ROS2_AVAILABLE:
            return
        super().__init__('localization_listener')
        self.localization_tab = localization_tab
        
        # Subscribe to /Odometry (default topic from localization2)
        try:
            self.subscription = self.create_subscription(
                Odometry,
                '/Odometry',
                self.localization_callback,
                10  # QoS depth
            )
            self.get_logger().info('✅ Subscribed to /Odometry topic (localization2 default)')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to /Odometry: {e}')
    
    def localization_callback(self, msg):
        """Callback for /Odometry topic messages"""
        try:
            # Pass the message to the Localization2Tab for processing
            self.localization_tab.update_localization_data(msg)
        except Exception as e:
            if ROS2_AVAILABLE:
                self.get_logger().error(f'Error in localization callback: {e}')


class Localization2Tab(ttk.Frame):
    """Tab cho Localization2 Control"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.project_root = Path(__file__).parent.parent
        self.workspace_path = self.project_root / "ws"
        self.script_path = self.project_root / "scripts" / "start_localization2.sh"
        self.log_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log"
        
        # Processes
        self.localization_process = None
        
        # ROS2
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_ros_running = False
        
        # State
        self.is_localization_running = False
        self.selected_map_dir = None
        self.map_info_loaded = False  # Flag để biết đã load map info chưa
        
        # Map size limits (để tránh crash RViz và đảm bảo RViz mượt)
        self.max_map_points = 50_000_000  # 50M points - giới hạn an toàn
        self.warning_map_points = 5_000_000  # 5M points - tự động optimize (giảm từ 8M)
        self.safe_map_points = 2_000_000  # 2M points - luôn optimize (giảm từ 3M)
        self.downsample_voxel_size = 0.22  # Voxel size cho downsample (m) - tăng để downsample nhiều hơn (0.18 -> 0.22)
        
        # Per-file PCD limits (để RViz mượt hơn - giảm mạnh để downsample nhiều hơn)
        self.max_points_per_file = 500_000  # 500K points per file - tự động downsample nếu vượt (giảm từ 800K)
        self.file_downsample_voxel_size = 0.15  # Voxel size cơ bản cho auto-downsample file (m) - tăng (0.12 -> 0.15)
        self.target_points_per_file = 300_000  # Target points sau downsample (~300K để RViz rất mượt, giảm từ 500K)
        
        # Localization data
        self.position_var = None
        self.orientation_var = None
        self.velocity_var = None
        self.timestamp_var = None
        
        # Localization data storage (from Recorder)
        self.current_localization = None
        self.localization_history = []
        self.max_history_size = 100
        
        # Map bounds for drift detection (from Recorder)
        self.map_bounds = None  # {'x': {'min': float, 'max': float}, 'y': {'min': float, 'max': float}, 'z': {'min': float, 'max': float}}
        self.map_bounds_original = None  # Original bounds without expansion (for strict drift detection)
        self.out_of_bounds_detected = False
        self.last_restart_time = 0
        self.restart_cooldown = 10.0  # Minimum seconds between restarts
        self.out_of_bounds_margin = 3.0  # Margin in meters beyond bounds before restarting
        self.out_of_bounds_count = 0  # Count consecutive out-of-bounds detections
        self.out_of_bounds_threshold = 1  # Restart after N consecutive detections
        self.is_restarting = False  # Flag to prevent multiple simultaneous restarts
        self.restart_lock = threading.Lock()  # Lock for thread-safe restart operations
        self.localization_start_time = 0  # Track when localization started
        self.initialization_grace_period = 10.0  # Seconds after start to ignore drift
        self.auto_restart_on_crash = True  # Auto-restart when process crashes
        self.crash_restart_count = 0  # Count consecutive crash restarts
        self.max_crash_restarts = 5  # Maximum consecutive crash restarts
        self.user_stopped = False  # Flag to track if user manually stopped
        
        # Global localization monitoring (from Recorder)
        self.last_global_localization_time = 0  # Track when global localization last occurred
        self.global_localization_grace_period = 3.0  # Seconds after global localization to allow position adjustment
        self.drift_threshold_after_global_loc = 3.0  # Larger threshold after global localization
        self.global_localization_fail_count = 0  # Count consecutive global localization failures
        self.global_localization_fail_threshold = 3  # Restart after N consecutive failures
        
        # Drift detection within bounds (from Recorder)
        self.restart_due_to_drift = False  # Flag to track if last restart was due to drift
        self.last_drift_restart_time = 0  # Timestamp of last restart due to drift
        self.drift_restart_window = 60.0  # Seconds after drift restart to use stricter detection
        self.last_position = None  # {'x': float, 'y': float, 'z': float, 'timestamp': float}
        self.position_jump_threshold = 10.0  # Maximum reasonable distance between updates (meters)
        self.max_reasonable_speed = 5.0  # Initial maximum reasonable speed (m/s)
        self.position_jump_count = 0  # Count consecutive position jumps
        self.position_jump_threshold_count = 3  # Restart after N consecutive jumps
        # Speed history for adaptive threshold calculation
        self.speed_history = []  # List of recent speeds for adaptive threshold
        self.speed_history_max_size = 100  # Keep last 100 speed measurements
        self.speed_threshold_multiplier = 3.0  # Threshold = mean + multiplier * std
        self.min_speed_samples = 10  # Minimum samples before using adaptive threshold
        self.adaptive_speed_threshold_enabled = True  # Enable adaptive speed threshold
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Localization2"""
        
        # Main container với scrollbar
        main_container = ttk.Frame(self)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Canvas và scrollbar
        self.canvas = tk.Canvas(main_container, highlightthickness=0)
        scrollbar = ttk.Scrollbar(main_container, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = ttk.Frame(self.canvas)
        
        # Configure scrollable frame
        def configure_scroll_region(event=None):
            self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        
        self.scrollable_frame.bind("<Configure>", configure_scroll_region)
        
        # Create window in canvas
        self.canvas_window = self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        
        # Configure canvas scrolling - ensure width matches canvas
        def configure_canvas_width(event):
            canvas_width = event.width
            self.canvas.itemconfig(self.canvas_window, width=canvas_width)
            # Update scroll region after width change
            self.after_idle(lambda: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        
        self.canvas.bind('<Configure>', configure_canvas_width)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        
        # Mouse wheel scrolling
        def on_mousewheel(event):
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        
        self.canvas.bind_all("<MouseWheel>", on_mousewheel)
        
        # Pack canvas and scrollbar
        self.canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Configure main container
        main_container.columnconfigure(0, weight=1)
        main_container.rowconfigure(0, weight=1)
        
        # Store canvas reference for later updates
        self.main_canvas = self.canvas
        
        # Map Selection Section
        self.setup_map_selection()
        
        # Control Section
        self.setup_control()
        
        # Localization Data Section
        self.setup_localization_data()
        
        # Status and Log Section
        self.setup_status_log()
        
        # Configure grid weights
        self.scrollable_frame.columnconfigure(0, weight=1)
        
        # Update canvas scroll region after all widgets are created
        self.after(100, lambda: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
    
    def setup_map_selection(self):
        """Setup map selection section"""
        map_frame = ttk.LabelFrame(self.scrollable_frame, text="Map Selection", padding="10")
        map_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Description
        desc_label = ttk.Label(map_frame, text="Select a map directory from FAST-LIVO2 Log", 
                              font=('Arial', 9), foreground='gray')
        desc_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Selected map display
        self.map_dir_var = tk.StringVar(value="No map selected")
        self.map_dir_label = ttk.Label(map_frame, text="Selected map:", font=('Arial', 10))
        self.map_dir_label.grid(row=1, column=0, sticky=tk.W, pady=(0, 5))
        
        self.map_dir_display = ttk.Label(map_frame, textvariable=self.map_dir_var, 
                                         font=('Arial', 9), foreground='blue', wraplength=600)
        self.map_dir_display.grid(row=2, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Buttons
        buttons_frame = ttk.Frame(map_frame)
        buttons_frame.grid(row=3, column=0, columnspan=2, pady=(0, 5))
        
        self.select_map_button = ttk.Button(buttons_frame, text="📁 Select Map Directory", 
                                           command=self.select_map_directory)
        self.select_map_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.select_latest_button = ttk.Button(buttons_frame, text="⬇️ Select Latest Map", 
                                               command=self.select_latest_map)
        self.select_latest_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.load_map_info_button = ttk.Button(buttons_frame, text="📊 Load Map Info", 
                                               command=self.load_map_info)
        self.load_map_info_button.pack(side=tk.LEFT)
        
        # Map info display
        self.map_info_var = tk.StringVar(value="No map info loaded - Click 'Load Map Info' to analyze")
        self.map_info_label = ttk.Label(map_frame, textvariable=self.map_info_var, 
                                       font=('Arial', 9), foreground='green', wraplength=600, justify=tk.LEFT)
        self.map_info_label.grid(row=4, column=0, columnspan=2, sticky=tk.W, pady=(10, 5))
        
        # Configure grid weights
        map_frame.columnconfigure(0, weight=1)
    
    def setup_control(self):
        """Setup control section"""
        control_frame = ttk.LabelFrame(self.scrollable_frame, text="Localization2 Control", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Description
        desc_label = ttk.Label(control_frame, text="FAST_LIO_LOCALIZATION2 system for localization", 
                              font=('Arial', 9), foreground='gray')
        desc_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Status
        self.status_var = tk.StringVar(value="Stopped")
        self.status_label = ttk.Label(control_frame, textvariable=self.status_var, 
                                      font=('Arial', 10, 'bold'))
        self.status_label.grid(row=1, column=0, columnspan=2, pady=(0, 10))
        
        # Control buttons
        buttons_frame = ttk.Frame(control_frame)
        buttons_frame.grid(row=2, column=0, columnspan=2, pady=(0, 5))
        
        self.start_button = ttk.Button(buttons_frame, text="🚀 Start Localization2", 
                                      command=self.start_localization2)
        self.start_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.stop_button = ttk.Button(buttons_frame, text="⏹️ Stop Localization2", 
                                     command=self.stop_localization2, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT)
        
        # Configure grid weights
        control_frame.columnconfigure(0, weight=1)
    
    def setup_localization_data(self):
        """Setup localization data display section"""
        data_frame = ttk.LabelFrame(self.scrollable_frame, text="Localization Data (/Odometry)", padding="10")
        data_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # Description
        desc_label = ttk.Label(data_frame, text="Real-time pose data from /Odometry topic", 
                              font=('Arial', 9), foreground='gray')
        desc_label.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0, 10))
        
        # Current pose display
        current_frame = ttk.Frame(data_frame)
        current_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # Position
        ttk.Label(current_frame, text="Position (x, y, z):", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, padx=(0, 10))
        self.position_var = tk.StringVar(value="N/A")
        ttk.Label(current_frame, textvariable=self.position_var, font=('Courier', 9), foreground='blue').grid(row=0, column=1, sticky=tk.W)
        
        # Orientation
        ttk.Label(current_frame, text="Orientation (x, y, z, w):", font=('Arial', 10, 'bold')).grid(row=1, column=0, sticky=tk.W, padx=(0, 10))
        self.orientation_var = tk.StringVar(value="N/A")
        ttk.Label(current_frame, textvariable=self.orientation_var, font=('Courier', 9), foreground='blue').grid(row=1, column=1, sticky=tk.W)
        
        # Velocity
        ttk.Label(current_frame, text="Velocity (x, y, z):", font=('Arial', 10, 'bold')).grid(row=2, column=0, sticky=tk.W, padx=(0, 10))
        self.velocity_var = tk.StringVar(value="N/A")
        ttk.Label(current_frame, textvariable=self.velocity_var, font=('Courier', 9), foreground='blue').grid(row=2, column=1, sticky=tk.W)
        
        # Timestamp
        ttk.Label(current_frame, text="Last Update:", font=('Arial', 10, 'bold')).grid(row=3, column=0, sticky=tk.W, padx=(0, 10))
        self.timestamp_var = tk.StringVar(value="N/A")
        ttk.Label(current_frame, textvariable=self.timestamp_var, font=('Courier', 9), foreground='green').grid(row=3, column=1, sticky=tk.W)
        
        # Configure grid weights
        data_frame.columnconfigure(0, weight=1)
        current_frame.columnconfigure(1, weight=1)
    
    def setup_status_log(self):
        """Setup status and log section"""
        log_frame = ttk.LabelFrame(self.scrollable_frame, text="Status & Log", padding="10")
        log_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 10))
        
        # Create text widget with scrollbar
        self.log_text = tk.Text(log_frame, height=15, wrap=tk.WORD, font=('Courier', 9))
        log_scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        log_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        # Clear log button
        self.clear_log_button = ttk.Button(log_frame, text="🗑️ Clear Log", 
                                          command=self.clear_log)
        self.clear_log_button.grid(row=1, column=0, sticky=tk.W, pady=(5, 0))
        
        # Configure grid weights
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Update canvas scroll region after adding log frame
        self.after_idle(lambda: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all")))
    
    def find_latest_map_directory(self):
        """Tìm map directory mới nhất trong Log/ có dữ liệu"""
        if not self.log_dir.exists():
            return None
        
        map_dirs = sorted(self.log_dir.glob("map_*"), key=lambda x: x.name, reverse=True)
        
        # Tìm map đầu tiên có dữ liệu (có PCD files hoặc pose.json có nội dung)
        for map_dir in map_dirs:
            if self.is_map_valid(map_dir):
                return map_dir
        
        return None
    
    def _check_pcd_format(self, pcd_file_path):
        """
        Kiểm tra format của PCD file (có RGB hay không)
        Returns: True nếu có RGB, False nếu không
        """
        try:
            # Đọc header của PCD file để kiểm tra format
            with open(pcd_file_path, 'rb') as f:
                header = f.read(2048).decode('utf-8', errors='ignore')
                # Kiểm tra xem có FIELDS chứa rgb hoặc rgba không
                if 'FIELDS' in header:
                    fields_line = [line for line in header.split('\n') if 'FIELDS' in line]
                    if fields_line:
                        fields = fields_line[0].upper()
                        # Kiểm tra có rgb hoặc rgba trong FIELDS
                        if 'RGB' in fields or 'RGBA' in fields:
                            return True
        except:
            pass
        return False
    
    def _read_pcd_with_auto_downsample(self, pcd_file_path, preserve_colors=True):
        """
        Đọc PCD file với tự động downsample nếu quá lớn
        Đảm bảo đọc đúng dữ liệu đã quét (colors, normals nếu có)
        Voxel size được tính động: càng nhiều points → downsample càng nhiều
        
        Args:
            pcd_file_path: Path đến PCD file
            preserve_colors: Có giữ colors không (default: True)
            
        Returns:
            tuple: (point_cloud, was_downsampled, voxel_size_used)
                - point_cloud: Point cloud đã được xử lý (downsample nếu cần)
                - was_downsampled: True nếu đã downsample, False nếu không
                - voxel_size_used: Voxel size đã sử dụng (None nếu không downsample)
        """
        if not HAS_OPEN3D:
            return None, False
        
        try:
            # Kiểm tra format PCD file trước khi đọc
            has_rgb_in_file = self._check_pcd_format(pcd_file_path)
            
            # Đọc PCD file (open3d tự động detect format và preserve colors/normals)
            # Không cần chỉ định format='auto' vì đó là default
            pcd = o3d.io.read_point_cloud(str(pcd_file_path))
            
            if len(pcd.points) == 0:
                return None, False, None
            
            num_points = len(pcd.points)
            was_downsampled = False
            voxel_size_used = None
            
            # Kiểm tra xem colors có được đọc đúng không
            has_colors_after_read = pcd.has_colors()
            if has_rgb_in_file and not has_colors_after_read:
                # File có RGB nhưng không đọc được - có thể là format issue
                # Thử đọc lại với format cụ thể
                try:
                    # Thử đọc với format PCD (không chỉ định format cụ thể)
                    pcd = o3d.io.read_point_cloud(str(pcd_file_path))
                    has_colors_after_read = pcd.has_colors()
                except:
                    pass
            
            # Kiểm tra nếu file quá lớn, tự động downsample
            if num_points > self.max_points_per_file:
                import numpy as np
                
                # Kiểm tra và preserve colors
                has_colors = pcd.has_colors()
                has_normals = pcd.has_normals()
                
                # Lưu colors gốc trước khi xử lý (để đảm bảo không mất dữ liệu)
                original_colors = None
                if has_colors:
                    original_colors = np.asarray(pcd.colors).copy()
                    # Open3D lưu colors trong range 0-1, nhưng PCD file có thể là 0-255
                    # Kiểm tra xem colors có trong range nào
                    colors = np.asarray(pcd.colors)
                    # Nếu colors > 1.0, có thể là đã được normalize sai hoặc format khác
                    # Chỉ normalize nếu thực sự cần (colors > 1.0 và có vẻ như là 0-255)
                    if colors.max() > 1.0 and colors.max() <= 255.0:
                        # Normalize từ 0-255 về 0-1 (open3d format)
                        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
                    # Nếu colors đã trong range 0-1, giữ nguyên
                
                # Tính toán voxel size động dựa trên số points
                # Càng nhiều points thì downsample càng nhiều (voxel size lớn hơn)
                # Voxel downsample giảm points theo volume (tỷ lệ với voxel_size^3)
                # Công thức: voxel_size = base_size * (num_points / target_points)^(1/3)
                reduction_ratio = num_points / self.target_points_per_file
                # Cube root để tính voxel size (vì volume tỷ lệ với size^3)
                dynamic_voxel_size = self.file_downsample_voxel_size * (reduction_ratio ** (1.0/3.0))
                
                # Giới hạn voxel size trong khoảng hợp lý (0.05m - 0.5m)
                dynamic_voxel_size = max(0.05, min(0.5, dynamic_voxel_size))
                voxel_size_used = dynamic_voxel_size
                
                # Downsample với voxel size động (càng nhiều points → voxel size càng lớn → downsample càng nhiều)
                if preserve_colors:
                    # Voxel downsample tự động preserve colors
                    pcd = pcd.voxel_down_sample(voxel_size=dynamic_voxel_size)
                else:
                    pcd = pcd.voxel_down_sample(voxel_size=dynamic_voxel_size)
                
                # Đảm bảo colors được preserve sau downsample
                if has_colors and not pcd.has_colors():
                    # Nếu mất colors sau downsample (hiếm khi xảy ra với open3d)
                    # Khôi phục từ original_colors đã lưu
                    if HAS_SCIPY and cKDTree and original_colors is not None:
                        # Đọc lại file gốc để lấy points gốc
                        original_pcd = o3d.io.read_point_cloud(str(pcd_file_path), format='auto')
                        original_points = np.asarray(original_pcd.points)
                        downsampled_points = np.asarray(pcd.points)
                        
                        # Tìm colors gần nhất từ original
                        tree = cKDTree(original_points)
                        _, indices = tree.query(downsampled_points, k=1)
                        downsampled_colors = original_colors[indices]
                        pcd.colors = o3d.utility.Vector3dVector(downsampled_colors)
                    elif original_colors is not None:
                        # Fallback: sử dụng colors trung bình nếu không có scipy
                        # (không lý tưởng nhưng tốt hơn không có colors)
                        import numpy as np
                        avg_color = np.mean(original_colors, axis=0)
                        downsampled_colors = np.tile(avg_color, (len(pcd.points), 1))
                        pcd.colors = o3d.utility.Vector3dVector(downsampled_colors)
                
                was_downsampled = True
                
            return pcd, was_downsampled, voxel_size_used
            
        except Exception as e:
            import traceback
            # Log lỗi chi tiết để debug
            print(f"Error reading PCD file {pcd_file_path}: {e}")
            print(traceback.format_exc())
            return None, False, None
    
    def is_map_valid(self, map_dir):
        """Kiểm tra map directory có dữ liệu hợp lệ không (giống pcd_viewer_tab.py)"""
        if not map_dir.exists():
            return False
        
        # Kiểm tra có pose.json
        pose_file = map_dir / "pose.json"
        if not pose_file.exists():
            return False
        
        # Kiểm tra pose.json có nội dung (không rỗng)
        try:
            if pose_file.stat().st_size == 0:
                return False
            
            # Đọc và validate pose.json có ít nhất 1 pose hợp lệ (giống pcd_viewer_tab.py)
            poses_found = 0
            with open(pose_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    # Format: tx ty tz w x y z (7 values)
                    if len(parts) >= 7:
                        try:
                            # Validate có thể parse được
                            tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2])
                            w, x, y, z = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                            poses_found += 1
                            # Chỉ cần 1 pose hợp lệ là đủ
                            if poses_found >= 1:
                                break
                        except ValueError:
                            continue
            
            if poses_found == 0:
                return False
                
        except Exception:
            return False
        
        # Kiểm tra có pcd directory và có ít nhất 1 PCD file
        pcd_dir = map_dir / "pcd"
        if pcd_dir.exists():
            pcd_files = list(pcd_dir.glob("*.pcd"))
            if len(pcd_files) > 0:
                return True
        
        # Nếu không có PCD files nhưng pose.json có nội dung hợp lệ, vẫn coi là valid
        # (có thể là single file map hoặc map đang được tạo)
        return True
    
    def select_latest_map(self):
        """Chọn map mới nhất có dữ liệu"""
        latest_map = self.find_latest_map_directory()
        if latest_map:
            self.selected_map_dir = str(latest_map)
            self.map_dir_var.set(self.selected_map_dir)
            
            # Đọc và hiển thị thông tin chi tiết về map (giống pcd_viewer_tab.py)
            try:
                pose_file = latest_map / "pose.json"
                poses_count = 0
                if pose_file.exists():
                    with open(pose_file, 'r') as f:
                        for line in f:
                            line = line.strip()
                            if not line:
                                continue
                            parts = line.split()
                            if len(parts) >= 7:
                                try:
                                    float(parts[0]), float(parts[1]), float(parts[2])
                                    float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                                    poses_count += 1
                                except ValueError:
                                    continue
                
                pcd_count = len(list((latest_map / "pcd").glob("*.pcd"))) if (latest_map / "pcd").exists() else 0
                self.log_message(f"✅ Selected latest valid map: {latest_map.name}")
                self.log_message(f"   📄 {poses_count} poses in pose.json")
                if pcd_count > 0:
                    self.log_message(f"   📦 {pcd_count} PCD tiles found")
                else:
                    self.log_message(f"   ⚠️ No PCD tiles found (using pose.json only)")
                
                # Reset map info khi chọn map mới
                self.map_info_loaded = False
                self.map_info_var.set(f"Map: {latest_map.name} | {poses_count} poses | {pcd_count} tiles - Click 'Load Map Info' for details")
            except Exception as e:
                self.log_message(f"✅ Selected latest valid map: {latest_map.name}")
                self.log_message(f"   ⚠️ Could not read map details: {e}")
                self.map_info_loaded = False
                self.map_info_var.set("No map info loaded - Click 'Load Map Info' to analyze")
        else:
            messagebox.showwarning("Warning", "No valid map directories found in Log/\n(Maps must have pose.json with content)")
            self.log_message("⚠️ No valid map directories found")
    
    def select_map_directory(self):
        """Chọn map directory"""
        initial_dir = str(self.log_dir) if self.log_dir.exists() else str(self.project_root)
        selected = filedialog.askdirectory(
            title="Select Map Directory",
            initialdir=initial_dir
        )
        
        if selected:
            # Check if it's a valid map directory
            map_path = Path(selected)
            if (map_path / "pose.json").exists() or (map_path / "pcd").exists():
                self.selected_map_dir = selected
                self.map_dir_var.set(self.selected_map_dir)
                self.log_message(f"✅ Selected map: {map_path.name}")
                # Reset map info khi chọn map mới
                self.map_info_loaded = False
                self.map_info_var.set("No map info loaded - Click 'Load Map Info' to analyze")
            else:
                messagebox.showwarning("Warning", "Selected directory doesn't appear to be a valid map directory.\nLooking for pose.json or pcd/ subdirectory.")
                self.log_message("⚠️ Invalid map directory selected")
    
    def log_message(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
    
    def load_map_info(self):
        """Load và hiển thị thông tin chi tiết về map (giống pcd_viewer_tab.py)"""
        if not self.selected_map_dir:
            messagebox.showwarning("Warning", "Please select a map directory first!")
            return
        
        map_path = Path(self.selected_map_dir)
        if not map_path.exists():
            messagebox.showerror("Error", "Selected map directory does not exist!")
            return
        
        # Load map info trong thread riêng để không block UI
        threading.Thread(target=self._load_map_info_worker, args=(str(map_path),), daemon=True).start()
    
    def _load_map_info_worker(self, map_dir_path):
        """Worker thread để load map info"""
        try:
            map_info = self.load_map_tiles_info(map_dir_path)
            if map_info:
                # Update UI trong main thread
                def update_ui():
                    self.map_info_var.set(
                        f"Map: {map_info['name']}\n"
                        f"Tiles: {map_info['loaded_count']}/{map_info['total_poses']}\n"
                        f"Total Points: {map_info['total_points']:,}\n"
                        f"Failed: {map_info['failed_count']}"
                    )
                    self.map_info_loaded = True
                self.after(0, update_ui)
        except Exception as e:
            self.log_message(f"❌ Error loading map info: {e}")
            def update_error():
                self.map_info_var.set(f"Error loading map info: {e}")
            self.after(0, update_error)
    
    def load_map_tiles_info(self, map_dir_path):
        """Load và tính toán thông tin về map tiles (tích hợp từ Recorder project)
        
        Cách đọc tiles PCD (từ Recorder):
        1. Đọc pose.json để lấy danh sách poses và file_index
        2. Sử dụng glob để tìm tất cả PCD files trong pcd/ directory (validation)
        3. Đọc từng PCD file theo file_index: pcd_dir / f"{file_index}.pcd"
        4. Validate số lượng PCD files khớp với số poses
        """
        if not HAS_OPEN3D:
            self.log_message("⚠️ open3d không được cài đặt. Không thể load map info chi tiết.")
            self.log_message("💡 Cài đặt: pip3 install open3d")
            return None
        
        map_dir = Path(map_dir_path)
        pose_file = map_dir / "pose.json"
        pcd_dir = map_dir / "pcd"
        
        if not pose_file.exists():
            self.log_message(f"❌ Không tìm thấy pose.json tại: {pose_file}")
            return None
        
        if not pcd_dir.exists():
            self.log_message(f"❌ Không tìm thấy thư mục pcd/ tại: {pcd_dir}")
            return None
        
        self.log_message(f"📂 Đang load map info từ: {map_dir.name}")
        self.log_message(f"📄 Đọc pose.json...")
        
        # Đọc pose.json
        poses = []
        try:
            with open(pose_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 7:
                        try:
                            tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2])
                            w, x, y, z = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                            poses.append((tx, ty, tz, w, x, y, z))
                        except ValueError:
                            continue
        except Exception as e:
            self.log_message(f"❌ Lỗi khi đọc pose.json: {e}")
            return None
        
        if not poses:
            self.log_message("❌ Không có pose nào trong pose.json")
            return None
        
        self.log_message(f"📊 Tìm thấy {len(poses)} poses")
        
        # Validation: Sử dụng glob để tìm tất cả PCD files (cách của Recorder)
        pcd_files_glob = list(pcd_dir.glob("*.pcd"))
        pcd_files_count = len(pcd_files_glob)
        self.log_message(f"📦 Tìm thấy {pcd_files_count} file PCD trong thư mục pcd/ (glob)")
        
        # Cảnh báo nếu số lượng không khớp
        if pcd_files_count != len(poses):
            self.log_message(f"⚠️  Số lượng PCD files ({pcd_files_count}) không khớp với số poses ({len(poses)})")
            self.log_message(f"   Sẽ đọc theo file_index từ pose.json (0.pcd, 1.pcd, ...)")
        
        self.log_message(f"📦 Đang đọc thông tin từ {len(poses)} PCD files...")
        
        # Đọc thông tin từ các PCD files theo file_index (giống Recorder)
        # Recorder sử dụng: pcd_dir / f"{file_index}.pcd"
        total_points = 0
        loaded_count = 0
        failed_count = 0
        downsampled_count = 0
        missing_files = []
        
        for file_index, (tx, ty, tz, w, x, y, z) in enumerate(poses):
            # Cách đọc tiles từ Recorder: sử dụng file_index để tạo path
            pcd_file = pcd_dir / f"{file_index}.pcd"
            
            if not pcd_file.exists():
                failed_count += 1
                missing_files.append(file_index)
                continue
            
            try:
                # Đọc PCD file với auto-downsample nếu cần
                pcd, was_downsampled, voxel_size = self._read_pcd_with_auto_downsample(pcd_file, preserve_colors=True)
                
                if pcd is None or len(pcd.points) == 0:
                    failed_count += 1
                    continue
                
                num_points = len(pcd.points)
                total_points += num_points
                loaded_count += 1
                
                if was_downsampled:
                    downsampled_count += 1
                
                # Progress indicator
                if loaded_count % 50 == 0 or loaded_count == len(poses):
                    status = f" (auto-downsampled: {downsampled_count})" if downsampled_count > 0 else ""
                    self.log_message(f"  📦 Đã đọc {loaded_count}/{len(poses)} tiles, {total_points:,} points{status}...")
                    
            except Exception as e:
                failed_count += 1
                if failed_count <= 5:  # Chỉ log 5 lỗi đầu tiên
                    self.log_message(f"  ⚠️  Lỗi khi đọc {pcd_file.name}: {e}")
                continue
        
        if loaded_count == 0:
            self.log_message("❌ Không đọc được tile nào")
            return None
        
        if failed_count > 0:
            self.log_message(f"⚠️  {failed_count} tiles không đọc được")
            if missing_files and len(missing_files) <= 10:
                self.log_message(f"   Missing files: {missing_files[:10]}{'...' if len(missing_files) > 10 else ''}")
        
        if downsampled_count > 0:
            self.log_message(f"✅ Đã đọc {loaded_count} tiles, tổng {total_points:,} points")
            self.log_message(f"   📉 {downsampled_count} files đã được tự động downsample (> {self.max_points_per_file:,} points/file)")
        else:
            self.log_message(f"✅ Đã đọc {loaded_count} tiles, tổng {total_points:,} points")
        
        return {
            'name': map_dir.name,
            'total_poses': len(poses),
            'loaded_count': loaded_count,
            'failed_count': failed_count,
            'total_points': total_points,
            'downsampled_count': downsampled_count,
            'pcd_files_found': pcd_files_count  # Thêm thông tin về số PCD files tìm được bằng glob
        }
    
    def clear_log(self):
        """Xóa log"""
        self.log_text.delete(1.0, tk.END)
    
    def start_localization2(self):
        """Start Localization2"""
        if not self.selected_map_dir:
            messagebox.showerror("Error", "Please select a map directory first!")
            return
        
        if not os.path.exists(self.selected_map_dir):
            messagebox.showerror("Error", "Selected map directory does not exist!")
            return
        
        # Validate map directory
        map_path = Path(self.selected_map_dir)
        if not self.is_map_valid(map_path):
            messagebox.showerror("Error", 
                f"Selected map directory is invalid or empty!\n\n"
                f"Map must have:\n"
                f"  - pose.json file with content\n"
                f"  - pcd/ directory with .pcd files (optional but recommended)\n\n"
                f"Please select a valid map directory.")
            self.log_message(f"❌ Invalid map directory: {map_path.name}")
            self.log_message("   Map must have pose.json with content and optionally PCD files")
            return
        
        # Check if script exists
        if not self.script_path.exists():
            messagebox.showerror("Error", f"Start script not found: {self.script_path}")
            self.log_message(f"❌ Script not found: {self.script_path}")
            return
        
        self.log_message("Starting Localization2...")
        self.log_message(f"Map directory: {self.selected_map_dir}")
        
        # Reset drift detection state
        self.localization_start_time = time.time()
        self.out_of_bounds_count = 0
        self.position_jump_count = 0
        self.last_position = None
        self.out_of_bounds_detected = False
        self.last_global_localization_time = 0
        self.global_localization_fail_count = 0
        self.restart_due_to_drift = False
        self.last_drift_restart_time = 0
        self.user_stopped = False
        
        # Update UI
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        self.status_var.set("Starting...")
        self.is_localization_running = True
        
        # Check map size và downsample nếu cần (trong thread riêng để không block UI)
        def start_thread():
            # Kiểm tra và downsample map nếu cần
            map_dir_to_use = self._check_and_downsample_map_if_needed()
            if not map_dir_to_use:
                self.log_message("❌ Failed to prepare map for localization")
                self._reset_ui_state()
                return
            
            # Extract map bounds for drift detection (from Recorder)
            self.log_message("📏 Đang trích xuất bounds từ bản đồ...")
            extracted_bounds = self._extract_map_bounds(map_dir_to_use)
            if extracted_bounds:
                # Expand bounds to account for tile transformations and global localization matching
                expansion_margin = 25.0  # Expand bounds by 25m
                expanded_bounds = {
                    'x': {
                        'min': extracted_bounds['x']['min'] - expansion_margin,
                        'max': extracted_bounds['x']['max'] + expansion_margin
                    },
                    'y': {
                        'min': extracted_bounds['y']['min'] - expansion_margin,
                        'max': extracted_bounds['y']['max'] + expansion_margin
                    },
                    'z': extracted_bounds['z']
                }
                self.map_bounds_original = extracted_bounds  # Use original for strict drift detection
                self.map_bounds = expanded_bounds  # Use expanded for display
                self.log_message(f"✅ Map bounds (đã mở rộng {expansion_margin}m): X=[{self.map_bounds_original['x']['min']:.2f}, {self.map_bounds_original['x']['max']:.2f}], Y=[{self.map_bounds_original['y']['min']:.2f}, {self.map_bounds_original['y']['max']:.2f}]")
            else:
                self.log_message("⚠️ Không thể trích xuất bounds từ bản đồ, tính năng phát hiện trôi sẽ bị tắt")
                self.map_bounds = None
                self.map_bounds_original = None
            
            # Start localization với map đã được xử lý
            self._start_localization2_with_map(map_dir_to_use)
        
        threading.Thread(target=start_thread, daemon=True).start()
    
    def _check_and_downsample_map_if_needed(self):
        """Kiểm tra kích thước map và downsample nếu cần"""
        map_path = Path(self.selected_map_dir)
        
        # Kiểm tra map info đã được load chưa
        if not self.map_info_loaded:
            self.log_message("📊 Đang kiểm tra kích thước map...")
            map_info = self.load_map_tiles_info(str(map_path))
            if not map_info:
                self.log_message("⚠️ Không thể đọc map info, tiếp tục với map gốc")
                return str(map_path)
            
            total_points = map_info['total_points']
        else:
            # Lấy thông tin từ map_info_var nếu đã load
            # Parse từ map_info_var để lấy total_points
            map_info_text = self.map_info_var.get()
            try:
                # Tìm "Total Points: X,XXX,XXX" trong text
                match = re.search(r'Total Points: ([\d,]+)', map_info_text)
                if match:
                    total_points = int(match.group(1).replace(',', ''))
                else:
                    # Nếu không parse được, load lại
                    map_info = self.load_map_tiles_info(str(map_path))
                    if not map_info:
                        return str(map_path)
                    total_points = map_info['total_points']
            except:
                # Nếu parse lỗi, load lại
                map_info = self.load_map_tiles_info(str(map_path))
                if not map_info:
                    return str(map_path)
                total_points = map_info['total_points']
        
        self.log_message(f"📊 Map có {total_points:,} points")
        
        # Kiểm tra nếu map quá nặng
        if total_points > self.max_map_points:
            self.log_message(f"⚠️ Map quá nặng ({total_points:,} points > {self.max_map_points:,})")
            self.log_message("📉 Tự động downsample map để tránh crash RViz...")
            
            downsampled_map_dir = self.downsample_map_tiles(str(map_path), total_map_points=total_points)
            if downsampled_map_dir:
                self.log_message(f"✅ Đã downsample map thành công")
                return downsampled_map_dir
            else:
                self.log_message("⚠️ Không thể downsample map, tiếp tục với map gốc (có thể crash RViz)")
                return str(map_path)
        elif total_points > self.warning_map_points:
            # Map > 5M points - tự động optimize để tránh lag RViz
            self.log_message(f"⚠️ Map khá nặng ({total_points:,} points > {self.warning_map_points:,})")
            self.log_message("📉 Tự động downsample toàn bộ map để RViz mượt hơn...")
            
            # Tự động downsample map với voxel size động (không hỏi user để tránh gián đoạn)
            downsampled_map_dir = self.downsample_map_tiles(str(map_path), total_map_points=total_points)
            if downsampled_map_dir:
                self.log_message(f"✅ Đã downsample map thành công")
                return downsampled_map_dir
            else:
                self.log_message("⚠️ Không thể downsample map, tiếp tục với map gốc")
        elif total_points > self.safe_map_points:
            # Map 2M-5M points - vẫn có thể lag, nên optimize tất cả files
            self.log_message(f"ℹ️ Map có {total_points:,} points (có thể gây lag nhẹ)")
            self.log_message("📉 Tự động optimize tất cả files để RViz mượt hơn...")
        
        # Luôn kiểm tra và auto-downsample các file PCD quá lớn
        # Để đảm bảo RViz mượt khi đọc từng file (ngay cả khi tổng map < 10M)
        # Truyền total_points để nếu map > safe_map_points, downsample tất cả files
        optimized_map_dir = self._auto_downsample_large_files(str(map_path), total_map_points=total_points)
        if optimized_map_dir:
            return optimized_map_dir
        
        # Map OK, sử dụng map gốc
        return str(map_path)
    
    def _auto_downsample_large_files(self, map_dir_path, total_map_points=None):
        """
        Tự động downsample các file PCD quá lớn (> max_points_per_file) 
        hoặc tất cả các file nếu tổng map > safe_map_points
        để đảm bảo RViz mượt khi chạy Localization2
        
        Cách đọc tiles PCD (tích hợp từ Recorder):
        - Đọc pose.json để lấy file_index
        - Sử dụng file_index để tạo path: pcd_dir / f"{file_index}.pcd"
        
        Args:
            map_dir_path: Path đến map directory
            total_map_points: Tổng số points của map (nếu biết trước)
        
        Returns:
            str: Path đến map directory đã được xử lý (nếu có file cần downsample)
                 None nếu không có file nào cần downsample
        """
        if not HAS_OPEN3D:
            return None
        
        map_dir = Path(map_dir_path)
        pose_file = map_dir / "pose.json"
        pcd_dir = map_dir / "pcd"
        
        if not pose_file.exists() or not pcd_dir.exists():
            return None
        
        # Đọc pose.json để biết các file cần kiểm tra (cách của Recorder)
        poses = []
        try:
            with open(pose_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 7:
                        try:
                            tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2])
                            w, x, y, z = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                            poses.append((tx, ty, tz, w, x, y, z))
                        except ValueError:
                            continue
        except Exception as e:
            return None
        
        # Nếu tổng map > safe_map_points, downsample tất cả các file (không chỉ file lớn)
        should_downsample_all = total_map_points and total_map_points > self.safe_map_points
        
        # Kiểm tra từng file PCD theo file_index (cách của Recorder)
        large_files = []
        total_points_checked = 0
        for file_index, _ in enumerate(poses):
            # Cách đọc tiles từ Recorder: sử dụng file_index để tạo path
            pcd_file = pcd_dir / f"{file_index}.pcd"
            if not pcd_file.exists():
                continue
            
            try:
                # Đọc nhanh để kiểm tra số points
                pcd = o3d.io.read_point_cloud(str(pcd_file))
                num_points = len(pcd.points)
                total_points_checked += num_points
                
                # Nếu tổng map lớn, downsample tất cả các file
                # Hoặc nếu file riêng lẻ quá lớn, downsample file đó
                if should_downsample_all or num_points > self.max_points_per_file:
                    large_files.append((file_index, pcd_file, num_points))
            except:
                continue
        
        # Nếu không có file nào cần xử lý, không cần optimize
        if not large_files:
            return None
        
        # Có file cần xử lý, tạo bản copy đã được xử lý
        if should_downsample_all:
            self.log_message(f"📉 Map có {total_map_points:,} points > {self.safe_map_points:,}")
            self.log_message(f"   Tự động downsample tất cả {len(large_files)} files để RViz mượt hơn...")
        else:
            self.log_message(f"📉 Phát hiện {len(large_files)} file PCD quá lớn (> {self.max_points_per_file:,} points/file)")
            self.log_message("   Tự động downsample các file này để RViz mượt hơn...")
        
        # Tạo thư mục optimized map
        temp_dir = Path(tempfile.gettempdir()) / "localization2_optimized"
        temp_dir.mkdir(parents=True, exist_ok=True)
        
        optimized_map_dir = temp_dir / f"{map_dir.name}_optimized"
        optimized_map_dir.mkdir(exist_ok=True)
        optimized_pcd_dir = optimized_map_dir / "pcd"
        optimized_pcd_dir.mkdir(exist_ok=True)
        
        # Copy pose.json
        shutil.copy2(pose_file, optimized_map_dir / "pose.json")
        
        # Xử lý từng file
        processed_count = 0
        for file_index, original_pcd_file, original_points in large_files:
            try:
                # Đọc và auto-downsample file (voxel size động dựa trên số points)
                pcd, was_downsampled, voxel_size = self._read_pcd_with_auto_downsample(original_pcd_file, preserve_colors=True)
                
                if pcd is None:
                    # Copy file gốc nếu không đọc được
                    shutil.copy2(original_pcd_file, optimized_pcd_dir / f"{file_index}.pcd")
                    continue
                
                # Lưu file đã được xử lý
                # Đảm bảo colors được lưu đúng format (PointXYZRGB nếu có colors)
                output_file = optimized_pcd_dir / f"{file_index}.pcd"
                # Open3D tự động detect format dựa trên có colors hay không
                # write_ascii=False: lưu binary format (nhanh hơn, nhỏ hơn)
                # compressed=False: không nén để đảm bảo tương thích tốt
                success = o3d.io.write_point_cloud(
                    str(output_file),
                    pcd,
                    write_ascii=False,
                    compressed=False
                )
                
                if success:
                    reduction = (1 - len(pcd.points) / original_points) * 100 if original_points > 0 else 0
                    color_status = "✅ RGB" if pcd.has_colors() else "⚠️ No RGB"
                    voxel_info = f" (voxel={voxel_size:.3f}m)" if voxel_size else ""
                    self.log_message(f"   📦 File {file_index}.pcd: {original_points:,} → {len(pcd.points):,} points ({reduction:.1f}% reduction){voxel_info} {color_status}")
                    processed_count += 1
                else:
                    # Copy file gốc nếu không lưu được
                    shutil.copy2(original_pcd_file, optimized_pcd_dir / f"{file_index}.pcd")
                    
            except Exception as e:
                # Copy file gốc nếu có lỗi
                try:
                    shutil.copy2(original_pcd_file, optimized_pcd_dir / f"{file_index}.pcd")
                except:
                    pass
                continue
        
        # Copy các file không quá lớn từ map gốc
        for file_index, _ in enumerate(poses):
            if file_index in [f[0] for f in large_files]:
                continue  # Đã xử lý rồi
            
            original_pcd_file = pcd_dir / f"{file_index}.pcd"
            if original_pcd_file.exists():
                try:
                    shutil.copy2(original_pcd_file, optimized_pcd_dir / f"{file_index}.pcd")
                except:
                    pass
        
        if processed_count > 0:
            self.log_message(f"✅ Đã tự động downsample {processed_count} file PCD quá lớn")
            self.log_message(f"   Optimized map: {optimized_map_dir}")
            return str(optimized_map_dir)
        
        return None
    
    @staticmethod
    def _downsample_single_file(args):
        """
        Worker function để downsample một file PCD (dùng cho multiprocessing)
        Args: (pcd_file_path, output_path, dynamic_voxel_size, max_points_per_file, target_points_per_file, file_downsample_voxel_size)
        Returns: (success, file_index, original_points, downsampled_points, has_colors, error_msg)
        """
        try:
            pcd_file_path, output_path, dynamic_voxel_size, max_points_per_file, target_points_per_file, file_downsample_voxel_size = args
            
            if not HAS_OPEN3D:
                return (False, None, 0, 0, False, "open3d not available")
            
            import numpy as np
            
            # Đọc PCD file
            pcd = o3d.io.read_point_cloud(str(pcd_file_path), format='auto')
            if len(pcd.points) == 0:
                return (False, None, 0, 0, False, "Empty point cloud")
            
            original_points = len(pcd.points)
            has_colors = pcd.has_colors()
            
            # Auto-downsample nếu file quá lớn
            if original_points > max_points_per_file:
                if has_colors:
                    colors = np.asarray(pcd.colors)
                    if colors.max() > 1.0 and colors.max() <= 255.0:
                        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
                
                # Tính voxel size động
                reduction_ratio = original_points / target_points_per_file
                file_voxel_size = file_downsample_voxel_size * (reduction_ratio ** (1.0/3.0))
                file_voxel_size = max(0.05, min(0.5, file_voxel_size))
                
                pcd = pcd.voxel_down_sample(voxel_size=file_voxel_size)
            
            # Downsample thêm với voxel size của map
            if len(pcd.points) > 0:
                pcd = pcd.voxel_down_sample(voxel_size=dynamic_voxel_size)
            
            downsampled_points = len(pcd.points)
            
            # Lưu file
            success = o3d.io.write_point_cloud(
                str(output_path),
                pcd,
                write_ascii=False,
                compressed=False
            )
            
            if success:
                return (True, None, original_points, downsampled_points, pcd.has_colors(), None)
            else:
                return (False, None, original_points, 0, False, "Failed to write file")
                
        except Exception as e:
            return (False, None, 0, 0, False, str(e))
    
    def downsample_map_tiles(self, map_dir_path, total_map_points=None):
        """
        Downsample tất cả PCD tiles trong map directory (song song để nhanh hơn)
        Voxel size được tính động dựa trên tổng số points để đảm bảo giảm đủ
        
        Cách đọc tiles PCD (tích hợp từ Recorder):
        - Đọc pose.json để lấy file_index
        - Sử dụng file_index để tạo path: pcd_dir / f"{file_index}.pcd"
        """
        if not HAS_OPEN3D:
            self.log_message("❌ open3d không được cài đặt. Không thể downsample map.")
            self.log_message("💡 Cài đặt: pip3 install open3d")
            return None
        
        map_dir = Path(map_dir_path)
        pose_file = map_dir / "pose.json"
        pcd_dir = map_dir / "pcd"
        
        if not pose_file.exists() or not pcd_dir.exists():
            self.log_message("❌ Map directory không hợp lệ")
            return None
        
        # Tính toán voxel size động dựa trên tổng số points
        # Map càng lớn → voxel size càng lớn → downsample càng nhiều
        if total_map_points:
            # Target: giảm xuống ~2-3M points để RViz rất mượt (giảm từ 4M)
            target_points = 2_500_000  # 2.5M points target (giảm từ 4M)
            if total_map_points > target_points:
                reduction_ratio = total_map_points / target_points
                # Cube root vì volume tỷ lệ với size^3
                dynamic_voxel_size = self.downsample_voxel_size * (reduction_ratio ** (1.0/3.0))
                # Giới hạn trong khoảng hợp lý (0.18m - 0.35m) - tăng để downsample nhiều hơn
                dynamic_voxel_size = max(0.18, min(0.35, dynamic_voxel_size))
            else:
                dynamic_voxel_size = self.downsample_voxel_size
        else:
            dynamic_voxel_size = self.downsample_voxel_size
        
        # Tạo thư mục downsampled map
        temp_dir = Path(tempfile.gettempdir()) / "localization2_downsampled"
        temp_dir.mkdir(parents=True, exist_ok=True)
        
        downsampled_map_dir = temp_dir / f"{map_dir.name}_downsampled"
        downsampled_map_dir.mkdir(exist_ok=True)
        downsampled_pcd_dir = downsampled_map_dir / "pcd"
        downsampled_pcd_dir.mkdir(exist_ok=True)
        
        # Copy pose.json
        shutil.copy2(pose_file, downsampled_map_dir / "pose.json")
        
        # Đọc pose.json
        poses = []
        try:
            with open(pose_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split()
                    if len(parts) >= 7:
                        try:
                            tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2])
                            w, x, y, z = float(parts[3]), float(parts[4]), float(parts[5]), float(parts[6])
                            poses.append((tx, ty, tz, w, x, y, z))
                        except ValueError:
                            continue
        except Exception as e:
            self.log_message(f"❌ Lỗi khi đọc pose.json: {e}")
            return None
        
        self.log_message(f"📉 Đang downsample {len(poses)} PCD tiles (voxel_size={dynamic_voxel_size:.3f}m)...")
        
        # Chuẩn bị danh sách files để xử lý song song
        files_to_process = []
        for file_index, _ in enumerate(poses):
            pcd_file = pcd_dir / f"{file_index}.pcd"
            if pcd_file.exists():
                downsampled_pcd_file = downsampled_pcd_dir / f"{file_index}.pcd"
                files_to_process.append((
                    str(pcd_file),
                    str(downsampled_pcd_file),
                    dynamic_voxel_size,
                    self.max_points_per_file,
                    self.target_points_per_file,
                    self.file_downsample_voxel_size
                ))
        
        # Sử dụng multiprocessing để xử lý song song (nhanh hơn nhiều)
        loaded_count = 0
        failed_count = 0
        total_points_before = 0
        total_points_after = 0
        has_rgb_count = 0
        
        if HAS_MULTIPROCESSING and len(files_to_process) > 10:
            # Sử dụng multiprocessing nếu có nhiều files (>10)
            num_workers = min(cpu_count(), 8)  # Tối đa 8 workers để tránh quá tải
            self.log_message(f"   🚀 Sử dụng {num_workers} workers để downsample song song (nhanh hơn ~{num_workers}x)...")
            
            try:
                with Pool(processes=num_workers) as pool:
                    results = pool.map(self._downsample_single_file, files_to_process)
                
                # Xử lý kết quả
                for i, (success, _, orig_points, down_points, has_colors, error_msg) in enumerate(results):
                    if success:
                        loaded_count += 1
                        total_points_before += orig_points
                        total_points_after += down_points
                        if has_colors:
                            has_rgb_count += 1
                        
                        # Progress indicator
                        if loaded_count % 50 == 0 or loaded_count == len(files_to_process):
                            reduction = (1 - total_points_after / total_points_before) * 100 if total_points_before > 0 else 0
                            self.log_message(f"  📦 Đã downsample {loaded_count}/{len(files_to_process)} tiles, "
                                           f"{total_points_after:,} points ({reduction:.1f}% reduction)...")
                    else:
                        failed_count += 1
                        if failed_count <= 5 and error_msg:
                            self.log_message(f"  ⚠️  Lỗi: {error_msg}")
            except Exception as e:
                self.log_message(f"  ⚠️  Lỗi multiprocessing, chuyển sang xử lý tuần tự: {e}")
                # Reset counters for sequential processing
                loaded_count = 0
                failed_count = 0
                total_points_before = 0
                total_points_after = 0
                has_rgb_count = 0
        
        # Xử lý tuần tự nếu không dùng multiprocessing hoặc có lỗi
        # Cách đọc tiles từ Recorder: sử dụng file_index để tạo path
        use_sequential = not (HAS_MULTIPROCESSING and len(files_to_process) > 10) or loaded_count == 0
        if use_sequential:
            for file_index, (tx, ty, tz, w, x, y, z) in enumerate(poses):
                # Cách đọc tiles từ Recorder: pcd_dir / f"{file_index}.pcd"
                pcd_file = pcd_dir / f"{file_index}.pcd"
                downsampled_pcd_file = downsampled_pcd_dir / f"{file_index}.pcd"
                
                if not pcd_file.exists():
                    failed_count += 1
                    continue
                
                try:
                    # Đọc PCD file với auto-downsample nếu file quá lớn
                    pcd, was_auto_downsampled, voxel_size = self._read_pcd_with_auto_downsample(pcd_file, preserve_colors=True)
                    
                    if pcd is None or len(pcd.points) == 0:
                        failed_count += 1
                        continue
                    
                    # Đếm points gốc
                    if was_auto_downsampled:
                        try:
                            original_pcd = o3d.io.read_point_cloud(str(pcd_file))
                            points_before_file = len(original_pcd.points) if original_pcd else len(pcd.points)
                        except:
                            points_before_file = int(len(pcd.points) * 1.5)
                    else:
                        points_before_file = len(pcd.points)
                    
                    total_points_before += points_before_file
                    has_colors = pcd.has_colors()
                    
                    # Downsample thêm với voxel size động
                    downsampled_pcd = pcd.voxel_down_sample(voxel_size=dynamic_voxel_size)
                    total_points_after += len(downsampled_pcd.points)
                    
                    # Đảm bảo colors được preserve
                    if has_colors and not downsampled_pcd.has_colors():
                        import numpy as np
                        if HAS_SCIPY and cKDTree:
                            original_points = np.asarray(pcd.points)
                            original_colors = np.asarray(pcd.colors)
                            downsampled_points = np.asarray(downsampled_pcd.points)
                            tree = cKDTree(original_points)
                            _, indices = tree.query(downsampled_points, k=1)
                            downsampled_colors = original_colors[indices]
                            downsampled_pcd.colors = o3d.utility.Vector3dVector(downsampled_colors)
                    
                    # Lưu file
                    success = o3d.io.write_point_cloud(
                        str(downsampled_pcd_file), 
                        downsampled_pcd, 
                        write_ascii=False,
                        compressed=False
                    )
                    if not success:
                        failed_count += 1
                        continue
                    
                    loaded_count += 1
                    if downsampled_pcd.has_colors():
                        has_rgb_count += 1
                    
                    # Progress indicator
                    if loaded_count % 50 == 0 or loaded_count == len(poses):
                        reduction = (1 - total_points_after / total_points_before) * 100 if total_points_before > 0 else 0
                        self.log_message(f"  📦 Đã downsample {loaded_count}/{len(poses)} tiles, "
                                       f"{total_points_after:,} points ({reduction:.1f}% reduction)...")
                        
                except Exception as e:
                    failed_count += 1
                    if failed_count <= 5:
                        self.log_message(f"  ⚠️  Lỗi khi downsample {pcd_file.name}: {e}")
                    continue
        
        if loaded_count == 0:
            self.log_message("❌ Không downsample được tile nào")
            return None
        
        if failed_count > 0:
            self.log_message(f"⚠️  {failed_count} tiles không downsample được")
        
        reduction = (1 - total_points_after / total_points_before) * 100 if total_points_before > 0 else 0
        
        # Kiểm tra xem có colors trong downsampled map không
        sample_pcd = o3d.io.read_point_cloud(str(downsampled_pcd_dir / "0.pcd"))
        has_rgb = sample_pcd.has_colors() if sample_pcd and len(sample_pcd.points) > 0 else False
        
        self.log_message(f"✅ Đã downsample {loaded_count} tiles")
        self.log_message(f"   Points: {total_points_before:,} → {total_points_after:,} ({reduction:.1f}% reduction)")
        if has_rgb_count > 0:
            rgb_percent = (has_rgb_count / loaded_count) * 100 if loaded_count > 0 else 0
            self.log_message(f"   ✅ {has_rgb_count}/{loaded_count} files có RGB colors ({rgb_percent:.1f}%) - map sẽ hiển thị đầy đủ màu sắc")
        else:
            self.log_message(f"   ⚠️ No RGB colors detected - map sẽ hiển thị màu vàng mặc định")
        self.log_message(f"   Downsampled map: {downsampled_map_dir}")
        
        return str(downsampled_map_dir)
    
    def _start_localization2_with_map(self, map_dir):
        """Start Localization2 với map directory đã được xử lý"""
        try:
            # Make script executable
            os.chmod(self.script_path, 0o755)
            
            # Start localization2
            cmd = [str(self.script_path), map_dir]
            self.log_message(f"Executing: {' '.join(cmd)}")
            
            self.localization_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                cwd=str(self.project_root)
            )
            
            # Start ROS listener
            self.start_ros_listener()
            
            # Read output và filter warnings không cần thiết
            warnings_to_filter = [
                "Failed to find match for field 'normal_x'",
                "Failed to find match for field 'normal_y'",
                "Failed to find match for field 'normal_z'",
                "Failed to find match for field 'intensity'",
                "Failed to find match for field 'curvature'",
            ]
            
            warning_count = {}
            for warning in warnings_to_filter:
                warning_count[warning] = 0
            
            warning_shown = False  # Chỉ hiển thị warning message một lần
            
            for line in iter(self.localization_process.stdout.readline, ''):
                if not line:
                    break
                
                line_stripped = line.strip()
                line_lower = line_stripped.lower()
                
                # Detect global localization events (from Recorder)
                # NOTE: "global match map id" chỉ là match ban đầu, chưa chắc đã thành công
                # Chỉ reset fail counter khi thực sự applied/successful
                if ("applied global localization" in line_lower or 
                    "global localization successfully" in line_lower):
                    # Global localization actually succeeded
                    self.last_global_localization_time = time.time()
                    self.global_localization_fail_count = 0  # Reset fail counter on success
                    self.log_message("   🔍 Phát hiện global localization thành công, tăng threshold drift detection tạm thời...")
                
                # Detect global localization failures (bao gồm cả khi match nhưng bị reject sau ICP)
                elif ("rejecting global localization" in line_lower or
                      "global localization rejected" in line_lower or
                      "gicp fallback rejected" in line_lower or
                      ("global match map id" in line_lower and "rejecting" in line_lower)):
                    # Global localization failed
                    self.global_localization_fail_count += 1
                    self.log_message(f"   ⚠️ Phát hiện global localization fail (Lần {self.global_localization_fail_count}/{self.global_localization_fail_threshold})")
                    
                    # Check if we should restart due to repeated failures
                    if self.global_localization_fail_count >= self.global_localization_fail_threshold:
                        self.log_message(f"   🚨 Global localization fail {self.global_localization_fail_count} lần liên tục, khởi động lại localization...")
                        # Schedule restart on main thread
                        self.after(0, self._restart_on_global_loc_fail)
                
                # Detect khi có "global match map id" nhưng sau đó bị reject (pattern: match -> reject)
                # Pattern này xảy ra khi ScanContext match nhưng ICP refinement fail
                elif "global match map id" in line_lower:
                    # Chỉ log, không reset counter vì chưa chắc thành công
                    # Sẽ đợi xem có "rejecting" sau đó không
                    pass
                
                # Filter warnings về missing fields (chỉ hiển thị summary)
                should_filter = False
                matched_warning = None
                for warning in warnings_to_filter:
                    if warning in line_stripped:
                        warning_count[warning] += 1
                        should_filter = True
                        matched_warning = warning
                        break
                
                if not should_filter:
                    self.log_message(line_stripped)
                elif not warning_shown:
                    # Hiển thị warning đầu tiên với note
                    self.log_message(f"⚠️ PCD files missing some fields (normal_x, normal_y, intensity, etc.) - this is normal for PointXYZRGB format. Warnings will be suppressed.")
                    warning_shown = True
            
            # Hiển thị summary nếu có nhiều warnings
            total_warnings = sum(warning_count.values())
            if total_warnings > 10:
                self.log_message(f"ℹ️ Suppressed {total_warnings} PCD field warnings (normal for PointXYZRGB format)")
            
            # Process finished
            return_code = self.localization_process.wait()
            self.localization_process = None
            
            if return_code == 0:
                self.log_message("✅ Localization2 finished successfully")
            else:
                self.log_message(f"⚠️ Localization2 finished with return code: {return_code}")
            
        except Exception as e:
            self.log_message(f"❌ Error starting Localization2: {e}")
        finally:
            self._reset_ui_state()
    
    def stop_localization2(self):
        """Stop Localization2"""
        self.log_message("Stopping Localization2...")
        self.user_stopped = True  # Mark as user stopped
        
        # Stop ROS listener
        self.stop_ros_listener()
        
        # Stop process
        if self.localization_process:
            try:
                if platform.system() == "Windows":
                    self.localization_process.terminate()
                else:
                    os.kill(self.localization_process.pid, signal.SIGTERM)
                
                # Wait a bit
                time.sleep(2)
                
                # Force kill if still running
                if self.localization_process.poll() is None:
                    if platform.system() == "Windows":
                        self.localization_process.kill()
                    else:
                        os.kill(self.localization_process.pid, signal.SIGKILL)
                
                self.log_message("✅ Localization2 stopped")
            except Exception as e:
                self.log_message(f"⚠️ Error stopping process: {e}")
        
        # Kill any remaining localization processes
        try:
            if platform.system() != "Windows":
                subprocess.run(['pkill', '-f', 'localization'], timeout=3, capture_output=True)
                subprocess.run(['pkill', '-f', 'fast_lio_localization'], timeout=3, capture_output=True)
        except:
            pass
        
        self._reset_ui_state()
    
    def _reset_ui_state(self):
        """Reset UI state"""
        self.is_localization_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.status_var.set("Stopped")
    
    def start_ros_listener(self):
        """Start ROS2 listener for localization data"""
        if not ROS2_AVAILABLE:
            self.log_message("⚠️ ROS2 not available, cannot subscribe to /Odometry")
            return
        
        if self.is_ros_running:
            return
        
        def ros_thread():
            try:
                if not rclpy.ok():
                    rclpy.init()
                
                self.ros_node = LocalizationListenerNode(self)
                self.ros_executor = MultiThreadedExecutor()
                self.ros_executor.add_node(self.ros_node)
                
                self.is_ros_running = True
                self.log_message("✅ Started ROS2 listener for /Odometry topic")
                
                # Run executor
                self.ros_executor.spin()
            except Exception as e:
                self.log_message(f"❌ Error in ROS2 listener: {e}")
            finally:
                self.is_ros_running = False
        
        self.ros_thread = threading.Thread(target=ros_thread, daemon=True)
        self.ros_thread.start()
    
    def stop_ros_listener(self):
        """Stop ROS2 listener"""
        if not self.is_ros_running:
            return
        
        try:
            if self.ros_executor:
                self.ros_executor.shutdown()
            if self.ros_node:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.log_message("✅ Stopped ROS2 listener")
        except Exception as e:
            self.log_message(f"⚠️ Error stopping ROS2 listener: {e}")
        finally:
            self.is_ros_running = False
    
    def update_localization_data(self, msg):
        """Update localization data from ROS message (with drift detection from Recorder)"""
        try:
            # Extract data from message
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            vel = msg.twist.twist.linear
            
            # Store current localization data (from Recorder)
            current_time = time.time()
            self.current_localization = {
                'position': {'x': pos.x, 'y': pos.y, 'z': pos.z},
                'orientation': {'x': orient.x, 'y': orient.y, 'z': orient.z, 'w': orient.w},
                'velocity': {'x': vel.x, 'y': vel.y, 'z': vel.z},
                'timestamp': msg.header.stamp,
                'frame_id': msg.header.frame_id
            }
            
            # Drift detection (from Recorder)
            if self.is_localization_running and self.map_bounds:
                self._check_drift_detection(pos.x, pos.y, pos.z, msg.header.frame_id, current_time)
            
            # Update UI (must be done in main thread)
            self.after(0, lambda: self._update_ui_data(
                pos.x, pos.y, pos.z,
                orient.x, orient.y, orient.z, orient.w,
                vel.x, vel.y, vel.z,
                msg.header.stamp
            ))
        except Exception as e:
            if ROS2_AVAILABLE and self.ros_node:
                self.ros_node.get_logger().error(f'Error updating localization data: {e}')
            else:
                self.log_message(f"Error updating localization data: {e}")
    
    def _update_ui_data(self, px, py, pz, ox, oy, oz, ow, vx, vy, vz, stamp):
        """Update UI with localization data (called from main thread)"""
        try:
            self.position_var.set(f"({px:.3f}, {py:.3f}, {pz:.3f})")
            self.orientation_var.set(f"({ox:.3f}, {oy:.3f}, {oz:.3f}, {ow:.3f})")
            self.velocity_var.set(f"({vx:.3f}, {vy:.3f}, {vz:.3f})")
            
            # Convert timestamp
            if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
                dt = datetime.fromtimestamp(stamp.sec + stamp.nanosec / 1e9)
                self.timestamp_var.set(dt.strftime("%H:%M:%S.%f")[:-3])
            else:
                self.timestamp_var.set("N/A")
        except Exception as e:
            self.log_message(f"Error updating UI: {e}")
    
    def _extract_map_bounds(self, map_path):
        """Extract map bounds from PCD file(s) (from Recorder)"""
        try:
            if not HAS_OPEN3D:
                self.log_message("⚠️ open3d không có sẵn, không thể trích xuất bounds")
                return None
            
            import numpy as np
            map_path_obj = Path(map_path)
            self.log_message(f"📏 Đang trích xuất bounds từ: {map_path}")
            
            # Handle FAST-Localization map directory (with pcd/ subfolder)
            if map_path_obj.is_dir():
                pcd_dir = map_path_obj / "pcd"
                if pcd_dir.exists() and pcd_dir.is_dir():
                    # Get all PCD files in the directory
                    pcd_files = list(pcd_dir.glob("*.pcd"))
                    self.log_message(f"   Tìm thấy {len(pcd_files)} file PCD trong thư mục pcd/")
                    
                    if not pcd_files:
                        self.log_message("⚠️ Không tìm thấy file PCD trong thư mục map")
                        return None
                    
                    # Calculate bounds from all PCD files
                    all_min_x = []
                    all_max_x = []
                    all_min_y = []
                    all_max_y = []
                    all_min_z = []
                    all_max_z = []
                    
                    successful_reads = 0
                    for pcd_file in pcd_files[:10]:  # Limit to first 10 files for speed
                        try:
                            pcd = o3d.io.read_point_cloud(str(pcd_file))
                            if len(pcd.points) > 0:
                                points = np.asarray(pcd.points)
                                all_min_x.append(float(np.min(points[:, 0])))
                                all_max_x.append(float(np.max(points[:, 0])))
                                all_min_y.append(float(np.min(points[:, 1])))
                                all_max_y.append(float(np.max(points[:, 1])))
                                all_min_z.append(float(np.min(points[:, 2])))
                                all_max_z.append(float(np.max(points[:, 2])))
                                successful_reads += 1
                        except:
                            continue
                    
                    self.log_message(f"   Đọc thành công {successful_reads}/{min(len(pcd_files), 10)} file PCD")
                    
                    if all_min_x:
                        result = {
                            'x': {'min': min(all_min_x), 'max': max(all_max_x)},
                            'y': {'min': min(all_min_y), 'max': max(all_max_y)},
                            'z': {'min': min(all_min_z), 'max': max(all_max_z)}
                        }
                        self.log_message(f"✅ Tổng hợp bounds từ {successful_reads} file: X=[{result['x']['min']:.2f}, {result['x']['max']:.2f}], Y=[{result['y']['min']:.2f}, {result['y']['max']:.2f}]")
                        return result
                    else:
                        self.log_message("⚠️ Không đọc được bounds từ bất kỳ file PCD nào")
                        return None
                else:
                    self.log_message("⚠️ Không tìm thấy thư mục pcd/")
                    return None
            
            self.log_message(f"⚠️ Đường dẫn không hợp lệ: {map_path}")
            return None
            
        except Exception as e:
            self.log_message(f"❌ Lỗi khi trích xuất bounds từ map: {e}")
            import traceback
            self.log_message(f"   Traceback: {traceback.format_exc()}")
            return None
    
    def _check_drift_detection(self, pos_x, pos_y, pos_z, frame_id, current_time):
        """Check for drift detection (from Recorder)"""
        try:
            # Check if we're past initialization grace period
            if current_time - self.localization_start_time < self.initialization_grace_period:
                return  # Still in grace period
            
            # Check if position is outside bounds
            is_outside = False
            if self.map_bounds_original:
                bounds_to_check = self.map_bounds_original
            else:
                bounds_to_check = self.map_bounds
            
            if bounds_to_check:
                if (pos_x < bounds_to_check['x']['min'] or pos_x > bounds_to_check['x']['max'] or
                    pos_y < bounds_to_check['y']['min'] or pos_y > bounds_to_check['y']['max']):
                    is_outside = True
            
            # Calculate how far outside bounds
            x_outside = 0
            y_outside = 0
            if is_outside and bounds_to_check:
                if pos_x < bounds_to_check['x']['min']:
                    x_outside = bounds_to_check['x']['min'] - pos_x
                elif pos_x > bounds_to_check['x']['max']:
                    x_outside = pos_x - bounds_to_check['x']['max']
                if pos_y < bounds_to_check['y']['min']:
                    y_outside = bounds_to_check['y']['min'] - pos_y
                elif pos_y > bounds_to_check['y']['max']:
                    y_outside = pos_y - bounds_to_check['y']['max']
            
            max_outside = max(x_outside, y_outside) if is_outside else 0
            
            # Detect drift within bounds: check for position jumps or abnormal speed
            position_jump_detected = False
            abnormal_speed_detected = False
            if not is_outside and self.last_position is not None:
                time_diff = current_time - self.last_position['timestamp']
                if time_diff > 0:
                    dx = pos_x - self.last_position['x']
                    dy = pos_y - self.last_position['y']
                    dz = pos_z - self.last_position['z']
                    distance_moved = (dx**2 + dy**2 + dz**2)**0.5
                    speed_calculated = distance_moved / time_diff
                    
                    # Update speed history for adaptive threshold
                    if self.adaptive_speed_threshold_enabled:
                        self.speed_history.append(speed_calculated)
                        if len(self.speed_history) > self.speed_history_max_size:
                            self.speed_history.pop(0)
                        
                        if len(self.speed_history) >= self.min_speed_samples:
                            import numpy as np
                            speeds_array = np.array(self.speed_history)
                            mean_speed = np.mean(speeds_array)
                            std_speed = np.std(speeds_array)
                            adaptive_threshold = mean_speed + self.speed_threshold_multiplier * std_speed
                            self.max_reasonable_speed = max(adaptive_threshold, 2.0)
                    
                    # Detect position jump
                    if distance_moved > self.position_jump_threshold:
                        position_jump_detected = True
                        self.position_jump_count += 1
                        self.log_message(f"⚠️ Phát hiện position jump: di chuyển {distance_moved:.2f}m trong {time_diff:.2f}s")
                    
                    # Detect abnormal speed
                    elif speed_calculated > self.max_reasonable_speed:
                        abnormal_speed_detected = True
                        self.position_jump_count += 1
                        self.log_message(f"⚠️ Phát hiện tốc độ bất thường: {speed_calculated:.2f}m/s")
                    else:
                        if self.position_jump_count > 0:
                            self.position_jump_count = 0
            
            # Update last position
            self.last_position = {
                'x': pos_x,
                'y': pos_y,
                'z': pos_z,
                'timestamp': current_time
            }
            
            # Check drift if actually outside bounds OR position jump detected OR global localization fail nhiều lần
            # Thêm check global localization fail như một dấu hiệu drift
            global_loc_fail_drift = self.global_localization_fail_count >= self.global_localization_fail_threshold
            
            if is_outside or position_jump_detected or (abnormal_speed_detected and self.position_jump_count >= self.position_jump_threshold_count) or global_loc_fail_drift:
                self.out_of_bounds_count += 1
                
                if is_outside:
                    self.log_message(f"⚠️ CẢNH BÁO: Thiết bị đã trôi ra ngoài bản đồ! (Lần {self.out_of_bounds_count}/{self.out_of_bounds_threshold})")
                    self.log_message(f"   📍 Vị trí: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})")
                    self.log_message(f"   🚨 Trôi ra ngoài: X={x_outside:.2f}m, Y={y_outside:.2f}m (tối đa: {max_outside:.2f}m)")
                elif global_loc_fail_drift:
                    self.log_message(f"⚠️ CẢNH BÁO: Global localization fail {self.global_localization_fail_count} lần liên tục - dấu hiệu drift!")
                    self.log_message(f"   📍 Vị trí: ({pos_x:.2f}, {pos_y:.2f}, {pos_z:.2f})")
                
                # Determine restart condition
                should_restart = False
                
                # Check if we're in grace period after global localization
                time_since_global_loc = current_time - self.last_global_localization_time if self.last_global_localization_time > 0 else float('inf')
                in_global_loc_grace = time_since_global_loc < self.global_localization_grace_period
                
                # Check if we recently restarted due to drift
                time_since_drift_restart = current_time - self.last_drift_restart_time if self.last_drift_restart_time > 0 else float('inf')
                in_drift_restart_window = self.restart_due_to_drift and time_since_drift_restart < self.drift_restart_window
                
                # Use adaptive threshold
                if in_drift_restart_window:
                    drift_threshold = 0.5
                elif in_global_loc_grace:
                    drift_threshold = self.drift_threshold_after_global_loc
                else:
                    drift_threshold = 1.0
                
                # Check if should restart
                # Priority 1: Global localization fail nhiều lần (dấu hiệu drift rõ ràng)
                if global_loc_fail_drift:
                    should_restart = True
                    self.log_message(f"   🚨 PHÁT HIỆN DRIFT: Global localization fail {self.global_localization_fail_count} lần liên tục, khởi động lại...")
                # Priority 2: Outside bounds với drift lớn
                elif is_outside and max_outside > drift_threshold:
                    if not in_global_loc_grace:
                        should_restart = True
                        self.log_message(f"   🚨 PHÁT HIỆN DRIFT: {max_outside:.2f}m ngoài bounds, khởi động lại...")
                # Priority 3: Position jump hoặc abnormal speed
                elif (position_jump_detected or abnormal_speed_detected) and self.position_jump_count >= self.position_jump_threshold_count:
                    if not in_global_loc_grace:
                        should_restart = True
                        self.log_message(f"   🚨 PHÁT HIỆN DRIFT TRONG BOUNDS: {self.position_jump_count} lần liên tục, khởi động lại...")
                # Priority 4: Outside bounds nhiều lần liên tục
                elif is_outside and not in_global_loc_grace and self.out_of_bounds_count >= self.out_of_bounds_threshold:
                    should_restart = True
                    self.log_message(f"   ⚠️ Đã phát hiện {self.out_of_bounds_count} lần liên tục, khởi động lại...")
                
                if should_restart:
                    with self.restart_lock:
                        if not self.is_restarting:
                            self.out_of_bounds_detected = True
                            self.log_message(f"   🚨 KÍCH HOẠT KHỞI ĐỘNG LẠI LOCALIZATION...")
                            self.after(0, self._check_and_restart_localization)
            elif not is_outside and not position_jump_detected and not abnormal_speed_detected:
                # Reset counter when back in bounds
                if self.out_of_bounds_count > 0:
                    self.out_of_bounds_count = 0
                if self.position_jump_count > 0:
                    self.position_jump_count = 0
                
                # Reset drift restart flag if stable
                if self.restart_due_to_drift:
                    time_since_drift_restart = current_time - self.last_drift_restart_time if self.last_drift_restart_time > 0 else float('inf')
                    if time_since_drift_restart > self.drift_restart_window:
                        self.restart_due_to_drift = False
                
                if self.out_of_bounds_detected:
                    self.out_of_bounds_detected = False
                    
        except Exception as e:
            self.log_message(f"Error in drift detection: {e}")
    
    def _check_and_restart_localization(self):
        """Check and restart localization if needed (from Recorder)"""
        try:
            with self.restart_lock:
                if self.is_restarting:
                    return
                self.is_restarting = True
            
            self.log_message("🔄 Đang khởi động lại localization do drift detection...")
            
            # Stop current localization
            self.stop_localization2()
            time.sleep(2)
            
            # Reset flags
            self.restart_due_to_drift = True
            self.last_drift_restart_time = time.time()
            self.out_of_bounds_count = 0
            self.position_jump_count = 0
            
            # Restart
            self.start_localization2()
            
        except Exception as e:
            self.log_message(f"Error restarting localization: {e}")
        finally:
            with self.restart_lock:
                self.is_restarting = False
    
    def _restart_on_global_loc_fail(self):
        """Restart localization when global localization fails repeatedly (from Recorder)"""
        try:
            with self.restart_lock:
                if self.is_restarting:
                    self.log_message("⏳ Đang trong quá trình khởi động lại, bỏ qua yêu cầu restart do global localization fail...")
                    return
                self.is_restarting = True
            
            self.log_message("=" * 60)
            self.log_message("🔄 Đang khởi động lại localization do global localization fail nhiều lần...")
            self.log_message(f"   Global localization fail count: {self.global_localization_fail_count}/{self.global_localization_fail_threshold}")
            self.log_message("=" * 60)
            
            # Reset fail counter và flags
            self.global_localization_fail_count = 0
            self.restart_due_to_drift = True
            self.last_drift_restart_time = time.time()
            self.out_of_bounds_count = 0
            self.position_jump_count = 0
            
            # Stop current localization
            self.stop_localization2()
            time.sleep(2)
            
            # Restart
            self.user_stopped = False  # Reset flag for restart
            self.start_localization2()
            
        except Exception as e:
            self.log_message(f"❌ Lỗi khi khởi động lại localization do global localization fail: {e}")
        finally:
            with self.restart_lock:
                self.is_restarting = False

