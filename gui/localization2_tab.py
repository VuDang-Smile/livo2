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
        
        # Map size limits (để tránh crash RViz)
        self.max_map_points = 50_000_000  # 50M points - giới hạn an toàn
        self.warning_map_points = 30_000_000  # 30M points - cảnh báo
        self.safe_map_points = 20_000_000  # 20M points - kích thước an toàn
        self.downsample_voxel_size = 0.15  # Voxel size cho downsample (m)
        
        # Localization data
        self.position_var = None
        self.orientation_var = None
        self.velocity_var = None
        self.timestamp_var = None
        
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
        """Load và tính toán thông tin về map tiles (giống pcd_viewer_tab.py nhưng không merge)"""
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
        self.log_message(f"📦 Đang đọc thông tin từ {len(poses)} PCD files...")
        
        # Đọc thông tin từ các PCD files (không merge, chỉ đếm points)
        total_points = 0
        loaded_count = 0
        failed_count = 0
        
        import numpy as np
        
        for file_index, (tx, ty, tz, w, x, y, z) in enumerate(poses):
            pcd_file = pcd_dir / f"{file_index}.pcd"
            
            if not pcd_file.exists():
                failed_count += 1
                continue
            
            try:
                # Load PCD file
                pcd = o3d.io.read_point_cloud(str(pcd_file))
                if len(pcd.points) == 0:
                    failed_count += 1
                    continue
                
                total_points += len(pcd.points)
                loaded_count += 1
                
                # Progress indicator
                if loaded_count % 50 == 0 or loaded_count == len(poses):
                    self.log_message(f"  📦 Đã đọc {loaded_count}/{len(poses)} tiles, {total_points:,} points...")
                    
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
        
        self.log_message(f"✅ Đã đọc {loaded_count} tiles, tổng {total_points:,} points")
        
        return {
            'name': map_dir.name,
            'total_poses': len(poses),
            'loaded_count': loaded_count,
            'failed_count': failed_count,
            'total_points': total_points
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
            
            downsampled_map_dir = self.downsample_map_tiles(str(map_path))
            if downsampled_map_dir:
                self.log_message(f"✅ Đã downsample map thành công")
                return downsampled_map_dir
            else:
                self.log_message("⚠️ Không thể downsample map, tiếp tục với map gốc (có thể crash RViz)")
                return str(map_path)
        elif total_points > self.warning_map_points:
            self.log_message(f"⚠️ Map khá nặng ({total_points:,} points), có thể gây chậm RViz")
            # Hỏi người dùng có muốn downsample không (trong main thread)
            response_queue = queue.Queue()
            def ask_response():
                response = messagebox.askyesno(
                    "Map Warning",
                    f"Map có {total_points:,} points (khá lớn).\n\n"
                    f"Bạn có muốn downsample để tăng tốc độ không?\n\n"
                    f"• YES: Downsample map (an toàn hơn)\n"
                    f"• NO: Sử dụng map gốc (có thể chậm)"
                )
                response_queue.put(response)
            
            self.after(0, ask_response)
            try:
                response = response_queue.get(timeout=60)
                if response:
                    self.log_message("📉 Đang downsample map...")
                    downsampled_map_dir = self.downsample_map_tiles(str(map_path))
                    if downsampled_map_dir:
                        self.log_message(f"✅ Đã downsample map thành công")
                        return downsampled_map_dir
            except queue.Empty:
                self.log_message("⚠️ Timeout, sử dụng map gốc")
        
        # Map OK, sử dụng map gốc
        return str(map_path)
    
    def downsample_map_tiles(self, map_dir_path):
        """Downsample tất cả PCD tiles trong map directory"""
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
        
        self.log_message(f"📉 Đang downsample {len(poses)} PCD tiles (voxel_size={self.downsample_voxel_size}m)...")
        
        # Downsample từng PCD file
        loaded_count = 0
        failed_count = 0
        total_points_before = 0
        total_points_after = 0
        
        for file_index, (tx, ty, tz, w, x, y, z) in enumerate(poses):
            pcd_file = pcd_dir / f"{file_index}.pcd"
            downsampled_pcd_file = downsampled_pcd_dir / f"{file_index}.pcd"
            
            if not pcd_file.exists():
                failed_count += 1
                continue
            
            try:
                # Load PCD file (open3d tự động detect và giữ RGB colors nếu có)
                pcd = o3d.io.read_point_cloud(str(pcd_file))
                if len(pcd.points) == 0:
                    failed_count += 1
                    continue
                
                total_points_before += len(pcd.points)
                
                # Kiểm tra xem có colors không
                has_colors = pcd.has_colors()
                if has_colors:
                    # Đảm bảo colors được normalize đúng (0-1 range)
                    import numpy as np
                    colors = np.asarray(pcd.colors)
                    if colors.max() > 1.0:
                        # Colors có thể ở dạng 0-255, normalize về 0-1
                        pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
                
                # Downsample (open3d tự động preserve colors nếu có)
                downsampled_pcd = pcd.voxel_down_sample(voxel_size=self.downsample_voxel_size)
                total_points_after += len(downsampled_pcd.points)
                
                # Đảm bảo colors được preserve sau downsample
                # Open3d's voxel_down_sample() tự động preserve colors, nhưng kiểm tra để chắc chắn
                if has_colors and not downsampled_pcd.has_colors():
                    # Nếu mất colors sau downsample (hiếm khi xảy ra), khôi phục colors từ original
                    self.log_message(f"  ⚠️  Colors bị mất sau downsample cho {pcd_file.name}, đang khôi phục...")
                    if HAS_SCIPY and cKDTree:
                        # Thử với KDTree để tìm nearest colors
                        import numpy as np
                        original_points = np.asarray(pcd.points)
                        original_colors = np.asarray(pcd.colors)
                        downsampled_points = np.asarray(downsampled_pcd.points)
                        
                        # Tìm colors cho downsampled points từ original
                        tree = cKDTree(original_points)
                        _, indices = tree.query(downsampled_points, k=1)
                        downsampled_colors = original_colors[indices]
                        downsampled_pcd.colors = o3d.utility.Vector3dVector(downsampled_colors)
                    else:
                        # Fallback: tạo colors trắng nếu không có scipy
                        import numpy as np
                        default_colors = np.ones((len(downsampled_pcd.points), 3))
                        downsampled_pcd.colors = o3d.utility.Vector3dVector(default_colors)
                
                # Lưu file đã downsample (open3d tự động lưu RGB nếu có colors)
                success = o3d.io.write_point_cloud(
                    str(downsampled_pcd_file), 
                    downsampled_pcd, 
                    write_ascii=False,
                    compressed=False  # Đảm bảo format đúng
                )
                if not success:
                    failed_count += 1
                    continue
                
                loaded_count += 1
                
                # Kiểm tra colors sau downsample
                has_colors_after = downsampled_pcd.has_colors()
                color_status = "✅ RGB" if has_colors_after else "⚠️ No RGB"
                
                # Progress indicator
                if loaded_count % 50 == 0 or loaded_count == len(poses):
                    reduction = (1 - total_points_after / total_points_before) * 100 if total_points_before > 0 else 0
                    self.log_message(f"  📦 Đã downsample {loaded_count}/{len(poses)} tiles, "
                                   f"{total_points_after:,} points ({reduction:.1f}% reduction) {color_status}...")
                    
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
        if has_rgb:
            self.log_message(f"   ✅ RGB colors preserved - map sẽ hiển thị đầy đủ màu sắc")
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
        """Update localization data from ROS message"""
        try:
            # Extract data from message
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            vel = msg.twist.twist.linear
            
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

