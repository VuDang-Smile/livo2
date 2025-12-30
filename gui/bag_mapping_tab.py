#!/usr/bin/env python3
"""
Bag Mapping Tab Module
Tab để chọn bag file và thực hiện mapping bằng FAST-LIVO2 với option show RViz2
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

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)

# Try to import ROS2 for service calls
try:
    import rclpy
    from std_srvs.srv import Trigger
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


class BagMappingTab(ttk.Frame):
    """Tab cho Bag Mapping với FAST-LIVO2"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
        
        # Processes
        self.mapping_process = None
        self.bag_process = None
        
        # State
        self.is_mapping_running = False
        self.is_bag_playing = False
        self.bag_path = None
        self.use_rviz = False
        self.config_path = None  # Path to config file
        self.bag_rate = 0.5  # Default: 0.5x for slower playback
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Bag Mapping"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="Bag Mapping với FAST-LIVO2",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển chính
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Frame chọn bag file
        bag_frame = ttk.LabelFrame(main_frame, text="Chọn Bag File", padding="10")
        bag_frame.pack(fill=tk.X, pady=5)
        
        bag_select_frame = ttk.Frame(bag_frame)
        bag_select_frame.pack(fill=tk.X)
        
        self.bag_path_var = tk.StringVar()
        bag_entry = ttk.Entry(bag_select_frame, textvariable=self.bag_path_var, state="readonly")
        bag_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        browse_btn = ttk.Button(
            bag_select_frame,
            text="Browse...",
            command=self.browse_bag_file
        )
        browse_btn.pack(side=tk.RIGHT)
        
        # Bag info label
        self.bag_info_label = ttk.Label(
            bag_frame,
            text="Chưa chọn bag file",
            foreground="gray"
        )
        self.bag_info_label.pack(pady=5)
        
        # Frame options
        options_frame = ttk.LabelFrame(main_frame, text="Tùy chọn", padding="10")
        options_frame.pack(fill=tk.X, pady=5)
        
        # RViz option
        rviz_frame = ttk.Frame(options_frame)
        rviz_frame.pack(side=tk.LEFT, padx=5)
        self.rviz_var = tk.BooleanVar(value=False)
        rviz_check = ttk.Checkbutton(
            rviz_frame,
            text="Hiển thị RViz2",
            variable=self.rviz_var,
            command=self.on_rviz_change
        )
        rviz_check.pack(side=tk.LEFT)
        
        # Config file selection
        config_frame = ttk.LabelFrame(main_frame, text="Chọn Config File", padding="10")
        config_frame.pack(fill=tk.X, pady=5)
        
        config_select_frame = ttk.Frame(config_frame)
        config_select_frame.pack(fill=tk.X)
        
        self.config_path_var = tk.StringVar()
        config_entry = ttk.Entry(config_select_frame, textvariable=self.config_path_var, state="readonly")
        config_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        browse_config_btn = ttk.Button(
            config_select_frame,
            text="Browse...",
            command=self.browse_config_file
        )
        browse_config_btn.pack(side=tk.RIGHT)
        
        # Quick select from common configs
        quick_config_frame = ttk.Frame(config_frame)
        quick_config_frame.pack(fill=tk.X, pady=5)
        ttk.Label(quick_config_frame, text="Hoặc chọn nhanh:").pack(side=tk.LEFT, padx=5)
        
        self.quick_config_var = tk.StringVar()
        # Find all yaml files in config directory
        config_dir = self.workspace_path / "src" / "FAST-LIVO2" / "config"
        config_files = []
        if config_dir.exists():
            config_files = sorted([f.name for f in config_dir.glob("*.yaml") if f.is_file()])
        
        if not config_files:
            config_files = ["mid360_equirectangular.yaml"]  # Fallback
        
        quick_config_combo = ttk.Combobox(
            quick_config_frame,
            textvariable=self.quick_config_var,
            values=config_files,
            state="readonly",
            width=30
        )
        quick_config_combo.pack(side=tk.LEFT, padx=5)
        quick_config_combo.bind("<<ComboboxSelected>>", self.on_quick_config_select)
        
        # Bag rate selection
        rate_frame = ttk.Frame(options_frame)
        rate_frame.pack(side=tk.LEFT, padx=10)
        ttk.Label(rate_frame, text="Bag Rate:").pack(side=tk.LEFT, padx=5)
        self.rate_var = tk.StringVar(value="0.5")
        rate_combo = ttk.Combobox(
            rate_frame,
            textvariable=self.rate_var,
            values=["0.25", "0.5", "0.75", "1.0", "1.5", "2.0"],
            state="readonly",
            width=10
        )
        rate_combo.pack(side=tk.LEFT)
        rate_combo.bind("<<ComboboxSelected>>", self.on_rate_change)
        
        # Frame điều khiển
        control_frame = ttk.Frame(main_frame, padding="10")
        control_frame.pack(fill=tk.X, pady=10)
        
        self.start_mapping_btn = ttk.Button(
            control_frame,
            text="🚀 Bắt đầu Mapping",
            command=self.start_mapping,
            state=tk.NORMAL
        )
        self.start_mapping_btn.pack(side=tk.LEFT, padx=5)
        
        self.play_bag_btn = ttk.Button(
            control_frame,
            text="▶ Play Bag",
            command=self.start_bag_playback,
            state=tk.DISABLED
        )
        self.play_bag_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="⏹ Dừng",
            command=self.stop_mapping,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Sẵn sàng",
            foreground="green"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Log frame
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=15,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Set default config (after log_text is created)
        self.set_default_config()
        
        # Initial log
        self.log("✅ Bag Mapping Tab đã sẵn sàng")
        self.log("📝 Vui lòng chọn bag file và config file để bắt đầu mapping")
    
    def set_default_config(self):
        """Set default config file"""
        default_config = self.workspace_path / "src" / "FAST-LIVO2" / "config" / "mid360_equirectangular.yaml"
        if default_config.exists():
            self.config_path = str(default_config)
            self.config_path_var.set(str(default_config))
            self.quick_config_var.set("mid360_equirectangular.yaml")
            self.log(f"✅ Đã chọn config mặc định: {default_config.name}")
        else:
            self.log("⚠️ Không tìm thấy config mặc định")
    
    def browse_config_file(self):
        """Chọn config file"""
        config_dir = self.workspace_path / "src" / "FAST-LIVO2" / "config"
        if not config_dir.exists():
            config_dir = Path.home()
        
        config_path = filedialog.askopenfilename(
            title="Chọn Config File",
            initialdir=str(config_dir),
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")]
        )
        
        if config_path:
            config_path_obj = Path(config_path)
            if config_path_obj.exists() and config_path_obj.suffix in ['.yaml', '.yml']:
                self.config_path = str(config_path_obj)
                self.config_path_var.set(str(config_path_obj))
                # Update quick select if it's in the config directory
                config_dir = self.workspace_path / "src" / "FAST-LIVO2" / "config"
                if config_dir.exists() and config_path_obj.parent.samefile(config_dir):
                    self.quick_config_var.set(config_path_obj.name)
                else:
                    self.quick_config_var.set("")
                self.log(f"✅ Đã chọn config: {config_path_obj.name}")
            else:
                messagebox.showerror("Lỗi", f"File không hợp lệ hoặc không tồn tại: {config_path}")
    
    def on_quick_config_select(self, event=None):
        """Xử lý khi chọn config từ quick select"""
        selected = self.quick_config_var.get()
        if selected:
            config_path = self.workspace_path / "src" / "FAST-LIVO2" / "config" / selected
            if config_path.exists():
                self.config_path = str(config_path)
                self.config_path_var.set(str(config_path))
                self.log(f"✅ Đã chọn config: {selected}")
            else:
                messagebox.showerror("Lỗi", f"Config file không tồn tại: {config_path}")
                self.quick_config_var.set("")
    
    
    def browse_bag_file(self):
        """Chọn bag file"""
        initial_dir = "/media/an/ANHSON/"
        bag_path = filedialog.askdirectory(
            title="Chọn Bag Folder",
            initialdir=initial_dir
        )
        
        if bag_path:
            bag_path_obj = Path(bag_path)
            if bag_path_obj.exists():
                self.bag_path_var.set(str(bag_path_obj))
                self.bag_path = str(bag_path_obj)
                self.update_bag_info()
                self.log(f"✅ Đã chọn bag: {bag_path_obj.name}")
            else:
                messagebox.showerror("Lỗi", f"Bag folder không tồn tại: {bag_path}")
    
    def update_bag_info(self):
        """Cập nhật thông tin bag file"""
        if not self.bag_path:
            self.bag_info_label.config(text="Chưa chọn bag file", foreground="gray")
            return
        
        bag_path_obj = Path(self.bag_path)
        try:
            # Thử lấy thông tin bag bằng ros2 bag info
            ws_setup = self.workspace_path / "install" / "setup.sh"
            if ws_setup.exists():
                cmd = f"source {ws_setup} && ros2 bag info {self.bag_path}"
                result = subprocess.run(
                    cmd,
                    shell=True,
                    executable="/bin/bash",
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                
                if result.returncode == 0:
                    # Parse thông tin từ output
                    info_lines = result.stdout.split('\n')
                    duration = "N/A"
                    topics = []
                    
                    for line in info_lines:
                        if 'duration:' in line.lower():
                            # Tìm duration
                            parts = line.split(':')
                            if len(parts) > 1:
                                duration = parts[1].strip()
                        if line.strip().startswith('/'):
                            topics.append(line.strip().split()[0])
                    
                    topics_str = ', '.join(topics[:3]) if topics else "N/A"
                    if len(topics) > 3:
                        topics_str += f" ... (+{len(topics)-3} topics)"
                    
                    info_text = f"📁 {bag_path_obj.name}\n"
                    info_text += f"⏱ Duration: {duration}\n"
                    info_text += f"📡 Topics: {topics_str}"
                    self.bag_info_label.config(text=info_text, foreground="black")
                    self.log("✓ Đã lấy thông tin bag thành công")
                else:
                    self.bag_info_label.config(
                        text=f"📁 {bag_path_obj.name}\n⚠️ Không thể lấy thông tin chi tiết",
                        foreground="orange"
                    )
            else:
                self.bag_info_label.config(
                    text=f"📁 {bag_path_obj.name}",
                    foreground="black"
                )
        except Exception as e:
            self.log(f"⚠️ Lỗi khi lấy thông tin bag: {e}")
            self.bag_info_label.config(
                text=f"📁 {bag_path_obj.name}",
                foreground="black"
            )
    
    def on_rviz_change(self):
        """Callback khi thay đổi RViz option"""
        self.use_rviz = self.rviz_var.get()
        if self.use_rviz:
            self.log("✅ RViz2 sẽ được hiển thị khi mapping")
        else:
            self.log("ℹ️ RViz2 sẽ không được hiển thị")
    
    def on_rate_change(self, event=None):
        """Callback khi thay đổi bag rate"""
        try:
            self.bag_rate = float(self.rate_var.get())
            self.log(f"⚡ Bag rate: {self.bag_rate}x")
        except ValueError:
            self.bag_rate = 0.5
            self.log(f"⚠️ Rate không hợp lệ, sử dụng mặc định: 0.5x")
    
    def log(self, message):
        """Thêm log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def start_mapping(self):
        """Bắt đầu mapping node (không tự động play bag)"""
        if self.is_mapping_running:
            self.log("⚠️ Mapping đã đang chạy")
            return
        
        # Kiểm tra workspace setup
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                "Lỗi",
                f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}\n"
                "Vui lòng build workspace trước."
            )
            return
        
        # Kiểm tra drive_ws (có thể cần cho CustomMsg)
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()
        
        # Kiểm tra config file
        if not self.config_path:
            messagebox.showerror(
                "Lỗi",
                "Vui lòng chọn config file trước khi bắt đầu mapping."
            )
            return
        
        config_path_obj = Path(self.config_path)
        if not config_path_obj.exists():
            messagebox.showerror(
                "Lỗi",
                f"Config file không tồn tại: {self.config_path}\n"
                "Vui lòng chọn lại config file."
            )
            return
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Build command cho mapping launch
            rviz_arg = "True" if self.use_rviz else "False"
            
            # Launch file
            launch_file = "mapping_mid360_equirectangular.launch.py"
            config_name = config_path_obj.name
            
            # Source environment
            if use_drive_ws:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={self.config_path}"
                )
                self.log("✅ Sẽ source: ROS2 base -> drive_ws -> ws")
            else:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={self.config_path}"
                )
                self.log("⚠️ Sẽ source: ROS2 base -> ws (không có drive_ws)")
            
            # Environment
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.log("=" * 60)
            self.log("🚀 Bắt đầu Mapping Node")
            self.log(f"📋 Config file: {config_name}")
            self.log(f"📁 Config path: {self.config_path}")
            self.log(f"🎯 Launch file: {launch_file}")
            self.log(f"👁️ RViz2: {'Có' if self.use_rviz else 'Không'}")
            self.log("=" * 60)
            
            # Start mapping process
            self.log("📡 Đang khởi động mapping node...")
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
            
            # Đợi một chút để mapping node khởi động
            import time
            time.sleep(3)
            
            # Kiểm tra mapping process
            if self.mapping_process.poll() is not None:
                error_output = self.mapping_process.stdout.read() if self.mapping_process.stdout else "Không có output"
                self.log(f"❌ Mapping process đã kết thúc ngay với code: {self.mapping_process.returncode}")
                self.log(f"Output: {error_output[:500]}")
                messagebox.showerror("Lỗi", "Mapping process đã kết thúc ngay. Kiểm tra log để biết chi tiết.")
                self.mapping_process = None
                return
            
            self.is_mapping_running = True
            self.log("✅ Mapping node đã khởi động thành công")
            
            # Update UI
            self.start_mapping_btn.config(state=tk.DISABLED)
            self.play_bag_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: 📡 Mapping node đang chạy",
                foreground="orange"
            )
            
            # Start monitoring thread
            threading.Thread(target=self.monitor_mapping_process, daemon=True).start()
            
            self.log("=" * 60)
            self.log("✅ Mapping node đã sẵn sàng!")
            self.log("💡 Bây giờ bạn có thể click '▶ Play Bag' để bắt đầu playback bag file")
            self.log("=" * 60)
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu mapping: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            self.cleanup_processes()
    
    def start_bag_playback(self):
        """Bắt đầu bag playback (sau khi mapping node đã chạy)"""
        if self.is_bag_playing:
            self.log("⚠️ Bag đã đang được play")
            return
        
        if not self.is_mapping_running:
            messagebox.showwarning("Cảnh báo", "Vui lòng khởi động mapping node trước!")
            return
        
        # Kiểm tra bag path
        if not self.bag_path:
            messagebox.showerror("Lỗi", "Vui lòng chọn bag file trước")
            return
        
        bag_path_obj = Path(self.bag_path)
        if not bag_path_obj.exists():
            messagebox.showerror("Lỗi", f"Bag folder không tồn tại: {self.bag_path}")
            return
        
        # Kiểm tra workspace setup
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                "Lỗi",
                f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}\n"
                "Vui lòng build workspace trước."
            )
            return
        
        # Kiểm tra drive_ws (có thể cần cho CustomMsg)
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Lấy bag rate từ UI
            try:
                bag_rate_value = float(self.rate_var.get())
            except ValueError:
                bag_rate_value = 0.5
                self.log("⚠️ Rate không hợp lệ, sử dụng mặc định: 0.5x")
            
            # Build command cho bag play với rate
            bag_play_cmd = f"ros2 bag play {self.bag_path} --rate {bag_rate_value}"
            
            # Environment
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.log("=" * 60)
            self.log("▶️ Bắt đầu Bag Playback")
            self.log(f"📁 Bag: {bag_path_obj.name}")
            self.log(f"📁 Bag path: {self.bag_path}")
            self.log(f"⚡ Rate: {bag_rate_value}x")
            self.log("=" * 60)
            
            # Start bag play process
            self.log("▶️ Đang khởi động bag playback...")
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
            self.log("✅ Bag playback đã khởi động")
            
            # Update UI
            self.play_bag_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: 🚀 Mapping đang chạy + ▶️ Bag đang play",
                foreground="green"
            )
            
            # Start monitoring thread
            threading.Thread(target=self.monitor_bag_process, daemon=True).start()
            
            self.log("=" * 60)
            self.log("✅ Bag playback đã được khởi động thành công!")
            self.log("💡 Sử dụng nút 'Dừng' để dừng mapping và bag playback")
            self.log("=" * 60)
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu bag playback: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
    
    def monitor_mapping_process(self):
        """Monitor mapping process output"""
        if not self.mapping_process:
            return
        
        try:
            for line in iter(self.mapping_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_mapping_running:
                    # Chỉ log một số dòng quan trọng để tránh spam
                    line_lower = line.lower()
                    if any(keyword in line_lower for keyword in ['error', 'warning', 'started', 'ready', 'failed']):
                        self.log(f"[Mapping] {line.strip()}")
                else:
                    break
        except Exception as e:
            if self.is_mapping_running:
                self.log(f"⚠️ Lỗi khi đọc mapping output: {e}")
    
    def monitor_bag_process(self):
        """Monitor bag process output"""
        if not self.bag_process:
            return
        
        try:
            for line in iter(self.bag_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_bag_playing:
                    # Log bag playback progress
                    if 'paused' in line.lower() or 'playing' in line.lower():
                        self.log(f"[Bag] {line.strip()}")
                else:
                    break
            
            # Bag playback đã kết thúc
            if self.is_bag_playing:
                self.log("✅ Bag playback đã hoàn thành")
                self.is_bag_playing = False
        except Exception as e:
            if self.is_bag_playing:
                self.log(f"⚠️ Lỗi khi đọc bag output: {e}")
    
    def stop_mapping(self):
        """Dừng mapping và bag playback"""
        if not self.is_mapping_running and not self.is_bag_playing:
            return
        
        self.log("=" * 60)
        self.log("⏹ Đang dừng mapping và bag playback...")
        
        self.cleanup_processes()
        
        self.log("✅ Đã dừng tất cả processes")
        self.log("=" * 60)
    
    def call_save_service(self):
        """Gọi ROS service để lưu PCD file bằng ros2 service call"""
        try:
            # Get workspace paths
            ws_setup = self.workspace_path / "install" / "setup.sh"
            drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
            use_drive_ws = drive_ws_setup.exists()
            
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # First check if service exists
            if use_drive_ws:
                check_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 service list | grep -q '/laserMapping/save_results'"
                )
            else:
                check_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 service list | grep -q '/laserMapping/save_results'"
                )
            
            # Check if service exists (quick check, 2 seconds)
            check_result = subprocess.run(
                check_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                timeout=2
            )
            
            if check_result.returncode != 0:
                # Service doesn't exist, skip service call
                return False
            
            # Service exists, call it
            if use_drive_ws:
                service_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"timeout 5 ros2 service call /laserMapping/save_results std_srvs/srv/Trigger"
                )
            else:
                service_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"timeout 5 ros2 service call /laserMapping/save_results std_srvs/srv/Trigger"
                )
            
            # Call service with shorter timeout (5 seconds)
            result = subprocess.run(
                service_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=6
            )
            
            if result.returncode == 0:
                return True
            else:
                # Log error for debugging
                if result.stderr:
                    self.log(f"⚠️ Service call error: {result.stderr[:200]}")
                return False
                
        except subprocess.TimeoutExpired:
            # Timeout is OK, process will save on graceful shutdown
            return False
        except Exception as e:
            self.log(f"⚠️ Lỗi khi gọi save service: {e}")
            return False
    
    def cleanup_processes(self):
        """Dọn dẹp tất cả processes"""
        # Stop bag process
        if self.bag_process:
            try:
                self.log("Đang dừng bag playback...")
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
                self.log(f"⚠️ Lỗi khi dừng bag process: {e}")
            finally:
                self.bag_process = None
                self.is_bag_playing = False
        
        # Stop mapping process
        if self.mapping_process:
            try:
                # Try to save PCD before stopping (optional, process will also save on graceful shutdown)
                self.log("💾 Đang thử lưu PCD file trước khi dừng...")
                save_success = self.call_save_service()
                if save_success:
                    self.log("✅ Đã lưu PCD file qua service")
                    time.sleep(1)  # Wait for file to be written
                else:
                    self.log("💡 Service không khả dụng, process sẽ tự lưu khi dừng gracefully...")
                
                self.log("Đang dừng mapping node...")
                # Send SIGTERM first to allow graceful shutdown
                if hasattr(os, 'setsid'):
                    try:
                        os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                else:
                    self.mapping_process.terminate()
                
                # Wait longer for graceful shutdown (process will call savePCD() at line 782)
                try:
                    self.mapping_process.wait(timeout=10)  # Increased timeout to 10 seconds
                    self.log("✅ Mapping node đã dừng gracefully")
                    
                    # Wait a bit for file I/O to complete
                    time.sleep(1)
                    
                    # Check if PCD files were saved
                    # ROOT_DIR is defined as CMAKE_CURRENT_SOURCE_DIR which is ws/src/FAST-LIVO2/
                    # self.workspace_path is already ws/, so we use it directly
                    pcd_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "PCD"
                    raw_pcd = pcd_dir / "all_raw_points.pcd"
                    downsampled_pcd = pcd_dir / "all_downsampled_points.pcd"
                    
                    if raw_pcd.exists() or downsampled_pcd.exists():
                        self.log(f"✅ PCD files đã được lưu tại: {pcd_dir}")
                        if raw_pcd.exists():
                            size_mb = raw_pcd.stat().st_size / (1024 * 1024)
                            self.log(f"   - all_raw_points.pcd: {size_mb:.1f} MB")
                        if downsampled_pcd.exists():
                            size_mb = downsampled_pcd.stat().st_size / (1024 * 1024)
                            self.log(f"   - all_downsampled_points.pcd: {size_mb:.1f} MB")
                    else:
                        self.log(f"⚠️ Không tìm thấy PCD files tại: {pcd_dir}")
                        self.log(f"   (Đường dẫn đã kiểm tra: {pcd_dir.absolute()})")
                        # Try to list what's in the directory for debugging
                        if pcd_dir.exists():
                            files = list(pcd_dir.glob("*.pcd"))
                            if files:
                                self.log(f"   Tìm thấy {len(files)} file .pcd khác trong thư mục")
                            else:
                                self.log(f"   Thư mục trống hoặc không có file .pcd")
                        else:
                            self.log(f"   Thư mục không tồn tại")
                        
                except subprocess.TimeoutExpired:
                    # If still running after 10 seconds, force kill
                    self.log("⚠️ Process chưa dừng sau 10s, đang force kill...")
                    if hasattr(os, 'setsid'):
                        try:
                            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        self.mapping_process.kill()
                    self.mapping_process.wait()
            except Exception as e:
                self.log(f"⚠️ Lỗi khi dừng mapping process: {e}")
            finally:
                self.mapping_process = None
                self.is_mapping_running = False
        
        # Update UI
        self.start_mapping_btn.config(state=tk.NORMAL)
        self.play_bag_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="red"
        )

