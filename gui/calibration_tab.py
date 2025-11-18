#!/usr/bin/env python3
"""
Calibration Tab Module
Chứa CalibrationTab để thực hiện LiDAR-Camera calibration
"""

import threading
import subprocess
import json
from pathlib import Path
from datetime import datetime
import os
from functools import partial

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class CalibrationTab(ttk.Frame):
    """Tab cho LiDAR-Camera Calibration"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "drive_ws"
        
        # Processes
        self.record_process = None
        self.preprocess_process = None
        self.initial_guess_process = None
        self.calibrate_process = None
        
        # Paths
        self.bag_output_dir = None
        self.preprocessed_dir = None
        
        # State
        self.is_recording = False
        self.current_step = "idle"  # idle, recording, preprocessing, calibrating
        
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widgets cho calibration tab"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="LiDAR-Camera Calibration",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Main container với notebook để chia thành các bước
        notebook = ttk.Notebook(self)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 1: Record Bag
        record_frame = ttk.Frame(notebook)
        notebook.add(record_frame, text="1. Record Bag")
        self.create_record_tab(record_frame)
        
        # Tab 2: Preprocessing
        preprocess_frame = ttk.Frame(notebook)
        notebook.add(preprocess_frame, text="2. Preprocessing")
        self.create_preprocess_tab(preprocess_frame)
        
        # Tab 3: Initial Guess
        initial_guess_frame = ttk.Frame(notebook)
        notebook.add(initial_guess_frame, text="3. Initial Guess")
        self.create_initial_guess_tab(initial_guess_frame)
        
        # Tab 4: Calibration
        calibrate_frame = ttk.Frame(notebook)
        notebook.add(calibrate_frame, text="4. Calibration")
        self.create_calibrate_tab(calibrate_frame)
        
        # Tab 5: Export Results
        export_frame = ttk.Frame(notebook)
        notebook.add(export_frame, text="5. Export Results")
        self.create_export_tab(export_frame)
        
        # Status bar
        self.status_label = ttk.Label(
            self,
            text="Trạng thái: Sẵn sàng",
            font=("Arial", 10)
        )
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
    
    def create_record_tab(self, parent):
        """Tạo tab record bag"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Ghi lại rosbag với topics /image_raw (equirectangular) và /livox/points2 hoặc /livox/lidar",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Output directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(dir_frame, text="Thư mục output:").pack(side=tk.LEFT, padx=5)
        self.bag_dir_var = tk.StringVar(value=str(self.workspace_path / "calibration_data" / "bags"))
        dir_entry = ttk.Entry(dir_frame, textvariable=self.bag_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            dir_frame,
            text="Browse",
            command=self.browse_bag_directory
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Topics to record
        topics_frame = ttk.LabelFrame(parent, text="Topics", padding=10)
        topics_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.topics_var = tk.StringVar(value="/image_raw /livox/points2")
        topics_entry = ttk.Entry(topics_frame, textvariable=self.topics_var, width=60)
        topics_entry.pack(pady=5)
        
        ttk.Label(
            topics_frame,
            text="Các topics sẽ được record (cách nhau bởi khoảng trắng)",
            font=("Arial", 9),
            foreground="gray"
        ).pack()
        
        # Record button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.record_btn = ttk.Button(
            button_frame,
            text="Bắt đầu Record",
            command=self.start_record,
            style="Accent.TButton"
        )
        self.record_btn.pack(side=tk.LEFT, padx=10)
        
        self.stop_record_btn = ttk.Button(
            button_frame,
            text="Dừng Record",
            command=self.stop_record,
            state=tk.DISABLED
        )
        self.stop_record_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.record_log = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.record_log.pack(fill=tk.BOTH, expand=True)
    
    def create_preprocess_tab(self, parent):
        """Tạo tab preprocessing"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Preprocessing dữ liệu từ rosbag để chuẩn bị cho calibration",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Input bag directory
        input_frame = ttk.Frame(parent)
        input_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(input_frame, text="Thư mục bags:").pack(side=tk.LEFT, padx=5)
        self.preprocess_input_var = tk.StringVar()
        input_entry = ttk.Entry(input_frame, textvariable=self.preprocess_input_var, width=50)
        input_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_input_btn = ttk.Button(
            input_frame,
            text="Browse",
            command=self.browse_preprocess_input
        )
        browse_input_btn.pack(side=tk.LEFT, padx=5)
        
        # Output preprocessed directory
        output_frame = ttk.Frame(parent)
        output_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(output_frame, text="Thư mục output:").pack(side=tk.LEFT, padx=5)
        self.preprocess_output_var = tk.StringVar(value=str(self.workspace_path / "calibration_data" / "preprocessed"))
        output_entry = ttk.Entry(output_frame, textvariable=self.preprocess_output_var, width=50)
        output_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_output_btn = ttk.Button(
            output_frame,
            text="Browse",
            command=self.browse_preprocess_output
        )
        browse_output_btn.pack(side=tk.LEFT, padx=5)
        
        # Camera model selection
        camera_model_frame = ttk.LabelFrame(parent, text="Camera Model", padding=10)
        camera_model_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(
            camera_model_frame,
            text="Chọn loại camera model:",
            font=("Arial", 9)
        ).pack(anchor=tk.W, pady=5)
        
        self.camera_model_var = tk.StringVar(value="equirectangular")
        camera_model_options = [
            ("Equirectangular (360°)", "equirectangular"),
            ("Pinhole/Plumb Bob", "plumb_bob"),
            ("Fisheye/Equidistant", "fisheye"),
            ("Omnidirectional", "omnidir"),
            ("Auto-detect", "auto")
        ]
        
        for text, value in camera_model_options:
            ttk.Radiobutton(
                camera_model_frame,
                text=text,
                variable=self.camera_model_var,
                value=value
            ).pack(anchor=tk.W, padx=20)
        
        ttk.Label(
            camera_model_frame,
            text="Lưu ý: Với equirectangular, intrinsics sẽ tự động được tính từ image size",
            font=("Arial", 8),
            foreground="gray"
        ).pack(anchor=tk.W, pady=5)
        
        # Options
        options_frame = ttk.LabelFrame(parent, text="Options", padding=10)
        options_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.auto_topic_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            options_frame,
            text="Auto-detect topics (-a)",
            variable=self.auto_topic_var
        ).pack(anchor=tk.W)
        
        self.dynamic_lidar_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            options_frame,
            text="Dynamic LiDAR integration (-d) - cho spinning LiDAR",
            variable=self.dynamic_lidar_var
        ).pack(anchor=tk.W)
        
        self.visualize_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            options_frame,
            text="Visualize (-v)",
            variable=self.visualize_var
        ).pack(anchor=tk.W)
        
        # Preprocess button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.preprocess_btn = ttk.Button(
            button_frame,
            text="Chạy Preprocessing",
            command=self.start_preprocess,
            style="Accent.TButton"
        )
        self.preprocess_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.preprocess_log = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.preprocess_log.pack(fill=tk.BOTH, expand=True)
    
    def create_initial_guess_tab(self, parent):
        """Tạo tab initial guess"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Tạo initial guess cho calibration (manual hoặc automatic)",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Preprocessed directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(dir_frame, text="Thư mục preprocessed:").pack(side=tk.LEFT, padx=5)
        self.initial_guess_dir_var = tk.StringVar()
        dir_entry = ttk.Entry(dir_frame, textvariable=self.initial_guess_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            dir_frame,
            text="Browse",
            command=self.browse_initial_guess_dir
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Mode selection
        mode_frame = ttk.LabelFrame(parent, text="Mode", padding=10)
        mode_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.initial_guess_mode = tk.StringVar(value="manual")
        
        ttk.Radiobutton(
            mode_frame,
            text="Manual - Chọn correspondences thủ công",
            variable=self.initial_guess_mode,
            value="manual"
        ).pack(anchor=tk.W, pady=5)
        
        ttk.Radiobutton(
            mode_frame,
            text="Automatic - Sử dụng SuperGlue (cần license)",
            variable=self.initial_guess_mode,
            value="auto"
        ).pack(anchor=tk.W, pady=5)
        
        # Buttons
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.initial_guess_btn = ttk.Button(
            button_frame,
            text="Chạy Initial Guess",
            command=self.start_initial_guess,
            style="Accent.TButton"
        )
        self.initial_guess_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.initial_guess_log = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.initial_guess_log.pack(fill=tk.BOTH, expand=True)
    
    def create_calibrate_tab(self, parent):
        """Tạo tab calibration"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Chạy fine registration để tinh chỉnh calibration",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Preprocessed directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(dir_frame, text="Thư mục preprocessed:").pack(side=tk.LEFT, padx=5)
        self.calibrate_dir_var = tk.StringVar()
        dir_entry = ttk.Entry(dir_frame, textvariable=self.calibrate_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            dir_frame,
            text="Browse",
            command=self.browse_calibrate_dir
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Options
        options_frame = ttk.LabelFrame(parent, text="Options", padding=10)
        options_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.auto_quit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            options_frame,
            text="Auto quit sau khi calibration xong (khuyến nghị)",
            variable=self.auto_quit_var
        ).pack(anchor=tk.W)
        
        self.background_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            options_frame,
            text="Chạy background (không hiển thị viewer) - khuyến nghị",
            variable=self.background_var
        ).pack(anchor=tk.W)
        
        # Registration type
        reg_frame = ttk.Frame(options_frame)
        reg_frame.pack(fill=tk.X, pady=5)
        ttk.Label(reg_frame, text="Registration type:").pack(side=tk.LEFT, padx=5)
        self.registration_type_var = tk.StringVar(value="nid_bfgs")
        ttk.Radiobutton(
            reg_frame,
            text="NID-BFGS (nhanh hơn, khuyến nghị)",
            variable=self.registration_type_var,
            value="nid_bfgs"
        ).pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(
            reg_frame,
            text="NID-Nelder-Mead (chậm hơn nhưng ổn định hơn)",
            variable=self.registration_type_var,
            value="nid_nelder_mead"
        ).pack(side=tk.LEFT, padx=5)
        
        # Button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.calibrate_btn = ttk.Button(
            button_frame,
            text="Chạy Calibration",
            command=self.start_calibrate,
            style="Accent.TButton"
        )
        self.calibrate_btn.pack(side=tk.LEFT, padx=10)
        
        self.stop_calibrate_btn = ttk.Button(
            button_frame,
            text="Dừng Calibration",
            command=self.stop_calibrate,
            state=tk.DISABLED
        )
        self.stop_calibrate_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.calibrate_log = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.calibrate_log.pack(fill=tk.BOTH, expand=True)
    
    def create_export_tab(self, parent):
        """Tạo tab export results"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Export và convert kết quả calibration sang format FAST-LIVO2",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Calib.json path
        calib_frame = ttk.Frame(parent)
        calib_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(calib_frame, text="File calib.json:").pack(side=tk.LEFT, padx=5)
        self.calib_json_var = tk.StringVar()
        calib_entry = ttk.Entry(calib_frame, textvariable=self.calib_json_var, width=50)
        calib_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_calib_btn = ttk.Button(
            calib_frame,
            text="Browse",
            command=self.browse_calib_json
        )
        browse_calib_btn.pack(side=tk.LEFT, padx=5)
        
        # Output YAML path
        yaml_frame = ttk.Frame(parent)
        yaml_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(yaml_frame, text="Output YAML:").pack(side=tk.LEFT, padx=5)
        self.output_yaml_var = tk.StringVar()
        yaml_entry = ttk.Entry(yaml_frame, textvariable=self.output_yaml_var, width=50)
        yaml_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Buttons
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.convert_btn = ttk.Button(
            button_frame,
            text="Convert sang FAST-LIVO2",
            command=self.convert_to_fast_livo2,
            style="Accent.TButton"
        )
        self.convert_btn.pack(side=tk.LEFT, padx=10)
        
        self.view_calib_btn = ttk.Button(
            button_frame,
            text="Xem calib.json",
            command=self.view_calib_json
        )
        self.view_calib_btn.pack(side=tk.LEFT, padx=10)
        
        # Results display
        results_frame = ttk.LabelFrame(parent, text="Kết quả", padding=10)
        results_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.results_text = scrolledtext.ScrolledText(
            results_frame,
            height=10,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.results_text.pack(fill=tk.BOTH, expand=True)
    
    # ========== Record Bag Methods ==========
    
    def browse_bag_directory(self):
        """Browse cho bag output directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục để lưu bag",
            initialdir=self.bag_dir_var.get()
        )
        if directory:
            self.bag_dir_var.set(directory)
    
    def start_record(self):
        """Bắt đầu record rosbag"""
        if self.is_recording:
            messagebox.showwarning("Cảnh báo", "Đang record, vui lòng dừng trước")
            return
        
        bag_dir = Path(self.bag_dir_var.get())
        if not bag_dir.exists():
            try:
                bag_dir.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể tạo thư mục: {e}")
                return
        
        topics = self.topics_var.get().split()
        if not topics:
            messagebox.showerror("Lỗi", "Vui lòng nhập ít nhất một topic")
            return
        
        # Tạo tên bag file với timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"calibration_{timestamp}"
        bag_path = bag_dir / bag_name
        
        # Build command
        cmd = f"ros2 bag record -o {bag_path} {' '.join(topics)}"
        
        self.log_record(f"Bắt đầu record bag: {bag_name}")
        self.log_record(f"Topics: {', '.join(topics)}")
        self.log_record(f"Output: {bag_path}")
        
        try:
            self.record_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            self.is_recording = True
            self.record_btn.config(state=tk.DISABLED)
            self.stop_record_btn.config(state=tk.NORMAL)
            self.status_label.config(text=f"Trạng thái: Đang record bag...", foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_record_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể bắt đầu record: {e}")
            self.log_record(f"Lỗi: {e}")
    
    def stop_record(self):
        """Dừng record rosbag"""
        if self.record_process:
            try:
                self.record_process.terminate()
                self.record_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.record_process.kill()
            except Exception as e:
                self.log_record(f"Lỗi khi dừng record: {e}")
            
            self.record_process = None
            self.is_recording = False
            self.record_btn.config(state=tk.NORMAL)
            self.stop_record_btn.config(state=tk.DISABLED)
            self.status_label.config(text="Trạng thái: Đã dừng record", foreground="green")
            self.log_record("Đã dừng record")
    
    def monitor_record_process(self):
        """Monitor record process output"""
        if not self.record_process:
            return
        
        for line in iter(self.record_process.stdout.readline, ''):
            if not line:
                break
            self.log_record(line.strip())
        
        if self.record_process.poll() is not None:
            self.is_recording = False
            self.after(0, partial(self._update_record_complete))
    
    def _update_record_complete(self):
        """Helper function để update UI sau khi record hoàn thành"""
        try:
            self.record_btn.config(state=tk.NORMAL)
            self.stop_record_btn.config(state=tk.DISABLED)
            self.status_label.config(text="Trạng thái: Record đã hoàn thành", foreground="green")
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_record(self, message):
        """Log message vào record log (thread-safe)"""
        self.after(0, partial(self._log_record_impl, message))
    
    def _log_record_impl(self, message):
        """Implementation của log_record (chạy trong main thread)"""
        try:
            self.record_log.config(state=tk.NORMAL)
            self.record_log.insert(tk.END, f"{message}\n")
            self.record_log.see(tk.END)
            self.record_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Preprocessing Methods ==========
    
    def browse_preprocess_input(self):
        """Browse cho preprocess input directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục chứa bags",
            initialdir=self.preprocess_input_var.get() or str(self.workspace_path)
        )
        if directory:
            self.preprocess_input_var.set(directory)
    
    def browse_preprocess_output(self):
        """Browse cho preprocess output directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục output",
            initialdir=self.preprocess_output_var.get() or str(self.workspace_path)
        )
        if directory:
            self.preprocess_output_var.set(directory)
    
    def start_preprocess(self):
        """Chạy preprocessing"""
        input_dir = self.preprocess_input_var.get()
        output_dir = self.preprocess_output_var.get()
        
        if not input_dir or not Path(input_dir).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn thư mục input hợp lệ")
            return
        
        if not output_dir:
            messagebox.showerror("Lỗi", "Vui lòng chọn thư mục output")
            return
        
        # Tạo output directory nếu chưa có
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # Build command
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy setup.sh tại: {setup_script}")
            return
        
        cmd_parts = ["ros2", "run", "direct_visual_lidar_calibration", "preprocess"]
        
        # Add camera model option
        camera_model = self.camera_model_var.get()
        if camera_model and camera_model != "auto":
            cmd_parts.extend(["--camera_model", camera_model])
        
        if self.auto_topic_var.get():
            cmd_parts.append("-a")
        if self.dynamic_lidar_var.get():
            cmd_parts.append("-d")
        if self.visualize_var.get():
            cmd_parts.append("-v")
        
        cmd_parts.extend([input_dir, output_dir])
        
        # Build command với đầy đủ ROS2 environment
        # Cần source cả ROS2 base và workspace setup
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        cmd = f"source {ros2_setup} && source {setup_script} && {' '.join(cmd_parts)}"
        
        self.log_preprocess(f"Bắt đầu preprocessing...")
        self.log_preprocess(f"Input: {input_dir}")
        self.log_preprocess(f"Output: {output_dir}")
        self.log_preprocess(f"Command: {' '.join(cmd_parts)}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            # Đảm bảo không có ROS_DOMAIN_ID conflict
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.preprocess_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.preprocess_btn.config(state=tk.DISABLED)
            self.status_label.config(text="Trạng thái: Đang preprocessing...", foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_preprocess_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể chạy preprocessing: {e}")
            self.log_preprocess(f"Lỗi: {e}")
    
    def monitor_preprocess_process(self):
        """Monitor preprocess process output"""
        if not self.preprocess_process:
            return
        
        try:
            for line in iter(self.preprocess_process.stdout.readline, ''):
                if not line:
                    break
                self.log_preprocess(line.strip())
        except Exception as e:
            self.log_preprocess(f"Lỗi khi đọc output: {e}")
        
        if self.preprocess_process.poll() is not None:
            exit_code = self.preprocess_process.poll()
            if exit_code == 0:
                # Sử dụng helper functions thay vì lambda để tránh closure issues
                self.after(0, partial(self._update_preprocess_success))
            else:
                self.after(0, partial(self._update_preprocess_failure))
    
    def _update_preprocess_success(self):
        """Helper function để update UI sau khi preprocessing thành công"""
        try:
            self.status_label.config(text="Trạng thái: Preprocessing hoàn thành", foreground="green")
            self.preprocess_btn.config(state=tk.NORMAL)
            # Auto-fill initial guess directory
            output_dir = self.preprocess_output_var.get()
            self.initial_guess_dir_var.set(output_dir)
            self.calibrate_dir_var.set(output_dir)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_preprocess_failure(self):
        """Helper function để update UI sau khi preprocessing thất bại"""
        try:
            self.status_label.config(text="Trạng thái: Preprocessing thất bại", foreground="red")
            self.preprocess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_preprocess(self, message):
        """Log message vào preprocess log (thread-safe)"""
        # Sử dụng after để đảm bảo thread-safe
        self.after(0, partial(self._log_preprocess_impl, message))
    
    def _log_preprocess_impl(self, message):
        """Implementation của log_preprocess (chạy trong main thread)"""
        try:
            self.preprocess_log.config(state=tk.NORMAL)
            self.preprocess_log.insert(tk.END, f"{message}\n")
            self.preprocess_log.see(tk.END)
            self.preprocess_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Initial Guess Methods ==========
    
    def browse_initial_guess_dir(self):
        """Browse cho initial guess directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục preprocessed",
            initialdir=self.initial_guess_dir_var.get() or str(self.workspace_path)
        )
        if directory:
            self.initial_guess_dir_var.set(directory)
    
    def start_initial_guess(self):
        """Chạy initial guess"""
        preprocessed_dir = self.initial_guess_dir_var.get()
        
        if not preprocessed_dir or not Path(preprocessed_dir).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn thư mục preprocessed hợp lệ")
            return
        
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy setup.sh tại: {setup_script}")
            return
        
        mode = self.initial_guess_mode.get()
        
        if mode == "manual":
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            cmd = f"source {ros2_setup} && source {setup_script} && ros2 run direct_visual_lidar_calibration initial_guess_manual {preprocessed_dir}"
        else:  # auto
            # Cần chạy find_matches_superglue trước
            messagebox.showinfo("Thông tin", "Automatic mode cần SuperGlue. Vui lòng chạy find_matches_superglue trước.")
            return
        
        self.log_initial_guess(f"Bắt đầu initial guess ({mode})...")
        self.log_initial_guess(f"Directory: {preprocessed_dir}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.initial_guess_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.initial_guess_btn.config(state=tk.DISABLED)
            self.status_label.config(text="Trạng thái: Đang chạy initial guess...", foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_initial_guess_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể chạy initial guess: {e}")
            self.log_initial_guess(f"Lỗi: {e}")
    
    def monitor_initial_guess_process(self):
        """Monitor initial guess process output"""
        if not self.initial_guess_process:
            return
        
        try:
            for line in iter(self.initial_guess_process.stdout.readline, ''):
                if not line:
                    break
                self.log_initial_guess(line.strip())
        except Exception as e:
            self.log_initial_guess(f"Lỗi khi đọc output: {e}")
        
        if self.initial_guess_process.poll() is not None:
            exit_code = self.initial_guess_process.poll()
            if exit_code == 0:
                self.after(0, partial(self._update_initial_guess_success))
            else:
                self.after(0, partial(self._update_initial_guess_failure))
    
    def _update_initial_guess_success(self):
        """Helper function để update UI sau khi initial guess thành công"""
        try:
            self.status_label.config(text="Trạng thái: Initial guess hoàn thành", foreground="green")
            self.initial_guess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_initial_guess_failure(self):
        """Helper function để update UI sau khi initial guess thất bại"""
        try:
            self.status_label.config(text="Trạng thái: Initial guess thất bại", foreground="red")
            self.initial_guess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_initial_guess(self, message):
        """Log message vào initial guess log (thread-safe)"""
        self.after(0, partial(self._log_initial_guess_impl, message))
    
    def _log_initial_guess_impl(self, message):
        """Implementation của log_initial_guess (chạy trong main thread)"""
        try:
            self.initial_guess_log.config(state=tk.NORMAL)
            self.initial_guess_log.insert(tk.END, f"{message}\n")
            self.initial_guess_log.see(tk.END)
            self.initial_guess_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Calibration Methods ==========
    
    def browse_calibrate_dir(self):
        """Browse cho calibrate directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục preprocessed",
            initialdir=self.calibrate_dir_var.get() or str(self.workspace_path)
        )
        if directory:
            self.calibrate_dir_var.set(directory)
    
    def start_calibrate(self):
        """Chạy calibration"""
        preprocessed_dir = self.calibrate_dir_var.get()
        
        if not preprocessed_dir or not Path(preprocessed_dir).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn thư mục preprocessed hợp lệ")
            return
        
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy setup.sh tại: {setup_script}")
            return
        
        cmd_parts = ["ros2", "run", "direct_visual_lidar_calibration", "calibrate", preprocessed_dir]
        
        # Add registration type
        registration_type = self.registration_type_var.get()
        cmd_parts.extend(["--registration_type", registration_type])
        
        if self.auto_quit_var.get():
            cmd_parts.append("--auto_quit")
        if self.background_var.get():
            cmd_parts.append("--background")
        
        # Build command với đầy đủ ROS2 environment
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        cmd = f"source {ros2_setup} && source {setup_script} && {' '.join(cmd_parts)}"
        
        self.log_calibrate(f"Bắt đầu calibration...")
        self.log_calibrate(f"Directory: {preprocessed_dir}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            # Đảm bảo không có ROS_DOMAIN_ID conflict
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.calibrate_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.calibrate_btn.config(state=tk.DISABLED)
            self.stop_calibrate_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Trạng thái: Đang calibration...", foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_calibrate_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể chạy calibration: {e}")
            self.log_calibrate(f"Lỗi: {e}")
    
    def monitor_calibrate_process(self):
        """Monitor calibrate process output"""
        if not self.calibrate_process:
            return
        
        try:
            for line in iter(self.calibrate_process.stdout.readline, ''):
                if not line:
                    break
                self.log_calibrate(line.strip())
        except Exception as e:
            self.log_calibrate(f"Lỗi khi đọc output: {e}")
        
        if self.calibrate_process.poll() is not None:
            exit_code = self.calibrate_process.poll()
            if exit_code == 0:
                self.after(0, partial(self._update_calibrate_success))
            else:
                self.after(0, partial(self._update_calibrate_failure))
    
    def stop_calibrate(self):
        """Dừng calibration process"""
        if self.calibrate_process:
            try:
                self.log_calibrate("Đang dừng calibration...")
                self.calibrate_process.terminate()
                self.calibrate_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    self.calibrate_process.kill()
                except:
                    pass
            except Exception as e:
                self.log_calibrate(f"Lỗi khi dừng calibration: {e}")
            finally:
                self.calibrate_process = None
                self.calibrate_btn.config(state=tk.NORMAL)
                self.stop_calibrate_btn.config(state=tk.DISABLED)
                self.status_label.config(text="Trạng thái: Calibration đã dừng", foreground="orange")
                self.log_calibrate("Calibration đã dừng")
    
    def _update_calibrate_success(self):
        """Helper function để update UI sau khi calibration thành công"""
        try:
            self.status_label.config(text="Trạng thái: Calibration hoàn thành", foreground="green")
            self.calibrate_btn.config(state=tk.NORMAL)
            self.stop_calibrate_btn.config(state=tk.DISABLED)
            # Auto-fill calib.json path
            calib_json_path = Path(self.calibrate_dir_var.get()) / "calib.json"
            if calib_json_path.exists():
                self.calib_json_var.set(str(calib_json_path))
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_calibrate_failure(self):
        """Helper function để update UI sau khi calibration thất bại"""
        try:
            self.status_label.config(text="Trạng thái: Calibration thất bại", foreground="red")
            self.calibrate_btn.config(state=tk.NORMAL)
            self.stop_calibrate_btn.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_calibrate(self, message):
        """Log message vào calibrate log (thread-safe)"""
        self.after(0, partial(self._log_calibrate_impl, message))
    
    def _log_calibrate_impl(self, message):
        """Implementation của log_calibrate (chạy trong main thread)"""
        try:
            self.calibrate_log.config(state=tk.NORMAL)
            self.calibrate_log.insert(tk.END, f"{message}\n")
            self.calibrate_log.see(tk.END)
            self.calibrate_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Export Methods ==========
    
    def browse_calib_json(self):
        """Browse cho calib.json file"""
        file_path = filedialog.askopenfilename(
            title="Chọn file calib.json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.calib_json_var.get() or str(self.workspace_path)
        )
        if file_path:
            self.calib_json_var.set(file_path)
    
    def view_calib_json(self):
        """Xem nội dung calib.json"""
        calib_json_path = self.calib_json_var.get()
        
        if not calib_json_path or not Path(calib_json_path).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn file calib.json hợp lệ")
            return
        
        try:
            with open(calib_json_path, 'r') as f:
                calib_data = json.load(f)
            
            self.results_text.config(state=tk.NORMAL)
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, json.dumps(calib_data, indent=2))
            self.results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể đọc file: {e}")
    
    def convert_to_fast_livo2(self):
        """Convert calib.json sang FAST-LIVO2 format"""
        calib_json_path = self.calib_json_var.get()
        
        if not calib_json_path or not Path(calib_json_path).exists():
            messagebox.showerror("Lỗi", "Vui lòng chọn file calib.json hợp lệ")
            return
        
        # Sử dụng script conversion đã tạo
        convert_script = self.workspace_path / "src" / "FAST-LIVO2" / "scripts" / "convert_calib_to_fast_livo2.py"
        
        if not convert_script.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy script conversion tại: {convert_script}")
            return
        
        output_yaml = self.output_yaml_var.get()
        if not output_yaml:
            # Tạo output path mặc định
            output_yaml = str(Path(calib_json_path).parent / "fast_livo2_calib.yaml")
            self.output_yaml_var.set(output_yaml)
        
        cmd = f"python3 {convert_script} {calib_json_path} --output {output_yaml}"
        
        try:
            result = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                # Đọc kết quả từ output
                self.results_text.config(state=tk.NORMAL)
                self.results_text.delete(1.0, tk.END)
                self.results_text.insert(tk.END, result.stdout)
                self.results_text.config(state=tk.DISABLED)
                
                messagebox.showinfo("Thành công", f"Đã convert thành công!\nKết quả đã lưu tại: {output_yaml}")
            else:
                messagebox.showerror("Lỗi", f"Convert thất bại:\n{result.stderr}")
                self.results_text.config(state=tk.NORMAL)
                self.results_text.delete(1.0, tk.END)
                self.results_text.insert(tk.END, result.stderr)
                self.results_text.config(state=tk.DISABLED)
                
        except subprocess.TimeoutExpired:
            messagebox.showerror("Lỗi", "Convert timeout")
        except Exception as e:
            messagebox.showerror("Lỗi", f"Lỗi khi convert: {e}")

