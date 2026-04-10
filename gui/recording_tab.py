#!/usr/bin/env python3
"""
Recording Tab Module
Chứa RecordingTab để record các ROS2 topics
"""

import threading
import subprocess
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


class RecordingTab(ttk.Frame):
    """Tab cho Recording ROS2 topics"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
        
        # Process
        self.record_process = None
        
        # State
        self.is_recording = False
        self.output_dir = None
        self.max_bag_size_gb = 2  # Kích thước tối đa mỗi bag file (GB)
        
        # Topics to record với mô tả
        self.topics_config = {
            "/livox/lidar": {
                "description": "CustomMsg - cần source drive_ws/install/setup.sh",
                "enabled": True
            },
            "/livox/points2": {
                "description": "PointCloud2",
                "enabled": True
            },
            "/livox/imu": {
                "description": "Imu",
                "enabled": True
            },
            "/image_perspective": {
                "description": "Image (Perspective)",
                "enabled": True
            },
            "/image_raw": {
                "description": "Image (Raw/Equirectangular)",
                "enabled": True
            },
            "/camera_info": {
                "description": "CameraInfo",
                "enabled": True
            }
        }
        
        # Dictionary để lưu checkbox variables
        self.topic_vars = {}
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Recording"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="ROS2 Topic Recording",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Frame chọn thư mục output
        dir_frame = ttk.LabelFrame(control_frame, text="Thư mục lưu", padding="10")
        dir_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.output_dir_var = tk.StringVar(value=str(self.workspace_path / "recordings"))
        dir_entry = ttk.Entry(dir_frame, textvariable=self.output_dir_var, width=60)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            dir_frame,
            text="Browse",
            command=self.browse_output_directory
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame topics với checkboxes
        topics_frame = ttk.LabelFrame(control_frame, text="Chọn Topics để record", padding="10")
        topics_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Tạo checkboxes cho từng topic
        for topic, config in self.topics_config.items():
            topic_row = ttk.Frame(topics_frame)
            topic_row.pack(fill=tk.X, padx=5, pady=2)
            
            # Checkbox variable
            var = tk.BooleanVar(value=config["enabled"])
            self.topic_vars[topic] = var
            
            # Checkbox
            checkbox = ttk.Checkbutton(
                topic_row,
                text=topic,
                variable=var,
                width=25
            )
            checkbox.pack(side=tk.LEFT, padx=5)
            
            # Description
            desc_label = ttk.Label(
                topic_row,
                text=f"({config['description']})",
                font=("Arial", 9),
                foreground="gray"
            )
            desc_label.pack(side=tk.LEFT, padx=5)
        
        # Lưu ý về CustomMsg
        note_label = ttk.Label(
            topics_frame,
            text="⚠️  Lưu ý: /livox/lidar (CustomMsg) cần source drive_ws/install/setup.sh trước khi record",
            font=("Arial", 9),
            foreground="orange"
        )
        note_label.pack(anchor=tk.W, padx=5, pady=5)
        
        # Frame cấu hình bag size
        bag_size_frame = ttk.LabelFrame(control_frame, text="Cấu hình Bag File", padding="10")
        bag_size_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(
            bag_size_frame,
            text="Kích thước tối đa mỗi bag file (GB):",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.max_bag_size_var = tk.StringVar(value="2")
        bag_size_spinbox = ttk.Spinbox(
            bag_size_frame,
            from_=0.5,
            to=10.0,
            increment=0.5,
            textvariable=self.max_bag_size_var,
            width=10,
            format="%.1f"
        )
        bag_size_spinbox.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(
            bag_size_frame,
            text="(0 = không giới hạn, sẽ tự động chia nhỏ khi đạt giới hạn)",
            font=("Arial", 9),
            foreground="gray"
        ).pack(side=tk.LEFT, padx=10)
        
        # Frame nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.start_btn = ttk.Button(
            button_frame,
            text="Start Recording",
            command=self.start_recording,
            style="Accent.TButton"
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            button_frame,
            text="Stop Recording",
            command=self.stop_recording,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            button_frame,
            text="Trạng thái: Sẵn sàng",
            foreground="gray"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin recording
        info_frame = ttk.LabelFrame(self, text="Thông tin Recording", padding="10")
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.info_label = ttk.Label(
            info_frame,
            text="Chưa bắt đầu recording",
            font=("Arial", 10)
        )
        self.info_label.pack(anchor=tk.W, padx=5)
        
        # Text area để hiển thị log
        log_frame = ttk.LabelFrame(self, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=15,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def browse_output_directory(self):
        """Browse cho output directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục để lưu recording",
            initialdir=self.output_dir_var.get()
        )
        if directory:
            self.output_dir_var.set(directory)
            self.log(f"Đã chọn thư mục: {directory}")
    
    def _verify_source(self, setup_script, name):
        """Verify rằng setup script có thể source được"""
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            # Chạy một command đơn giản để verify source
            verify_cmd = f"source {ros2_setup} && source {setup_script} && ros2 pkg list | head -1"
            result = subprocess.run(
                verify_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            self.log(f"⚠️  Lỗi khi verify source {name}: {e}")
            return False
    
    def start_recording(self):
        """Bắt đầu record rosbag"""
        if self.is_recording:
            messagebox.showwarning("Cảnh báo", "Đang record, vui lòng dừng trước")
            return
        
        # Kiểm tra thư mục output
        output_dir = Path(self.output_dir_var.get())
        if not output_dir.exists():
            try:
                output_dir.mkdir(parents=True, exist_ok=True)
                self.log(f"Đã tạo thư mục: {output_dir}")
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể tạo thư mục: {e}")
                return
        
        # Kiểm tra và source drive_ws setup.sh để có CustomMsg
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = False
        
        if drive_ws_setup.exists():
            self.log("📦 Đang kiểm tra drive_ws/install/setup.sh...")
            if self._verify_source(drive_ws_setup, "drive_ws"):
                self.log("✅ drive_ws/install/setup.sh đã sẵn sàng")
                use_drive_ws = True
            else:
                self.log("⚠️  Không thể verify drive_ws setup, nhưng vẫn sẽ thử source")
                use_drive_ws = True
        else:
            self.log("⚠️  Cảnh báo: Không tìm thấy drive_ws/install/setup.sh")
            self.log("⚠️  Topic /livox/lidar (CustomMsg) có thể không record được")
            response = messagebox.askyesno(
                "Cảnh báo",
                "Không tìm thấy drive_ws/install/setup.sh.\n"
                "Topic /livox/lidar (CustomMsg) có thể không record được.\n\n"
                "Bạn có muốn tiếp tục không?"
            )
            if not response:
                return
        
        # Kiểm tra ws setup.sh
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                "Lỗi",
                f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}\n"
                "Vui lòng build workspace trước."
            )
            return
        
        self.log("📦 Đang kiểm tra ws/install/setup.sh...")
        if self._verify_source(ws_setup, "ws"):
            self.log("✅ ws/install/setup.sh đã sẵn sàng")
        else:
            self.log("⚠️  Không thể verify ws setup, nhưng vẫn sẽ thử source")
        
        # Lấy kích thước tối đa bag file
        try:
            max_bag_size = float(self.max_bag_size_var.get())
            if max_bag_size < 0:
                max_bag_size = 0
            # ROS2 yêu cầu tối thiểu 86,016 bytes (khoảng 0.00008 GB)
            # Nếu người dùng chọn giá trị quá nhỏ, sẽ dùng giá trị tối thiểu
            min_size_gb = 0.0001  # ~100KB, đủ lớn hơn 86KB
            if 0 < max_bag_size < min_size_gb:
                self.log(f"⚠️  Giá trị quá nhỏ, sử dụng tối thiểu {min_size_gb}GB")
                max_bag_size = min_size_gb
            self.max_bag_size_gb = max_bag_size
        except ValueError:
            self.log("⚠️  Giá trị kích thước bag không hợp lệ, sử dụng mặc định 2GB")
            self.max_bag_size_gb = 2
            max_bag_size = 2
        
        # Lấy danh sách topics được chọn
        selected_topics = [topic for topic, var in self.topic_vars.items() if var.get()]
        
        if not selected_topics:
            messagebox.showerror("Lỗi", "Vui lòng chọn ít nhất một topic để record")
            return
        
        # Tạo tên bag file với timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"recording_{timestamp}"
        bag_path = output_dir / bag_name
        
        # Build command với source cả drive_ws và ws setup.sh
        # Cần source drive_ws trước để có CustomMsg, sau đó source ws
        topics_str = " ".join(selected_topics)
        
        self.log(f"📡 Topics được chọn: {len(selected_topics)}/{len(self.topic_vars)}")
        for topic in selected_topics:
            self.log(f"   ✓ {topic}")
        
        # Build command với đầy đủ environment
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        
        # Build ros2 bag record command với option chia nhỏ bag
        bag_record_cmd = f"ros2 bag record -o {bag_path} {topics_str}"
        
        # Thêm option chia nhỏ bag nếu max_bag_size > 0
        if max_bag_size > 0:
            # Chuyển đổi GB sang bytes (1GB = 1024^3 bytes)
            max_bag_size_bytes = int(max_bag_size * 1024 * 1024 * 1024)
            # ROS2 yêu cầu tối thiểu 86,016 bytes
            min_size_bytes = 86016
            if max_bag_size_bytes < min_size_bytes:
                max_bag_size_bytes = min_size_bytes
                self.log(f"⚠️  Điều chỉnh kích thước tối thiểu: {min_size_bytes:,} bytes")
            bag_record_cmd += f" --max-bag-size {max_bag_size_bytes}"
            self.log(f"📦 Kích thước tối đa mỗi bag file: {max_bag_size}GB ({max_bag_size_bytes:,} bytes)")
            self.log("📦 Bag files sẽ tự động chia nhỏ khi đạt giới hạn")
        else:
            self.log("📦 Không giới hạn kích thước bag file (sẽ tạo một file duy nhất)")
        
        # Source theo thứ tự: ROS2 base -> drive_ws -> ws
        self.log("🔧 Đang chuẩn bị source environment...")
        if use_drive_ws:
            cmd = (
                f"source {ros2_setup} && "
                f"source {drive_ws_setup} && "
                f"source {ws_setup} && "
                f"{bag_record_cmd}"
            )
            self.log("✅ Sẽ source: ROS2 base -> drive_ws -> ws")
            self.log("✅ CustomMsg support đã được kích hoạt")
        else:
            cmd = (
                f"source {ros2_setup} && "
                f"source {ws_setup} && "
                f"{bag_record_cmd}"
            )
            self.log("⚠️  Sẽ source: ROS2 base -> ws (không có drive_ws)")
            self.log("⚠️  CustomMsg có thể không hoạt động")
        
        self.log(f"📝 Bắt đầu record bag: {bag_name}")
        self.log(f"📁 Output: {bag_path}")
        self.log(f"📡 Topics: {topics_str}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.record_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.is_recording = True
            self.output_dir = bag_path
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: Đang recording...",
                foreground="orange"
            )
            self.info_label.config(
                text=f"Đang record: {bag_name}\nThư mục: {bag_path}"
            )
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_record_process, daemon=True).start()
            
            self.log("✅ Recording đã được khởi động")
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu record: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            self.is_recording = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
    
    def stop_recording(self):
        """Dừng record rosbag"""
        if not self.is_recording:
            return
        
        if self.record_process:
            try:
                self.log("Đang dừng recording...")
                self.record_process.terminate()
                try:
                    self.record_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.record_process.kill()
                    self.record_process.wait()
            except Exception as e:
                self.log(f"Lỗi khi dừng record: {e}")
            
            self.record_process = None
        
        self.is_recording = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="green"
        )
        
        if self.output_dir:
            self.info_label.config(
                text=f"Đã dừng recording\nBag file: {self.output_dir}"
            )
            self.log(f"✅ Recording đã dừng. Bag file: {self.output_dir}")
        else:
            self.info_label.config(text="Đã dừng recording")
            self.log("✅ Recording đã dừng")
    
    def monitor_record_process(self):
        """Monitor record process output"""
        if not self.record_process:
            return
        
        try:
            for line in iter(self.record_process.stdout.readline, ''):
                if not line:
                    break
                line = line.strip()
                if line:
                    # Log output
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed']):
                        self.log(f"❌ ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️  WARNING: {line}")
                    else:
                        # Chỉ log các dòng quan trọng để tránh spam
                        if any(keyword in line.lower() for keyword in ['recording', 'bag', 'topic', 'message']):
                            self.log(line)
        except Exception as e:
            self.log(f"Lỗi khi đọc output: {e}")
        
        # Kiểm tra exit code
        if self.record_process.poll() is not None:
            exit_code = self.record_process.poll()
            if exit_code != 0:
                self.log(f"✗ Recording đã dừng với exit code: {exit_code}")
            else:
                self.log(f"✓ Recording đã dừng bình thường")
            
            self.is_recording = False
            self.after(0, partial(self._update_recording_stopped))
    
    def _update_recording_stopped(self):
        """Helper function để update UI sau khi recording dừng"""
        try:
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Đã dừng",
                foreground="green"
            )
            if self.output_dir:
                self.info_label.config(
                    text=f"Recording đã hoàn thành\nBag file: {self.output_dir}"
                )
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
