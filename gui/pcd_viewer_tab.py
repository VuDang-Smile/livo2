#!/usr/bin/env python3
"""
PCD Viewer Tab Module
Chứa PCDViewerTab để hiển thị PCD files trong RViz
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


class PCDViewerTab(ttk.Frame):
    """Tab cho PCD Viewer"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.fast_livo_path = self.workspace_path / "src" / "FAST-LIVO2"
        self.default_pcd_dir = self.fast_livo_path / "Log" / "PCD"
        
        # Processes
        self.pcd_viewer_process = None
        self.rviz_process = None
        
        # State
        self.is_viewer_running = False
        self.is_rviz_running = False
        self.pcd_file_path = None
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab PCD Viewer"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="PCD Map Viewer",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Frame chọn PCD file
        pcd_frame = ttk.LabelFrame(control_frame, text="Chọn PCD File", padding="10")
        pcd_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.pcd_path_var = tk.StringVar()
        pcd_entry = ttk.Entry(pcd_frame, textvariable=self.pcd_path_var, width=60)
        pcd_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            pcd_frame,
            text="Browse",
            command=self.browse_pcd_file
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        auto_find_btn = ttk.Button(
            pcd_frame,
            text="Auto Find",
            command=self.auto_find_pcd
        )
        auto_find_btn.pack(side=tk.LEFT, padx=5)
        
        # Frame cấu hình
        config_frame = ttk.LabelFrame(control_frame, text="Cấu hình", padding="10")
        config_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Topic name
        topic_frame = ttk.Frame(config_frame)
        topic_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            topic_frame,
            text="Topic Name:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.topic_var = tk.StringVar(value="/pcd_map")
        topic_entry = ttk.Entry(
            topic_frame,
            textvariable=self.topic_var,
            width=20
        )
        topic_entry.pack(side=tk.LEFT, padx=5)
        
        # Frame ID
        frame_id_frame = ttk.Frame(config_frame)
        frame_id_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            frame_id_frame,
            text="Frame ID:",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.frame_id_var = tk.StringVar(value="map")
        frame_id_entry = ttk.Entry(
            frame_id_frame,
            textvariable=self.frame_id_var,
            width=20
        )
        frame_id_entry.pack(side=tk.LEFT, padx=5)
        
        # Publish rate
        rate_frame = ttk.Frame(config_frame)
        rate_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(
            rate_frame,
            text="Publish Rate (Hz):",
            font=("Arial", 10)
        ).pack(side=tk.LEFT, padx=5)
        
        self.rate_var = tk.StringVar(value="1.0")
        rate_spinbox = ttk.Spinbox(
            rate_frame,
            from_=0.0,
            to=10.0,
            increment=0.1,
            textvariable=self.rate_var,
            width=10,
            format="%.1f"
        )
        rate_spinbox.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(
            rate_frame,
            text="(0.0 = publish once)",
            font=("Arial", 9),
            foreground="gray"
        ).pack(side=tk.LEFT, padx=10)
        
        # Loop option
        self.loop_var = tk.BooleanVar(value=True)
        loop_checkbox = ttk.Checkbutton(
            config_frame,
            text="Loop (publish liên tục)",
            variable=self.loop_var
        )
        loop_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Use RViz option
        self.use_rviz_var = tk.BooleanVar(value=True)
        rviz_checkbox = ttk.Checkbutton(
            config_frame,
            text="Launch RViz",
            variable=self.use_rviz_var
        )
        rviz_checkbox.pack(anchor=tk.W, padx=5, pady=2)
        
        # Frame nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.start_btn = ttk.Button(
            button_frame,
            text="Start PCD Viewer",
            command=self.start_viewer,
            style="Accent.TButton"
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            button_frame,
            text="Stop PCD Viewer",
            command=self.stop_viewer,
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
        
        # Frame thông tin PCD
        info_frame = ttk.LabelFrame(self, text="Thông tin PCD", padding="10")
        info_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.info_label = ttk.Label(
            info_frame,
            text="Chưa chọn PCD file",
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
        
        # Tự động tìm PCD file khi khởi động
        self.auto_find_pcd()
    
    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def browse_pcd_file(self):
        """Browse cho PCD file"""
        file = filedialog.askopenfilename(
            title="Chọn PCD file",
            initialdir=self.pcd_path_var.get() or str(self.default_pcd_dir),
            filetypes=[("PCD files", "*.pcd"), ("All files", "*.*")]
        )
        if file:
            self.pcd_path_var.set(file)
            self._update_pcd_info(file)
            self.log(f"Đã chọn PCD file: {file}")
    
    def auto_find_pcd(self):
        """Tự động tìm PCD file trong Log/PCD directory"""
        if not self.default_pcd_dir.exists():
            self.log(f"⚠️  Thư mục PCD không tồn tại: {self.default_pcd_dir}")
            return
        
        # Ưu tiên tìm all_raw_points.pcd, sau đó all_downsampled_points.pcd
        raw_file = self.default_pcd_dir / "all_raw_points.pcd"
        downsampled_file = self.default_pcd_dir / "all_downsampled_points.pcd"
        
        if raw_file.exists():
            self.pcd_path_var.set(str(raw_file))
            self._update_pcd_info(str(raw_file))
            self.log(f"✓ Tự động tìm thấy: {raw_file.name}")
        elif downsampled_file.exists():
            self.pcd_path_var.set(str(downsampled_file))
            self._update_pcd_info(str(downsampled_file))
            self.log(f"✓ Tự động tìm thấy: {downsampled_file.name}")
        else:
            self.log(f"⚠️  Không tìm thấy PCD file trong {self.default_pcd_dir}")
            self.info_label.config(text="Không tìm thấy PCD file")
    
    def _update_pcd_info(self, pcd_path):
        """Cập nhật thông tin về PCD file"""
        try:
            pcd_path_obj = Path(pcd_path)
            if not pcd_path_obj.exists():
                self.info_label.config(text="PCD file không tồn tại")
                return
            
            # Lấy thông tin file
            file_size = pcd_path_obj.stat().st_size
            file_size_mb = file_size / (1024 * 1024)
            
            self.info_label.config(
                text=f"File: {pcd_path_obj.name}\n"
                     f"Kích thước: {file_size_mb:.2f} MB\n"
                     f"Đường dẫn: {pcd_path}"
            )
        except Exception as e:
            self.info_label.config(text=f"Lỗi: {e}")
    
    def start_viewer(self):
        """Bắt đầu PCD viewer"""
        if self.is_viewer_running:
            messagebox.showwarning("Cảnh báo", "PCD viewer đang chạy, vui lòng dừng trước")
            return
        
        # Kiểm tra PCD path
        pcd_path = self.pcd_path_var.get()
        if not pcd_path:
            messagebox.showerror("Lỗi", "Vui lòng chọn PCD file hoặc dùng Auto Find")
            return
        
        pcd_path_obj = Path(pcd_path)
        if not pcd_path_obj.exists():
            messagebox.showerror("Lỗi", f"PCD file không tồn tại: {pcd_path}")
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
        
        # Lấy các parameters
        topic_name = self.topic_var.get().strip() or "/pcd_map"
        frame_id = self.frame_id_var.get().strip() or "map"
        
        try:
            publish_rate = float(self.rate_var.get())
            if publish_rate < 0:
                publish_rate = 1.0
        except ValueError:
            publish_rate = 1.0
            self.log("⚠️  Giá trị rate không hợp lệ, sử dụng mặc định 1.0")
        
        loop = self.loop_var.get()
        use_rviz = self.use_rviz_var.get()
        
        # Build command
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        
        # Build launch command
        launch_cmd = (
            f"ros2 launch fast_livo pcd_viewer.launch.py "
            f"pcd_file:={pcd_path} "
            f"topic_name:={topic_name} "
            f"frame_id:={frame_id} "
            f"publish_rate:={publish_rate} "
            f"loop:={str(loop).lower()} "
            f"use_rviz:={str(use_rviz).lower()}"
        )
        
        # Source environment
        cmd = (
            f"source {ros2_setup} && "
            f"source {ws_setup} && "
            f"{launch_cmd}"
        )
        
        self.log(f"📝 Bắt đầu PCD viewer")
        self.log(f"📁 PCD file: {pcd_path_obj.name}")
        self.log(f"📁 Đường dẫn: {pcd_path}")
        self.log(f"⚙️  Topic: {topic_name}")
        self.log(f"⚙️  Frame ID: {frame_id}")
        self.log(f"⚙️  Publish Rate: {publish_rate} Hz")
        self.log(f"⚙️  Loop: {'Có' if loop else 'Không'}")
        self.log(f"⚙️  RViz: {'Có' if use_rviz else 'Không'}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.pcd_viewer_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.is_viewer_running = True
            self.pcd_file_path = pcd_path
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: Đang chạy...",
                foreground="orange"
            )
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_viewer_process, daemon=True).start()
            
            self.log("✅ PCD viewer đã được khởi động")
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu PCD viewer: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            self.is_viewer_running = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
    
    def stop_viewer(self):
        """Dừng PCD viewer"""
        if not self.is_viewer_running:
            return
        
        if self.pcd_viewer_process:
            try:
                self.log("Đang dừng PCD viewer...")
                # Terminate process
                self.pcd_viewer_process.terminate()
                try:
                    self.pcd_viewer_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.pcd_viewer_process.kill()
                    self.pcd_viewer_process.wait()
                
                # Kill các process con (rviz, pcd_viewer_node) nếu còn chạy
                try:
                    # Tìm và kill rviz2 process
                    subprocess.run(
                        ["pkill", "-f", "rviz2.*pcd_viewer"],
                        timeout=2,
                        capture_output=True
                    )
                    # Tìm và kill pcd_viewer_node
                    subprocess.run(
                        ["pkill", "-f", "pcd_viewer_node"],
                        timeout=2,
                        capture_output=True
                    )
                except Exception:
                    pass  # Ignore errors khi kill processes
                    
            except Exception as e:
                self.log(f"Lỗi khi dừng PCD viewer: {e}")
            
            self.pcd_viewer_process = None
        
        self.is_viewer_running = False
        self.is_rviz_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="green"
        )
        
        if self.pcd_file_path:
            pcd_name = Path(self.pcd_file_path).name
            self.info_label.config(
                text=f"Đã dừng viewer\nFile: {pcd_name}"
            )
            self.log(f"✅ PCD viewer đã dừng. File: {pcd_name}")
        else:
            self.info_label.config(text="Đã dừng viewer")
            self.log("✅ PCD viewer đã dừng")
    
    def monitor_viewer_process(self):
        """Monitor viewer process output"""
        if not self.pcd_viewer_process:
            return
        
        try:
            for line in iter(self.pcd_viewer_process.stdout.readline, ''):
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
                        # Log các dòng quan trọng
                        if any(keyword in line.lower() for keyword in ['loaded', 'publishing', 'started', 'rviz']):
                            self.log(line)
        except Exception as e:
            self.log(f"Lỗi khi đọc output: {e}")
        
        # Kiểm tra exit code
        if self.pcd_viewer_process.poll() is not None:
            exit_code = self.pcd_viewer_process.poll()
            if exit_code != 0:
                self.log(f"✗ PCD viewer đã dừng với exit code: {exit_code}")
            else:
                self.log(f"✓ PCD viewer đã hoàn thành")
            
            self.is_viewer_running = False
            self.after(0, partial(self._update_viewer_stopped))
    
    def _update_viewer_stopped(self):
        """Helper function để update UI sau khi viewer dừng"""
        try:
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Đã dừng",
                foreground="green"
            )
            if self.pcd_file_path:
                pcd_name = Path(self.pcd_file_path).name
                self.info_label.config(
                    text=f"Viewer đã dừng\nFile: {pcd_name}"
                )
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")

