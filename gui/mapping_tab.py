#!/usr/bin/env python3
"""
Mapping Tab Module
Chứa MappingTab để start/stop FAST-LIVO2 mapping
"""

import threading
import subprocess
from pathlib import Path
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class TopicChecker(Node):
    """ROS2 Node để kiểm tra các topics có sẵn không"""
    
    def __init__(self):
        super().__init__('topic_checker')
        self.topics_status = {
            '/livox/lidar': False,
            '/livox/imu': False,
            '/image_perspective': False
        }
        self.topics_count = {
            '/livox/lidar': 0,
            '/livox/imu': 0,
            '/image_perspective': 0
        }
        
        # Tạo timer để check topics
        self.timer = self.create_timer(1.0, self.check_topics)
        self.check_topics()
    
    def check_topics(self):
        """Kiểm tra các topics có đang publish không"""
        try:
            topic_list = self.get_topic_names_and_types()
            topic_names = [name for name, _ in topic_list]
            
            for topic in self.topics_status.keys():
                if topic in topic_names:
                    self.topics_status[topic] = True
                else:
                    self.topics_status[topic] = False
        except Exception as e:
            self.get_logger().error(f'Lỗi kiểm tra topics: {e}')


class MappingTab(ttk.Frame):
    """Tab cho FAST-LIVO2 Mapping"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "drive_ws"
        
        # Processes
        self.mapping_process = None
        self.rviz_process = None
        
        # ROS
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_ros_running = False
        
        # State
        self.is_mapping_running = False
        self.is_rviz_running = False
        self.selected_config = "mid360_perspective"  # Default
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Mapping"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="FAST-LIVO2 Mapping",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Frame chọn config
        config_frame = ttk.LabelFrame(control_frame, text="Cấu hình", padding="5")
        config_frame.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.config_var = tk.StringVar(value="mid360_perspective")
        ttk.Radiobutton(
            config_frame,
            text="MID360 Perspective",
            variable=self.config_var,
            value="mid360_perspective",
            command=self.on_config_change
        ).pack(side=tk.LEFT, padx=5)
        
        ttk.Radiobutton(
            config_frame,
            text="Avia Perspective",
            variable=self.config_var,
            value="avia_perspective",
            command=self.on_config_change
        ).pack(side=tk.LEFT, padx=5)
        
        # Nút Check Topics
        self.check_topics_btn = ttk.Button(
            control_frame,
            text="Check Topics",
            command=self.check_topics
        )
        self.check_topics_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start Mapping
        self.start_mapping_btn = ttk.Button(
            control_frame,
            text="Start Mapping",
            command=self.start_mapping,
            state=tk.DISABLED
        )
        self.start_mapping_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Mapping
        self.stop_mapping_btn = ttk.Button(
            control_frame,
            text="Stop Mapping",
            command=self.stop_mapping,
            state=tk.DISABLED
        )
        self.stop_mapping_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start RViz
        self.start_rviz_btn = ttk.Button(
            control_frame,
            text="Start RViz",
            command=self.start_rviz,
            state=tk.DISABLED
        )
        self.start_rviz_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop RViz
        self.stop_rviz_btn = ttk.Button(
            control_frame,
            text="Stop RViz",
            command=self.stop_rviz,
            state=tk.DISABLED
        )
        self.stop_rviz_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Chưa kiểm tra topics",
            foreground="orange"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin topics
        topics_frame = ttk.LabelFrame(self, text="Trạng thái Topics", padding="10")
        topics_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Topic status labels
        self.lidar_status_label = ttk.Label(
            topics_frame,
            text="/livox/lidar: ❌ Chưa kiểm tra",
            font=("Arial", 10)
        )
        self.lidar_status_label.pack(anchor=tk.W, padx=5, pady=2)
        
        self.imu_status_label = ttk.Label(
            topics_frame,
            text="/livox/imu: ❌ Chưa kiểm tra",
            font=("Arial", 10)
        )
        self.imu_status_label.pack(anchor=tk.W, padx=5, pady=2)
        
        self.image_status_label = ttk.Label(
            topics_frame,
            text="/image_perspective: ❌ Chưa kiểm tra",
            font=("Arial", 10)
        )
        self.image_status_label.pack(anchor=tk.W, padx=5, pady=2)
        
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
    
    def on_config_change(self):
        """Xử lý khi thay đổi config"""
        self.selected_config = self.config_var.get()
        self.log(f"Đã chọn config: {self.selected_config}")
    
    def check_topics(self):
        """Kiểm tra các topics có sẵn không"""
        if not self.is_ros_running:
            self.start_ros_checker()
        
        # Đợi một chút để ROS node có thời gian check
        self.after(2000, self.update_topics_status)
    
    def start_ros_checker(self):
        """Start ROS node để check topics"""
        if self.is_ros_running:
            return
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.ros_node = TopicChecker()
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            self.is_ros_running = True
            
            self.log("Đã khởi động ROS topic checker")
        except Exception as e:
            self.log(f"Lỗi khởi động ROS checker: {e}")
            messagebox.showerror("Lỗi", f"Không thể khởi động ROS checker: {e}")
    
    def ros_spin(self):
        """ROS spin trong thread riêng"""
        try:
            while rclpy.ok() and self.is_ros_running:
                self.ros_executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.log(f"Lỗi trong ROS spin: {e}")
    
    def update_topics_status(self):
        """Cập nhật trạng thái topics"""
        if not self.ros_node:
            return
        
        # Update labels
        lidar_ok = self.ros_node.topics_status.get('/livox/lidar', False)
        imu_ok = self.ros_node.topics_status.get('/livox/imu', False)
        image_ok = self.ros_node.topics_status.get('/image_perspective', False)
        
        self.lidar_status_label.config(
            text=f"/livox/lidar: {'✅ Đang publish' if lidar_ok else '❌ Không có'}",
            foreground="green" if lidar_ok else "red"
        )
        
        self.imu_status_label.config(
            text=f"/livox/imu: {'✅ Đang publish' if imu_ok else '❌ Không có'}",
            foreground="green" if imu_ok else "red"
        )
        
        self.image_status_label.config(
            text=f"/image_perspective: {'✅ Đang publish' if image_ok else '❌ Không có'}",
            foreground="green" if image_ok else "red"
        )
        
        # Update status và buttons
        all_ok = lidar_ok and imu_ok and image_ok
        if all_ok:
            self.status_label.config(
                text="Trạng thái: ✅ Tất cả topics sẵn sàng",
                foreground="green"
            )
            self.start_mapping_btn.config(state=tk.NORMAL)
            self.start_rviz_btn.config(state=tk.NORMAL)
            self.log("✅ Tất cả topics đã sẵn sàng!")
        else:
            self.status_label.config(
                text="Trạng thái: ⚠️ Thiếu topics",
                foreground="orange"
            )
            self.start_mapping_btn.config(state=tk.DISABLED)
            self.start_rviz_btn.config(state=tk.DISABLED)
            missing = []
            if not lidar_ok:
                missing.append("/livox/lidar")
            if not imu_ok:
                missing.append("/livox/imu")
            if not image_ok:
                missing.append("/image_perspective")
            self.log(f"⚠️ Thiếu topics: {', '.join(missing)}")
    
    def start_mapping(self):
        """Start FAST-LIVO2 mapping"""
        if self.is_mapping_running:
            self.log("Mapping đã đang chạy")
            return
        
        try:
            setup_script = self.workspace_path / "install" / "setup.sh"
            if not setup_script.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy setup.sh tại: {setup_script}\n"
                    "Vui lòng build workspace trước."
                )
                return
            
            # Chọn launch file dựa trên config
            if self.selected_config == "mid360_perspective":
                launch_file_name = "mapping_mid360_perspective.launch.py"
            else:
                launch_file_name = "mapping_avia_perspective.launch.py"
            
            # Source setup và chạy launch
            cmd = [
                "bash", "-c",
                f"source {setup_script} && "
                f"ros2 launch fast_livo {launch_file_name}"
            ]
            
            self.log(f"Đang khởi động mapping với config: {self.selected_config}")
            self.log(f"Launch file: {launch_file_name}")
            
            self.mapping_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(self.workspace_path),
                bufsize=1,
                universal_newlines=True
            )
            
            self.is_mapping_running = True
            self.start_mapping_btn.config(state=tk.DISABLED)
            self.stop_mapping_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: 🚀 Mapping đang chạy",
                foreground="green"
            )
            
            # Start thread để đọc output
            threading.Thread(
                target=self._read_mapping_output,
                daemon=True
            ).start()
            
            self.log("✅ Mapping đã được khởi động")
            
        except Exception as e:
            self.log(f"❌ Lỗi khởi động mapping: {e}")
            messagebox.showerror("Lỗi", f"Không thể khởi động mapping: {e}")
            self.is_mapping_running = False
            self.start_mapping_btn.config(state=tk.NORMAL)
            self.stop_mapping_btn.config(state=tk.DISABLED)
    
    def _read_mapping_output(self):
        """Đọc output từ mapping process"""
        try:
            for line in iter(self.mapping_process.stdout.readline, ''):
                if line:
                    self.after(0, lambda l=line: self.log(l.strip()))
        except Exception as e:
            self.log(f"Lỗi đọc output: {e}")
    
    def stop_mapping(self):
        """Stop FAST-LIVO2 mapping"""
        if not self.is_mapping_running:
            return
        
        try:
            if self.mapping_process:
                self.mapping_process.terminate()
                try:
                    self.mapping_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.mapping_process.kill()
                    self.mapping_process.wait()
                self.mapping_process = None
            
            self.is_mapping_running = False
            self.start_mapping_btn.config(state=tk.NORMAL)
            self.stop_mapping_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Mapping đã dừng",
                foreground="orange"
            )
            self.log("✅ Mapping đã được dừng")
            
        except Exception as e:
            self.log(f"❌ Lỗi dừng mapping: {e}")
            messagebox.showerror("Lỗi", f"Không thể dừng mapping: {e}")
    
    def start_rviz(self):
        """Start RViz"""
        if self.is_rviz_running:
            self.log("RViz đã đang chạy")
            return
        
        try:
            setup_script = self.workspace_path / "install" / "setup.sh"
            
            cmd = [
                "bash", "-c",
                f"source {setup_script} && rviz2"
            ]
            
            self.log("Đang khởi động RViz...")
            
            self.rviz_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=str(self.workspace_path)
            )
            
            self.is_rviz_running = True
            self.start_rviz_btn.config(state=tk.DISABLED)
            self.stop_rviz_btn.config(state=tk.NORMAL)
            self.log("✅ RViz đã được khởi động")
            
        except Exception as e:
            self.log(f"❌ Lỗi khởi động RViz: {e}")
            messagebox.showerror("Lỗi", f"Không thể khởi động RViz: {e}")
            self.is_rviz_running = False
            self.start_rviz_btn.config(state=tk.NORMAL)
            self.stop_rviz_btn.config(state=tk.DISABLED)
    
    def stop_rviz(self):
        """Stop RViz"""
        if not self.is_rviz_running:
            return
        
        try:
            if self.rviz_process:
                self.rviz_process.terminate()
                try:
                    self.rviz_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.rviz_process.kill()
                    self.rviz_process.wait()
                self.rviz_process = None
            
            self.is_rviz_running = False
            self.start_rviz_btn.config(state=tk.NORMAL)
            self.stop_rviz_btn.config(state=tk.DISABLED)
            self.log("✅ RViz đã được dừng")
            
        except Exception as e:
            self.log(f"❌ Lỗi dừng RViz: {e}")
            messagebox.showerror("Lỗi", f"Không thể dừng RViz: {e}")

