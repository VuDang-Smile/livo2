import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
import time
from pathlib import Path
import sys
import subprocess
import os
import threading
import signal
import zipfile
sys.path.insert(0, str(Path(__file__).parent.parent / "languages"))
from translate_engine import Translator

# Try to import requests for backend upload
try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False

class BagMappingInterface:
    def __init__(self, root):
        self.root = root
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
        self.bag_path = None
        self.config_path = None
        
        self.mapping_process = None
        self.is_mapping_running = False
        # Thêm process cho ros2 bag play
        self.bag_process = None
        self.is_bag_playing = False
        self.use_rviz = False
        # Bag rate mặc định, dùng cho ros2 bag play
        self.bag_rate = 0.5
        # Backend URL for map upload
        self.backend_base_url = os.environ.get("LIVO_BACKEND_URL", "http://backend.lidar.tm")
        
        self.translator = Translator('en')
        self.current_lang = 'en'
        
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
        
        # Separator để tách biệt progress và log
        separator2 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator2.pack(fill=tk.X, pady=(0, 10))
        
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
                                 font=("Consolas", 10), state=tk.DISABLED, bd=1, relief=tk.SUNKEN)
        self.log_panel.pack(fill=tk.BOTH, expand=True)



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
            
            time.sleep(3)
            
            if self.mapping_process.poll() is not None:
                error_output = self.mapping_process.stdout.read() if self.mapping_process.stdout else self.translator.get('log.no_output', 'No output')
                self.add_log(f"❌ {self.translator.get('log.mapping_process_exited', 'Mapping process exited immediately with code')}: {self.mapping_process.returncode}")
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
        
        self.btn_upload.config(state=tk.DISABLED)
        self.add_log(self.translator.get('log.pipeline_start', '🚀 Starting map processing and upload pipeline...'))
        threading.Thread(target=self._run_upload_pipeline_thread, daemon=True).start()
    
    def _run_upload_pipeline_thread(self):
        """Thread thực hiện full pipeline và upload"""
        start_time = time.time()
        
        try:
            # 1. Merge PCD
            self.add_log(self.translator.get('log.step_merge_pcd', 'Step 1/6: Merging PCD files...'))
            if not self.merge_pcd_files(silent=True):
                self.add_log(self.translator.get('log.pipeline_stopped_merge', '❌ Pipeline stopped at Merge PCD step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.progress.config(value=20))
            
            # 2. HBA Optimize
            self.add_log(self.translator.get('log.step_hba', 'Step 2/6: Running HBA Optimize...'))
            if not self._run_hba_core():
                self.add_log(self.translator.get('log.pipeline_stopped_hba', '❌ Pipeline stopped at HBA step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.progress.config(value=40))
            
            # 3. SC Tile
            self.add_log(self.translator.get('log.step_sc', 'Step 3/6: Running ScanContext Tiling...'))
            if not self._run_sc_core():
                self.add_log(self.translator.get('log.pipeline_stopped_sc', '❌ Pipeline stopped at ScanContext step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.progress.config(value=60))
            
            # 4. Generate Floorplan
            self.add_log(self.translator.get('log.step_floorplan', 'Step 4/6: Generating floorplan...'))
            if not self._generate_floorplan():
                self.add_log(self.translator.get('log.pipeline_stopped_floorplan', '❌ Pipeline stopped at Generate Floorplan step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.progress.config(value=80))
            
            # 5. Zip files
            self.add_log(self.translator.get('log.step_zip', 'Step 5/6: Compressing files (Zip)...'))
            zip_path = self._zip_map_folders()
            if not zip_path:
                self.add_log(self.translator.get('log.pipeline_stopped_zip', '❌ Pipeline stopped at Zip step.'))
                self.root.after(0, lambda: self.btn_upload.config(state=tk.NORMAL))
                return
            
            self.root.after(0, lambda: self.progress.config(value=90))
            
            # 6. Upload to backend
            self.add_log(self.translator.get('log.step_upload', 'Step 6/6: Uploading to backend...'))
            upload_success, upload_info = self._upload_map_to_backend(zip_path)
            if upload_success:
                if upload_info:
                    upload_msg = self.translator.get('log.upload_backend_success', '✅ Backend upload successful (upload_id: {upload_id})').replace('{upload_id}', str(upload_info))
                else:
                    upload_msg = "✅ Backend upload successful"
                self.add_log(upload_msg)
            else:
                fail_reason = upload_info or "Unknown error"
                self.add_log(self.translator.get('log.upload_backend_failed', '⚠️ Backend upload failed: {reason}').replace('{reason}', str(fail_reason)))
            
            self.root.after(0, lambda: self.progress.config(value=100))
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
            self.progress['value'] = val
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

    def launch_rviz_cmd(self):
        self.add_log(self.translator.get('log.system_launching_rviz2', 'SYSTEM: Launching RViz2...'))

    def clear_log(self):
        self.log_panel.config(state=tk.NORMAL)
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state=tk.DISABLED)

    def add_log(self, message):
        self.log_panel.config(state=tk.NORMAL)
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_panel.insert(tk.END, f"[{time_str}] {message}\n")
        self.log_panel.see(tk.END)
        self.log_panel.config(state=tk.DISABLED)
    
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

    def monitor_mapping_process(self):
        if not self.mapping_process:
            return
        
        try:
            for line in iter(self.mapping_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_mapping_running:
                    line_lower = line.lower()
                    if any(keyword in line_lower for keyword in ['error', 'warning', 'started', 'ready', 'failed']):
                        self.add_log(f"[Mapping] {line.strip()}")
                else:
                    break
        except Exception as e:
            if self.is_mapping_running:
                self.add_log(f"⚠️ {self.translator.get('log.error_reading_mapping_output', 'Error reading mapping output')}: {e}")
    
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
        except Exception as e:
            if self.is_bag_playing:
                self.add_log(self.translator.get('log.error_reading_bag', '⚠️ Error reading bag output: {error}').replace('{error}', str(e)))
    
    def cleanup_processes(self):
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
        """Core logic của ScanContext tiling (đồng bộ) - sử dụng input từ hba_map"""
        hba_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "hba_map"
        merged_map_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "merged_map"
        sc_script = Path(__file__).parent.parent / "scripts" / "generate_fast_localization_map.py"
        output_dir = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "fastloc_map"

        # Ưu tiên input từ hba_map, fallback về merged_map
        input_pcd = hba_map_dir / "merge_all_hba.pcd"
        if not input_pcd.exists():
            self.add_log(self.translator.get('log.hba_not_found_try_merged', '⚠️ HBA map not found, trying merged_all.pcd...'))
            input_pcd = merged_map_dir / "merged_all.pcd"
            if not input_pcd.exists():
                self.add_log(self.translator.get('log.no_merged_pcd_found', '❌ No merged PCD files found'))
                return False

        self.add_log("=" * 60)
        self.add_log(self.translator.get('log.sc_start', '🗺️ Preparing ScanContext map from {filename}...').replace('{filename}', input_pcd.name))
        self.add_log(f"📁 Input: {input_pcd.parent}")
        self.add_log(f"📁 Output: {output_dir}")

        try:
            cmd = (
                f"python3 {sc_script} "
                f"--input_pcd {input_pcd} "
                f"--output_dir {output_dir} "
                f"--strip_color --voxel_size 0.2 --tile_size 50.0"
            )
            process = subprocess.Popen(
                cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
            
            for line in iter(process.stdout.readline, ''):
                if line:
                    self.add_log(f"[ScanContext] {line.strip()}")
            
            process.wait()
            
            if process.returncode == 0:
                self.add_log(self.translator.get('log.sc_success', '✅ ScanContext Tiling successful!'))
                self.add_log(f"📁 Output: {output_dir}")
                return True
            else:
                self.add_log(self.translator.get('log.sc_failed', '❌ Tiling failed with code {code}').replace('{code}', str(process.returncode)))
                return False
        except Exception as e:
            self.add_log(self.translator.get('log.sc_error', '❌ ScanContext error: {error}').replace('{error}', str(e)))
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
    
    def _zip_map_folders(self):
        """Zip các folder map: merged_map, hba_map, fastloc_map, floorplan_2d"""
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
        
        if not existing_folders:
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
        self.add_log(self.translator.get('log.zip_folders', '📁 Folders to zip: {folders}').replace('{folders}', ', '.join(existing_folders)))
        
        try:
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
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
    
    def _upload_map_to_backend(self, zip_path: Path):
        """Upload file zip map lên backend."""
        if not REQUESTS_AVAILABLE:
            msg = self.translator.get('log.upload_backend_requests_missing', 'Missing requests library. Install with: pip install requests')
            self.add_log(f"❌ {msg}")
            return False, msg
        
        zip_path = Path(zip_path)
        if not zip_path.exists():
            error_msg = self.translator.get('log.upload_backend_file_not_found', 'File not found: {path}').replace('{path}', str(zip_path))
            return False, error_msg
        
        upload_url = f"{self.backend_base_url.rstrip('/')}/api/v1/maps/upload"
        self.add_log(self.translator.get('log.upload_endpoint', '🌐 Endpoint: {url}').replace('{url}', upload_url))
        
        try:
            with open(zip_path, "rb") as f:
                files = {"file": (zip_path.name, f, "application/zip")}
                response = requests.post(upload_url, files=files, timeout=120)
            
            if response.status_code in (200, 201):
                upload_id = None
                try:
                    data = response.json()
                    upload_id = data.get("upload_id") or data.get("uploadId")
                except Exception:
                    upload_id = None
                return True, upload_id
            
            # Non-200 response
            error_text = response.text.strip()[:200] if response.text else f"status {response.status_code}"
            return False, error_text
        
        except requests.exceptions.RequestException as e:
            return False, str(e)
    
    def stop_process(self):
        self.cleanup_processes()
        self.btn_start.config(state=tk.NORMAL)
        self.progress['value'] = 0
        self.status_label.config(text=self.translator.get('label.status_stopped', 'Status: Stopped'))
        self.add_log(self.translator.get('log.process_terminated', 'PROCESS: Mapping and Upload terminated.'))

if __name__ == "__main__":
    root = tk.Tk()
    app = BagMappingInterface(root)
    root.mainloop()