import tkinter as tk
from tkinter import ttk
import threading
import time
from datetime import datetime
from pathlib import Path
import subprocess
import os
from functools import partial
from ament_index_python import get_package_share_directory
from languages.translate_engine import translator
from recorder_logic import Recorder # Import file mới
from theta_logic import ThetaDriver # Import file mới
from livox_logic import LivoxTab # Import file mới
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)

class LivoxApp:
    def __init__(self, root):
        self.root = root
        self.root.title(translator.get("title.livox_panel"))
        self.root.geometry("1000x600")
        
        self.is_recording = False
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "drive_ws"
        self.record_process = None
        self.output_dir = None
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
        
        
        self.is_recording = False
        self._setup_layout()
        self.recorder = Recorder(log_callback=self.log)
        self.theta_driver = ThetaDriver(log_callback=self.log, update_ui_theta_connected = self.update_ui_theta_connected, canvas = self.canvas)
        self.livox_tab = LivoxTab(log_callback=self.log, update_label_livo_connected=self.update_label_livo_connected)
        self.mid360_ip_var = tk.StringVar(value="192.168.1.109")

    def update_label_livo_connected(self, is_running):
        if is_running:
            self.label_livox.config(
                text="● " + translator.get("label.livox_driver") + ": " + translator.get("status.running"),
                fg="green"
            )
        else:
            self.label_livox.config(
                text="● " + translator.get("label.livox_driver") + ": " + translator.get("status.not_running"),
                fg="red"
            )

    
    def toggle_language(self):
        # Logic đổi qua lại giữa 'en' và 'jp'
        new_lang = 'en' if translator.lang_code == 'jp' else 'jp'
        translator.switch_language(new_lang)
        
        # Cập nhật lại tiêu đề cửa sổ
        self.root.title(translator.get("title.livox_panel"))
        
        # Cập nhật UI texts
        self.update_ui_texts()

    def update_ui_texts(self):
        """Cập nhật lại các text trên UI khi đổi ngôn ngữ"""
        # Cập nhật tiêu đề cửa sổ
        self.root.title(translator.get("title.livox_panel"))
        
        # Cập nhật nút chuyển đổi ngôn ngữ
        current_lang_text = "日本語" if translator.lang_code == "en" else "English"
        if hasattr(self, 'btn_switch_lang'):
            self.btn_switch_lang.config(text="🌐 " + current_lang_text)
        
        # Cập nhật sidebar
        if hasattr(self, 'sidebar'):
            self.sidebar.config(text=translator.get("label.livox_driver_2"))
        
        # Cập nhật các nút
        if hasattr(self, 'btn_start_livox'):
            self.btn_start_livox.config(text=translator.get("button.start"))
        if hasattr(self, 'btn_stop_livox'):
            self.btn_stop_livox.config(text=translator.get("button.stop"))
        
        # Cập nhật các label
        if hasattr(self, 'label_theta_usb'):
            if hasattr(self, 'is_camera_connected') and self.is_camera_connected:
                self.label_theta_usb.config(text="● " + translator.get("label.camera") + ": " + translator.get("status.connected"))
            else:
                self.label_theta_usb.config(text="● " + translator.get("label.camera") + ": " + translator.get("status.disconnected"))
        
        if hasattr(self, 'label_slam_usb'):
            if hasattr(self, 'is_mid360_connected') and self.is_mid360_connected:
                self.label_slam_usb.config(text="● " + translator.get("label.livox_360") + ": " + translator.get("status.connected"))
            else:
                self.label_slam_usb.config(text="● " + translator.get("label.livox_360") + ": " + translator.get("status.disconnected"))
        
        if hasattr(self, 'label_theta'):
            if hasattr(self, 'is_active_theta') and self.is_active_theta:
                self.label_theta.config(text="● " + translator.get("label.theta_driver") + ": " + translator.get("status.running"))
            else:
                self.label_theta.config(text="● " + translator.get("label.theta_driver") + ": " + translator.get("status.not_running"))
        
        if hasattr(self, 'label_livox'):
            if hasattr(self, 'livox_tab') and hasattr(self.livox_tab, 'is_running') and self.livox_tab.is_running:
                self.label_livox.config(text="● " + translator.get("label.livox_driver") + ": " + translator.get("status.running"))
            else:
                self.label_livox.config(text="● " + translator.get("label.livox_driver") + ": " + translator.get("status.not_running"))
        
        # Cập nhật recording configuration
        if hasattr(self, 'conf_frame'):
            self.conf_frame.config(text=translator.get("label.recording_configuration"))
        
        # Cập nhật các nút và label khác
        if hasattr(self, 'btn_record'):
            if hasattr(self, 'recorder') and self.recorder.is_recording:
                self.btn_record.config(text="■ " + translator.get("button.stop_recording"))
            else:
                self.btn_record.config(text="● " + translator.get("button.start_recording"))
        
        if hasattr(self, 'lbl_status'):
            if hasattr(self, 'recorder') and self.recorder.is_recording:
                self.lbl_status.config(text=translator.get("label.status_recording"))
            else:
                self.lbl_status.config(text=translator.get("label.status") + ": " + translator.get("status.ready"))
        
        if hasattr(self, 'btn_rviz'):
            self.btn_rviz.config(text="📊 " + translator.get("button.launch_rviz2"))
        
        if hasattr(self, 'btn_clear_log'):
            self.btn_clear_log.config(text=translator.get("button.clear_log"))

    def _setup_layout(self):

        # --- 0. Top Bar ---
        self.top_menu = tk.Frame(self.root, pady=5)
        self.top_menu.pack(side="top", fill="both")
        
        # Xác định tên nút dựa trên ngôn ngữ hiện tại
        current_lang_text = "日本語" if translator.lang_code == "en" else "English"
        
        self.btn_switch_lang = tk.Button(
            self.top_menu, 
            text="🌐 " + current_lang_text, 
            command=self.toggle_language,
            font=("Arial", 9, "bold"),
            # bg="#ffffff",
            width=12
        )
        self.btn_switch_lang.pack(side="right", padx=10)

        # 1. Main Container (Sidebar + Workspace)
        self.main_container = tk.Frame(self.root)
        self.main_container.pack(side="top", fill="both", expand=True)

        # 2. Sidebar (Trái)
        self.sidebar = tk.LabelFrame(self.main_container, text=translator.get("label.livox_driver_2"), padx=10, pady=10)
        self.sidebar.pack(side="left", fill="y", padx=5, pady=5)

        # tk.Checkbutton(self.sidebar, text=translator.get("label.enable_converter")).pack(anchor="w")

        btn_driver_frame = tk.Frame(self.sidebar)
        btn_driver_frame.pack(fill="x", pady=10)
        self.btn_start_livox = tk.Button(btn_driver_frame, text=translator.get("button.start"), width=10, command=self.start_livox_driver)
        self.btn_start_livox.pack(side="left", padx=2)
        self.btn_stop_livox = tk.Button(btn_driver_frame, text=translator.get("button.stop"), width=10, command=self.stop_livox_driver)
        self.btn_stop_livox.pack(side="left", padx=2)
        self.btn_stop_livox.config(state=tk.DISABLED)

        # Trong hàm khởi tạo (ví dụ __init__)
        self.label_theta_usb = tk.Label(
            self.sidebar, 
            text="● " + translator.get("label.camera") + ": " + translator.get("status.disconnected")
        )
        self.label_theta_usb.pack(anchor="w", pady=5)

        self.label_slam_usb = tk.Label(
            self.sidebar, 
            text="● " + translator.get("label.livox_360") + ": " + translator.get("status.disconnected")
        )
        self.label_slam_usb.pack(anchor="w", pady=5)
        
        self.label_theta = tk.Label(
            self.sidebar, 
            text="● " + translator.get("label.theta_driver") + ": " + translator.get("status.not_running")
        )
        self.label_theta.pack(anchor="w", pady=5)

        self.label_livox = tk.Label(
            self.sidebar, 
            text="● " + translator.get("label.livox_driver") + ": " + translator.get("status.not_running")
        )
        self.label_livox.pack(anchor="w")

        # --- THÊM NÚT RVIZ TẠI ĐÂY ---
        # Một đường kẻ ngang nhẹ để phân cách (tùy chọn)

        # 3. Workspace (Phải)
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side="right", fill="both", expand=True, padx=5, pady=5)

        # --- Card 1: Configuration ---
        self.conf_frame = tk.LabelFrame(self.workspace, text=translator.get("label.recording_configuration"), padx=10, pady=10)
        self.conf_frame.pack(fill="x", pady=5)
        
        tk.Label(self.conf_frame, text=translator.get("label.storage_directory")).pack(anchor="w")
        dir_frame = tk.Frame(self.conf_frame)
        dir_frame.pack(fill="x")
        self.ent_dir = tk.Entry(dir_frame)
        self.output_dir_var = tk.StringVar(value=str(self.workspace_path / "recordings"))

        # self.ent_dir.insert(0, "/home/khanhbv/Desktop/recordings")
        self.ent_dir = ttk.Entry(dir_frame, textvariable=self.output_dir_var, width=60)
        self.ent_dir.pack(side="left", fill="x", expand=True, padx=(0, 5))

        # tk.Button(dir_frame, text="Browse").pack(side="right")
        tk.Button(
            dir_frame,
            text=translator.get("button.browse"),
            command=self.browse_output_directory
        ).pack(side=tk.RIGHT)

        # --- Card 3: Control Area (Record & Upload) ---
        self.ctrl_frame = tk.Frame(self.workspace, relief="groove", borderwidth=2, padx=10, pady=10)
        self.ctrl_frame.pack(fill="x", pady=5)

        self.btn_record = tk.Button(self.ctrl_frame, text="● " + translator.get("button.start"), 
                                    font=("Arial", 10, "bold"), fg="red", command=self.toggle_recording)
        self.btn_record.pack(side="left", padx=10)
        
        self.lbl_status = tk.Label(self.ctrl_frame, text=translator.get("label.status") + ": " + translator.get("status.ready"))
        self.lbl_status.pack(side="left", padx=20)

        self.btn_rviz = tk.Button(
            self.ctrl_frame, 
            text="📊 " + translator.get("button.launch_rviz2"), 
            command=self.open_rviz,
            width=10
        )
        self.btn_rviz.pack(side="right", pady=10)  


        # Image canvas
        self.canvas = tk.Canvas(
            self.workspace,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

       # 4. Log Panel (Dưới cùng)
        self.log_header_frame = tk.Frame(self.root)
        self.log_header_frame.pack(side="top", fill="x", padx=10)
        
        tk.Label(self.log_header_frame, text=translator.get("label.system_logs"), font=("Arial", 9, "bold")).pack(side="left")
        
        self.btn_clear_log = tk.Button(self.log_header_frame, text=translator.get("button.clear_log"), 
                                       command=self.clear_logs, font=("Arial", 8), 
                                       padx=5, pady=0, bg="#e1e1e1")
        self.btn_clear_log.pack(side="right")

        # Thay đổi ở đây: Thêm fill="both" và expand=True
        self.log_container = tk.Frame(self.root, padx=5, pady=5)
        self.log_container.pack(side="bottom", fill="both", expand=True) 
        
        # Thay đổi ở đây: Tăng height lên (ví dụ 25 dòng)
        self.log_text = tk.Text(self.log_container, height=10, state="disabled", bg="#f8f8f8", font=("Consolas", 9))
        self.log_text.pack(side="left", fill="both", expand=True)
        
        self.scrollbar = tk.Scrollbar(self.log_container, command=self.log_text.yview)
        self.scrollbar.pack(side="right", fill="y")
        self.log_text.config(yscrollcommand=self.scrollbar.set)
        self.max_bag_size_var = tk.StringVar(value="2")

        for topic, config in self.topics_config.items():
            # Checkbox variable
            var = tk.BooleanVar(value=config["enabled"])
            self.topic_vars[topic] = var

    def open_rviz(self):
        """Mở RViz2 trực tiếp bằng subprocess"""
        rviz_config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.rviz")
        self.log(translator.get("log.launching_rviz2"))
        
        # Kiểm tra xem file có tồn tại không để tránh lỗi im lặng
        if not os.path.exists(rviz_config_file):
            self.log(translator.get("log.warning_config_not_found").replace("{path}", rviz_config_file))
            cmd = ['rviz2']
        else:
            cmd = ['rviz2', '-d', rviz_config_file]

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Ẩn log của RViz để đỡ rối terminal
                stderr=subprocess.STDOUT
            )
            self.log(translator.get("log.rviz2_opened"))
            
        except Exception as e:
            self.log(translator.get("log.error_rviz2").replace("{error}", str(e)))

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
            messagebox.showwarning(translator.get("dialog.warning"), translator.get("message.recording_in_progress"))
            return
        
        # Kiểm tra thư mục output
        output_dir = Path(self.output_dir_var.get())
        if not output_dir.exists():
            try:
                output_dir.mkdir(parents=True, exist_ok=True)
                self.log(translator.get("log.created_directory").replace("{path}", str(output_dir)))
            except Exception as e:
                messagebox.showerror(translator.get("dialog.error"), translator.get("message.cannot_create_directory").replace("{error}", str(e)))
                return
        
        # Kiểm tra và source drive_ws setup.sh để có CustomMsg
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = False
        
        if drive_ws_setup.exists():
            self.log(translator.get("log.checking_drive_ws"))
            if self._verify_source(drive_ws_setup, "drive_ws"):
                self.log(translator.get("log.drive_ws_ready"))
                use_drive_ws = True
            else:
                self.log(translator.get("log.cannot_verify_drive_ws"))
                use_drive_ws = True
        else:
            self.log(translator.get("log.warning_drive_ws_not_found"))
            self.log(translator.get("log.custommsg_may_not_recordable_log"))
            response = messagebox.askyesno(
                translator.get("dialog.warning"),
                translator.get("message.drive_ws_not_found")
            )
            if not response:
                return
        
        # Kiểm tra ws setup.sh
        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                translator.get("dialog.error"),
                translator.get("message.ws_setup_not_found").replace("{path}", str(ws_setup))
            )
            return
        
        self.log(translator.get("log.checking_ws_setup"))
        if self._verify_source(ws_setup, "ws"):
            self.log(translator.get("log.ws_setup_ready"))
        else:
            self.log(translator.get("log.cannot_verify_ws_setup"))
        
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
            messagebox.showerror(translator.get("dialog.error"), translator.get("message.select_at_least_one_topic"))
            return
        
        # Tạo tên bag file với timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"recording_{timestamp}"
        bag_path = output_dir / bag_name
        
        # Build command với source cả drive_ws và ws setup.sh
        # Cần source drive_ws trước để có CustomMsg, sau đó source ws
        topics_str = " ".join(selected_topics)
        
        self.log(translator.get("log.topics_selected").replace("{count}", str(len(selected_topics))).replace("{total}", str(len(self.topic_vars))))
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
                self.log(translator.get("log.adjusting_min_size").replace("{bytes:,}", f"{min_size_bytes:,}"))
            bag_record_cmd += f" --max-bag-size {max_bag_size_bytes}"
            self.log(translator.get("log.max_bag_size").replace("{size}", str(max_bag_size)).replace("{bytes:,}", f"{max_bag_size_bytes:,}"))
            self.log(translator.get("log.bag_files_will_split"))
        else:
            self.log(translator.get("log.no_bag_size_limit"))
        
        # Source theo thứ tự: ROS2 base -> drive_ws -> ws
        self.log(translator.get("log.preparing_source"))
        if use_drive_ws:
            cmd = (
                f"source {ros2_setup} && "
                f"source {drive_ws_setup} && "
                f"source {ws_setup} && "
                f"{bag_record_cmd}"
            )
            self.log(translator.get("log.will_source_with_drive_ws"))
            self.log(translator.get("log.custommsg_enabled"))
        else:
            cmd = (
                f"source {ros2_setup} && "
                f"source {ws_setup} && "
                f"{bag_record_cmd}"
            )
            self.log(translator.get("log.will_source_without_drive_ws"))
            self.log(translator.get("log.custommsg_may_not_work"))
        
        self.log(translator.get("log.starting_record_bag").replace("{name}", bag_name))
        self.log(translator.get("log.output_path").replace("{path}", str(bag_path)))
        self.log(translator.get("log.topics_list").replace("{topics}", topics_str))
        
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
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_record_process, daemon=True).start()
            
            self.log(translator.get("log.recording_started"))
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu record: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror(translator.get("dialog.error"), error_msg)
            self.is_recording = False
    
    def stop_recording(self):
        """Dừng record rosbag"""
        if not self.is_recording:
            return
        
        if self.record_process:
            try:
                self.log(translator.get("log.stopping_recording"))
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
        
        if self.output_dir:
            self.log(translator.get("log.recording_stopped_with_path").replace("{path}", str(self.output_dir)))
        else:
            self.log(translator.get("log.recording_stopped_no_path"))


    def browse_output_directory(self):
        """Browse cho output directory"""
        directory = filedialog.askdirectory(
            title=translator.get("message.select_directory_to_save"),
            initialdir=self.output_dir_var.get()
        )
        if directory:
            self.output_dir_var.set(directory)
            self.log(translator.get("log.directory_selected").replace("{path}", directory))

    def log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def clear_logs(self):
        """Xóa toàn bộ nội dung trong Log Text"""
        self.log_text.config(state="normal")
        self.log_text.delete('1.0', tk.END)
        self.log_text.config(state="disabled")
        self.log(translator.get("log.log_cleared"))

    # def toggle_recording(self):
    #     if not self.is_recording:
    #         self.btn_record.config(text="■ STOP RECORDING")
    #         self.lbl_status.config(text="Status: Recording...", fg="red")
    #         self.btn_upload.pack_forget()
    #         self.log("Recording started...")
    #         self.start_recording()
    #     else:
    #         self.btn_record.config(text="● START RECORDING")
    #         self.lbl_status.config(text="Status: File Saved", fg="green")
    #         self.btn_upload.pack(side="left", padx=10)
    #         self.log("Recording saved.")
    #         self.stop_recording()

    def toggle_recording(self):
        # Disable button để tránh double-click và race condition
        self.btn_record.config(state=tk.DISABLED)
        
        try:
            if not self.recorder.is_recording:
                # Gọi logic start từ file riêng
                success = self.recorder.start(
                    output_dir_str=self.output_dir_var.get(),
                    topic_vars=self.topic_vars,
                    workspace_path=self.workspace_path,
                    drive_ws_path=self.drive_ws_path,
                    max_bag_size_str=self.max_bag_size_var.get()
                )
                
                if success:
                    self.btn_record.config(text="■ " + translator.get("button.stop_recording"), state=tk.NORMAL)
                    self.lbl_status.config(text=translator.get("label.status_recording"), fg="red")
                    # self.btn_upload.pack_forget()
                else:
                    # Nếu start thất bại, enable lại button
                    self.btn_record.config(state=tk.NORMAL)
            else:
                # Gọi logic stop từ file riêng
                saved_path = self.recorder.stop()
                self.btn_record.config(text="● " + translator.get("button.start_recording"), state=tk.NORMAL)
                self.lbl_status.config(text=translator.get("label.status_saved"), fg="green")
                # self.btn_upload.pack(side="left", padx=10)
                if saved_path:
                    self.log(f"✅ Đã lưu tại: {saved_path}")
        except Exception as e:
            # Xử lý exception và đảm bảo button được enable lại
            self.log(f"❌ Lỗi trong toggle_recording: {e}")
            self.btn_record.config(state=tk.NORMAL)
            # Sync lại UI state
            if self.recorder.is_recording:
                self.btn_record.config(text="■ " + translator.get("button.stop_recording"))
                self.lbl_status.config(text=translator.get("label.status_recording"), fg="red")
            else:
                self.btn_record.config(text="● " + translator.get("button.start_recording"))
                self.lbl_status.config(text=translator.get("label.status") + ": " + translator.get("status.ready"), fg="black")

    def check_mid360_connection(self):
        """Kiểm tra kết nối và lưu vào biến self.is_mid360_connected"""
        ip = self.mid360_ip_var.get().strip()
        # Khởi tạo biến trạng thái mặc định là False
        self.is_mid360_connected = False 
        
        if not ip:
            self.log(translator.get("log.no_ip"))
            return

        def check():
            try:
                # Ping nhanh (1 gói tin, chờ 1 giây)
                # Dùng 'n' cho Windows, 'c' cho Linux
                param = '-n' if subprocess.os.name == 'nt' else '-c'
                result = subprocess.run(['ping', param, '1', '-W', '1', ip], 
                                    capture_output=True, text=True)
                
                if result.returncode == 0:
                    self.is_mid360_connected = True
                    self.label_slam_usb.config(
                        text="● " + translator.get("label.livox_360") + ": " + translator.get("status.connected"),
                        fg="green"
                    )
                    self.log(translator.get("log.livox_mid360_connected"))

                else:
                    self.is_mid360_connected = False
                    self.label_slam_usb.config(
                        text="● " + translator.get("label.livox_360") + ": " + translator.get("status.disconnected"),
                        fg="red"
                    )
                    self.log(translator.get("log.livox_mid360_disconnected"))
            except Exception:
                self.is_mid360_connected = False
                self.log(translator.get("log.livox_mid360_error"))


        # Chạy ngầm để không treo giao diện (GUI)
        threading.Thread(target=check, daemon=True).start()

    def check_theta_usb_callback(self, is_connected):
        """Callback để cập nhật UI dựa trên trạng thái kết nối USB của Theta"""
        if is_connected:
            self.label_theta_usb.config(text="● " + translator.get("label.camera") + ": " + translator.get("status.connected"), fg="green")
        else:
            self.label_theta_usb.config(text="● " + translator.get("label.camera") + ": " + translator.get("status.disconnected"), fg="red")
    
    def update_ui_theta_connected(self, is_active_theta):
        """Hàm này chuyên trách việc đổi màu/text trên giao diện"""
        # Kiểm tra xem các widget đã được khởi tạo chưa
        if not hasattr(self, 'label_theta') or not hasattr(self, 'btn_start_livox'):
            # Nếu widget chưa được khởi tạo, schedule lại sau
            self.root.after(100, lambda: self.update_ui_theta_connected(is_active_theta))
            return
        
        try:
            # Xác định màu sắc dựa trên biến success
            color = "green" if is_active_theta else "red"
            
            # Xác định nội dung text dựa trên biến success
            status_text = translator.get("status.running") if is_active_theta else translator.get("status.not_running")

            print(f"UI cập nhật: is_active_theta={is_active_theta}")
            
            # Cập nhật UI Theta
            full_text_theta = f"● {translator.get('label.theta_driver')}: {status_text}"

            self.label_theta.config(text=full_text_theta, fg=color)
            if is_active_theta:
                self.btn_start_livox.config(state=tk.DISABLED)
                self.btn_stop_livox.config(state=tk.NORMAL)
            else:
                self.btn_start_livox.config(state=tk.NORMAL)
                self.btn_stop_livox.config(state=tk.DISABLED)
        except Exception as e:
            # Log lỗi nhưng không crash
            print(f"⚠️  Lỗi khi cập nhật UI theta: {e}")
            import traceback
            traceback.print_exc()

        # self.label_livox.config(text=full_text_livox, fg=color)
    
    def stop_livox_driver(self):
        """Stop tất cả drivers và đảm bảo cleanup hoàn toàn"""
        try:
            # Stop theo thứ tự để tránh dependency issues
            if hasattr(self, 'livox_tab') and self.livox_tab:
                self.livox_tab.stop_ros_subscriber()
                self.livox_tab.stop_converter()
                self.livox_tab.stop_livox_driver()
            
            if hasattr(self, 'theta_driver') and self.theta_driver:
                self.theta_driver.stop_all()
            
            # Chờ một chút để đảm bảo processes đã dừng
            time.sleep(0.5)
            
            # Cập nhật UI để enable lại nút start và disable nút stop
            self.update_ui_theta_connected(False)
        except Exception as e:
            self.log(translator.get("log.error_stop_drivers").replace("{error}", str(e)))
            # Đảm bảo UI được cập nhật ngay cả khi có lỗi
            self.update_ui_theta_connected(False)
    
    def start_livox_driver(self):
        """Start livox driver với proper sequencing và cleanup"""
        # Disable nút bấm để tránh người dùng click loạn xạ gây crash
        self.btn_start_livox.config(state=tk.DISABLED)
        
        def run_start():
            success = False
            try:
                # Đảm bảo stop tất cả processes cũ trước
                self.log(translator.get("log.stopping_recording"))
                self.stop_livox_driver()
                
                # Chờ đủ lâu để các process cũ giải phóng port và resources
                time.sleep(1.5)
                
                self.log(translator.get("log.starting_livox_theta_drivers"))
                self.check_mid360_connection()
                
                # Khởi chạy các driver theo thứ tự
                self.theta_driver.check_theta_usb_connection(self.check_theta_usb_callback)
                self.theta_driver.launch_theta_driver()
                
                # Chờ một chút để theta driver khởi động
                time.sleep(0.5)
                
                self.livox_tab.start_livox_driver()
                
                self.log(translator.get("log.system_restarted"))
                success = True
            except Exception as e:
                self.log(translator.get("log.error_start_drivers").replace("{error}", str(e)))
                import traceback
                self.log(translator.get("log.details").replace("{details}", traceback.format_exc()))
            finally:
                # Chỉ enable lại nút start nếu có lỗi
                # Nếu start thành công, nút start sẽ được disable bởi update_ui_theta_connected()
                if not success:
                    self.root.after(0, lambda: self.btn_start_livox.config(state=tk.NORMAL))
        
        # Chạy trong thread riêng để không block UI
        threading.Thread(target=run_start, daemon=True).start()
            
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
            self.log(f"{translator.get('log.error_reading_output')} {e}")
        
        # Kiểm tra exit code
        # if self.record_process.poll() is not None:
        #     exit_code = self.record_process.poll()
        #     if exit_code != 0:
        #         self.log(f"✗ Recording đã dừng với exit code: {exit_code}")
        #     else:
        #         self.log(f"✓ Recording đã dừng bình thường")
            
        #     self.is_recording = False

if __name__ == "__main__":
    root = tk.Tk()
    app = LivoxApp(root)
    root.mainloop()