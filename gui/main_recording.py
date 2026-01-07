import tkinter as tk
from tkinter import ttk
import threading
import time
from datetime import datetime
from pathlib import Path
import subprocess
import os
from functools import partial
from languages.translate_engine import translator

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
        self._setup_layout()
    
    def toggle_language(self):
        # Logic đổi qua lại giữa 'en' và 'jp'
        new_lang = 'en' if translator.lang_code == 'jp' else 'jp'
        translator.switch_language(new_lang)
        
        # Cập nhật lại tiêu đề cửa sổ
        self.root.title(translator.get("title.livox_panel"))
        
        # Làm mới toàn bộ UI
        self.refresh_ui()

    def refresh_ui(self):
        # Xóa các thành phần hiện tại để render lại với ngôn ngữ mới
        for widget in self.root.winfo_children():
            widget.destroy()
            
        # Khởi tạo lại dictionary lưu biến
        self.topic_vars = {}
        # Gọi lại hàm dựng giao diện
        self._setup_layout()

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

        tk.Checkbutton(self.sidebar, text=translator.get("label.enable_converter")).pack(anchor="w")

        btn_driver_frame = tk.Frame(self.sidebar)
        btn_driver_frame.pack(fill="x", pady=10)
        tk.Button(btn_driver_frame, text=translator.get("button.start"), width=10).pack(side="left", padx=2)
        tk.Button(btn_driver_frame, text=translator.get("button.stop"), width=10).pack(side="left", padx=2)
        tk.Label(self.sidebar, text="● " + translator.get("label.lidar_topic") + ": " + translator.get("status.active"), fg="green").pack(anchor="w", pady=5)
        tk.Label(self.sidebar, text="● " + translator.get("label.imu_topic") + ": " + translator.get("status.active"), fg="green").pack(anchor="w")

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

        self.btn_upload = tk.Button(self.ctrl_frame, text="↑ " + translator.get("button.upload"), command=self.start_upload_thread)
        
        self.lbl_status = tk.Label(self.ctrl_frame, text=translator.get("label.status") + ": " + translator.get("status.ready"))
        self.lbl_status.pack(side="left", padx=20)

        self.lbl_percent = tk.Label(self.ctrl_frame, text="0%")

        self.progress = ttk.Progressbar(self.workspace, orient="horizontal", mode="determinate")

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
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_record_process, daemon=True).start()
            
            self.log("✅ Recording đã được khởi động")
            
        except Exception as e:
            error_msg = f"Không thể bắt đầu record: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            self.is_recording = False
    
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
        
        if self.output_dir:
            self.log(f"✅ Recording đã dừng. Bag file: {self.output_dir}")
        else:
            self.log("✅ Recording đã dừng")


    def browse_output_directory(self):
        """Browse cho output directory"""
        directory = filedialog.askdirectory(
            title="Chọn thư mục để lưu recording",
            initialdir=self.output_dir_var.get()
        )
        if directory:
            self.output_dir_var.set(directory)
            self.log(f"Đã chọn thư mục: {directory}")

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
        self.log("Log cleared.")

    def toggle_recording(self):
        if not self.is_recording:
            self.btn_record.config(text="■ STOP RECORDING")
            self.lbl_status.config(text="Status: Recording...", fg="red")
            self.btn_upload.pack_forget()
            self.log("Recording started...")
            self.start_recording()
        else:
            self.btn_record.config(text="● START RECORDING")
            self.lbl_status.config(text="Status: File Saved", fg="green")
            self.btn_upload.pack(side="left", padx=10)
            self.log("Recording saved.")
            self.stop_recording()

    def start_upload_thread(self):
        self.btn_upload.config(state="disabled")
        self.progress.pack(fill="x", padx=20, pady=5)
        self.lbl_percent.pack(side="right", padx=20)
        threading.Thread(target=self.simulate_upload, daemon=True).start()

    def simulate_upload(self):
        self.log("Starting upload to server...")
        for i in range(101):
            time.sleep(0.04)
            self.progress['value'] = i
            self.lbl_percent.config(text=f"Uploading: {i}%")
        
        self.log("SUCCESS: Upload complete.")
        self.btn_upload.config(state="normal")
        time.sleep(1)
        self.progress.pack_forget()
        self.lbl_percent.pack_forget()
    
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