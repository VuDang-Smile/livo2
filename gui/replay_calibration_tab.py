#!/usr/bin/env python3
"""
Replay Calibration Tab Module
Clone từ ReplayTab để dùng riêng cho Calibration
"""

import threading
import subprocess
from pathlib import Path
from datetime import datetime
import os
import time
from functools import partial
from bag_cut_tab import cut_bag_5s_data

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class ReplayCalibrationTab(ttk.Frame):
    """Tab Replay dành cho Calibration"""

    def __init__(self, parent):
        super().__init__(parent)

        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"

        # Process
        self.replay_process = None
        self.record_process = None

        # State
        self.is_replaying = False
        self.bag_path = None
        self.is_cutting = False
        self.cut_output_path = None
        self.is_recording = False
        self.replay_start_time = None
        self.bag_duration_seconds = None
        # Replay options (giữ mặc định, ẩn UI)
        self.rate_var = tk.StringVar(value="1.0")
        self.loop_var = tk.BooleanVar(value=False)
        self.clock_var = tk.BooleanVar(value=True)

        # Tạo UI
        self.create_widgets()

    def create_widgets(self):
        """Tạo các widget cho tab Replay Calibration"""

        # Title
        title_label = ttk.Label(
            self,
            text="Replay Calibration",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)

        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)

        # Frame chọn bag folder
        bag_frame = ttk.LabelFrame(control_frame, text="Chọn Bag Folder", padding="10")
        bag_frame.pack(fill=tk.X, padx=10, pady=5)

        self.bag_path_var = tk.StringVar()
        bag_entry = ttk.Entry(bag_frame, textvariable=self.bag_path_var, width=60)
        bag_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        browse_btn = ttk.Button(
            bag_frame,
            text="Browse",
            command=self.browse_bag_folder
        )
        browse_btn.pack(side=tk.LEFT, padx=5)

        # Frame nút điều khiển
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, padx=10, pady=10)

        self.start_btn = ttk.Button(
            button_frame,
            text="Start Replay",
            command=self.start_replay,
            style="Accent.TButton"
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(
            button_frame,
            text="Stop Replay",
            command=self.stop_replay,
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

        # Frame thông tin bag
        info_frame = ttk.LabelFrame(self, text="Thông tin Bag", padding="10")
        info_frame.pack(fill=tk.X, padx=10, pady=5)

        self.info_label = ttk.Label(
            info_frame,
            text="Chưa chọn bag folder",
            font=("Arial", 10)
        )
        self.info_label.pack(anchor=tk.W, padx=5)

        self.cut_status_label = ttk.Label(
            info_frame,
            text="Chưa cắt 5s",
            font=("Arial", 9),
            foreground="gray"
        )
        self.cut_status_label.pack(anchor=tk.W, padx=5, pady=(2, 0))

        # Text area để hiển thị log riêng cho tab Replay
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
        """Thêm message vào log của tab Replay"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        if hasattr(self, "log_text") and self.log_text:
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        else:
            # Fallback nếu UI chưa sẵn sàng
            print(f"[{timestamp}] {message}")

    def browse_bag_folder(self):
        """Browse cho bag folder"""
        initial_dir = "/media/an/01DC80D9DB838380"
        if not Path(initial_dir).exists():
            initial_dir = str(self.workspace_path)

        folder = filedialog.askdirectory(
            title="Chọn thư mục chứa bag files",
            initialdir=self.bag_path_var.get() or initial_dir
        )
        if folder:
            self.bag_path_var.set(folder)
            self._update_bag_info(folder)
            self.log(f"Đã chọn bag folder: {folder}")

    def _update_bag_info(self, bag_path):
        """Cập nhật thông tin về bag"""
        try:
            bag_path_obj = Path(bag_path)
            if not bag_path_obj.exists():
                self.info_label.config(text="Bag folder không tồn tại")
                return

            has_metadata = (bag_path_obj / "metadata.yaml").exists()
            has_db3 = any(bag_path_obj.glob("*.db3"))

            if has_metadata or has_db3:
                self.info_label.config(text=f"Bag folder: {bag_path_obj.name}")
                self.log("Đang lấy thông tin bag...")
                self._get_bag_info(bag_path)
            else:
                self.info_label.config(text=f"Thư mục: {bag_path_obj.name} (chưa xác nhận là bag folder)")
        except Exception as e:
            self.info_label.config(text=f"Lỗi: {e}")

    def _get_bag_info(self, bag_path):
        """Lấy thông tin về bag file"""
        try:
            ws_setup = self.workspace_path / "install" / "setup.sh"
            if not ws_setup.exists():
                return

            ros2_setup = "/opt/ros/jazzy/setup.bash"
            cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag info {bag_path}"

            result = subprocess.run(
                cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                output_lines = result.stdout.split('\n')
                info_text = "Bag info:\n"
                for line in output_lines[:10]:
                    if line.strip():
                        info_text += f"  {line}\n"
                self.info_label.config(text=info_text)
                self.log("✓ Đã lấy thông tin bag thành công")
            else:
                self.log(f"⚠️  Không thể lấy thông tin bag: {result.stderr}")
        except Exception as e:
            self.log(f"⚠️  Lỗi khi lấy thông tin bag: {e}")

    def cut_5s_data(self):
        """Cắt đúng 5s dữ liệu (timestamp) và tự động load vào Replay"""
        if self.is_cutting:
            messagebox.showinfo("Đang cắt", "Đang cắt bag, vui lòng chờ...")
            return

        bag_path = self.bag_path_var.get()
        if not bag_path:
            messagebox.showerror("Lỗi", "Vui lòng chọn bag folder")
            return

        bag_path_obj = Path(bag_path)
        if not bag_path_obj.exists():
            messagebox.showerror("Lỗi", f"Bag folder không tồn tại: {bag_path}")
            return

        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}")
            return

        self.is_cutting = True
        self.cut_output_path = None
        self.cut_status_label.config(text="Đang cắt 5s dữ liệu...", foreground="orange")
        self.log("🔧 Đang cắt 5s dữ liệu dựa trên timestamp...")

        def worker():
            try:
                ok, out_path, msg = cut_bag_5s_data(
                    bag_path=bag_path_obj,
                    workspace_path=self.workspace_path,
                    drive_ws_path=self.drive_ws_path,
                    logger=self.log,
                    duration_seconds=5.0,
                )
                self.after(0, lambda: self._update_cut_result(ok, out_path, msg))
            except Exception as e:
                self.after(0, lambda: self._update_cut_result(False, None, str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def _update_cut_result(self, success, out_path, message):
        """Update UI sau khi cắt bag"""
        self.is_cutting = False

        if success and out_path:
            self.cut_output_path = out_path
            self.cut_status_label.config(text=f"Đã cắt: {out_path}", foreground="green")
            self.bag_path_var.set(out_path)
            self._update_bag_info(out_path)
            self.log(f"✅ Cắt thành công -> {out_path}")
        else:
            self.cut_status_label.config(text=f"Cắt thất bại: {message}", foreground="red")
            self.log(f"❌ Cắt thất bại: {message}")
            messagebox.showerror("Lỗi", f"Cắt bag thất bại: {message}")

    def start_replay(self):
        """Cắt 5s -> Record -> Replay -> Khi replay kết thúc thì dừng record"""
        if self.is_replaying or self.is_cutting or self.is_recording:
            messagebox.showwarning("Cảnh báo", "Đang chạy tác vụ khác, vui lòng dừng trước")
            return

        bag_path = self.bag_path_var.get()
        if not bag_path:
            messagebox.showerror("Lỗi", "Vui lòng chọn bag folder")
            return

        bag_path_obj = Path(bag_path)
        if not bag_path_obj.exists():
            messagebox.showerror("Lỗi", f"Bag folder không tồn tại: {bag_path}")
            return

        ws_setup = self.workspace_path / "install" / "setup.sh"
        if not ws_setup.exists():
            messagebox.showerror(
                "Lỗi",
                f"Không tìm thấy ws/install/setup.sh tại: {ws_setup}\n"
                "Vui lòng build workspace trước."
            )
            return

        # UI trạng thái cắt trước khi replay
        self.is_cutting = True
        self.cut_output_path = None
        self.cut_status_label.config(text="Đang cắt 5s dữ liệu trước khi replay...", foreground="orange")
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Trạng thái: Đang cắt 5s...", foreground="orange")
        self.log("🔧 Đang cắt 5s dữ liệu (timestamp) trước khi replay...")

        def worker():
            try:
                ok, out_path, msg = cut_bag_5s_data(
                    bag_path=bag_path_obj,
                    workspace_path=self.workspace_path,
                    drive_ws_path=self.drive_ws_path,
                    logger=self.log,
                    duration_seconds=5.0,
                )
                self.after(0, lambda: self._after_cut_and_play(ok, out_path, msg))
            except Exception as e:
                self.after(0, lambda: self._after_cut_and_play(False, None, str(e)))

        threading.Thread(target=worker, daemon=True).start()

    def _after_cut_and_play(self, success, out_path, message):
        """Sau khi cắt xong, tự động replay file đã cắt"""
        self.is_cutting = False
        if not success or not out_path:
            self.cut_status_label.config(text=f"Cắt thất bại: {message}", foreground="red")
            self.status_label.config(text="Trạng thái: Sẵn sàng", foreground="gray")
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            messagebox.showerror("Lỗi", f"Cắt bag thất bại: {message}")
            return

        # Cập nhật info và dùng file đã cắt để replay
        self.cut_output_path = out_path
        self.cut_status_label.config(text=f"Đã cắt: {out_path}", foreground="green")
        self.bag_path_var.set(out_path)
        self._update_bag_info(out_path)
        self.log(f"✅ Cắt thành công -> {out_path}")

        # Thực hiện record rồi replay với file đã cắt
        self._start_record_and_replay(Path(out_path))

    def _start_record_and_replay(self, bag_path_obj: Path):
        """Record song song và replay với bag đã cắt"""
        if self.is_replaying or self.is_recording:
            return

        bag_path = str(bag_path_obj)
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()

        try:
            rate = float(self.rate_var.get())
            if rate <= 0:
                rate = 1.0
        except ValueError:
            rate = 1.0
            self.log("⚠️  Giá trị rate không hợp lệ, sử dụng mặc định 1.0")

        loop = self.loop_var.get()
        clock = self.clock_var.get()

        ros2_setup = "/opt/ros/jazzy/setup.bash"
        ws_setup = self.workspace_path / "install" / "setup.sh"
        bag_play_cmd = f"ros2 bag play {bag_path}"
        if rate != 1.0:
            bag_play_cmd += f" --rate {rate}"
        if loop:
            bag_play_cmd += " --loop"
        if clock:
            bag_play_cmd += " --clock"

        self.log("🔧 Đang chuẩn bị source environment...")
        if use_drive_ws:
            cmd = (
                f"source {ros2_setup} && "
                f"source {drive_ws_setup} && "
                f"source {ws_setup} && "
                f"{bag_play_cmd}"
            )
            self.log("✅ Sẽ source: ROS2 base -> drive_ws -> ws")
            self.log("✅ CustomMsg support đã được kích hoạt")
        else:
            cmd = (
                f"source {ros2_setup} && "
                f"source {ws_setup} && "
                f"{bag_play_cmd}"
            )
            self.log("⚠️  Sẽ source: ROS2 base -> ws (không có drive_ws)")
            self.log("⚠️  CustomMsg có thể không hoạt động")

        # Chuẩn bị record (song song khi replay)
        record_dir = self.workspace_path / "calibration_data" / "bags"
        record_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        record_bag_path = record_dir / f"replay_capture_{timestamp}"
        topics = ["/image_raw", "/camera_info", "/livox/points2"]
        record_cmd = f"ros2 bag record -o {record_bag_path} {' '.join(topics)}"

        self.log(f"📝 Bắt đầu record song song khi replay: {record_bag_path}")
        self.log(f"📝 Bắt đầu replay bag: {bag_path_obj.name}")
        self.log(f"📁 Bag path: {bag_path}")
        self.log(f"⚙️  Rate: {rate}x")
        self.log(f"⚙️  Loop: {'Có' if loop else 'Không'}")
        self.log(f"⚙️  Clock: {'Có' if clock else 'Không'}")

        try:
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'

            # Khởi chạy record trước
            self.record_process = subprocess.Popen(
                (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"{record_cmd}"
                ),
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env,
            )
            self.is_recording = True
            self.log(f"✅ Record bắt đầu: {record_bag_path}")

            # Khởi chạy replay
            self.replay_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )

            self.is_replaying = True
            self.bag_path = bag_path
            
            # Lấy duration của bag để tự động dừng sau khi phát xong
            self.bag_duration_seconds = self._get_bag_duration(bag_path_obj)
            self.replay_start_time = time.time()
            
            if self.bag_duration_seconds:
                expected_duration = self.bag_duration_seconds / rate + 2.0  # +2s buffer
                self.log(f"⏱️  Bag duration: {self.bag_duration_seconds:.2f}s, sẽ tự động dừng sau ~{expected_duration:.1f}s")
            else:
                self.log("⚠️  Không thể lấy duration của bag, sẽ monitor process exit")
            
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(
                text="Trạng thái: Đang replay...",
                foreground="orange"
            )
            self.info_label.config(
                text=f"Đang replay: {bag_path_obj.name}\nRate: {rate}x | Loop: {'Có' if loop else 'Không'} | Clock: {'Có' if clock else 'Không'}"
            )

            # Khởi chạy monitor replay process
            threading.Thread(target=self.monitor_replay_process, daemon=True).start()
            
            # Khởi chạy auto-stop timer nếu có duration và không loop
            if self.bag_duration_seconds and not loop:
                threading.Thread(
                    target=self._auto_stop_replay_after_duration,
                    args=(expected_duration,),
                    daemon=True
                ).start()

            self.log("✅ Replay đã được khởi động")

        except Exception as e:
            error_msg = f"Không thể bắt đầu replay: {e}"
            self.log(f"❌ Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            # Nếu record đã khởi chạy thì dừng
            self._stop_record_process()
            self.is_replaying = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)

    def _stop_record_process(self):
        """Dừng record nếu đang chạy"""
        if self.record_process:
            try:
                self.log("Đang dừng record song song...")
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

    def stop_replay(self):
        """Dừng replay bag"""
        if not self.is_replaying:
            return

        if self.replay_process:
            try:
                self.log("Đang dừng replay...")
                self.replay_process.terminate()
                try:
                    self.replay_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.replay_process.kill()
                    self.replay_process.wait()
            except Exception as e:
                self.log(f"Lỗi khi dừng replay: {e}")

            self.replay_process = None

        # Dừng record nếu đang chạy
        self._stop_record_process()

        self.is_replaying = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="green"
        )

        if self.bag_path:
            bag_name = Path(self.bag_path).name
            self.info_label.config(
                text=f"Đã dừng replay\nBag: {bag_name}"
            )
            self.log(f"✅ Replay đã dừng. Bag: {bag_name}")
        else:
            self.info_label.config(text="Đã dừng replay")
            self.log("✅ Replay đã dừng")

    def _get_bag_duration(self, bag_path_obj: Path) -> float:
        """Lấy duration của bag file (seconds)"""
        try:
            # Thử đọc từ metadata.yaml trước
            metadata_file = bag_path_obj / "metadata.yaml"
            if metadata_file.exists():
                import yaml
                data = yaml.safe_load(metadata_file.read_text())
                info = data.get("rosbag2_bagfile_information", {}) or {}
                duration_ns = (info.get("duration") or {}).get("nanoseconds")
                if duration_ns:
                    return duration_ns / 1e9  # Convert nanoseconds to seconds
        except Exception as e:
            self.log(f"⚠️  Không thể lấy duration từ metadata.yaml: {e}")
        
        # Fallback: thử dùng ros2 bag info
        try:
            ws_setup = self.workspace_path / "install" / "setup.sh"
            if not ws_setup.exists():
                return None

            ros2_setup = "/opt/ros/jazzy/setup.bash"
            cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag info {bag_path_obj}"

            result = subprocess.run(
                cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                import re
                # Tìm pattern "Duration: X.XXXs" hoặc "Duration: HH:MM:SS.mmm"
                for line in result.stdout.split('\n'):
                    if 'duration' in line.lower():
                        # Tìm số giây
                        match = re.search(r'(\d+\.\d+)\s*s', line.lower())
                        if match:
                            return float(match.group(1))
                        # Hoặc format HH:MM:SS.mmm
                        match = re.search(r'(\d+):(\d+):(\d+\.?\d*)', line)
                        if match:
                            hours = int(match.group(1))
                            minutes = int(match.group(2))
                            seconds = float(match.group(3))
                            return hours * 3600 + minutes * 60 + seconds
        except Exception as e:
            self.log(f"⚠️  Không thể lấy duration từ ros2 bag info: {e}")
        
        return None

    def _auto_stop_replay_after_duration(self, wait_seconds: float):
        """Tự động dừng replay sau khi đã phát xong bag"""
        try:
            time.sleep(wait_seconds)
            
            # Kiểm tra xem replay còn đang chạy không
            if self.is_replaying and self.replay_process:
                if self.replay_process.poll() is None:
                    # Process vẫn đang chạy, tự động dừng
                    self.log(f"⏱️  Đã phát đủ thời gian ({wait_seconds:.1f}s), tự động dừng replay...")
                    # Đợi thêm để đảm bảo messages cuối cùng được record
                    time.sleep(2)
                    self.after(0, self.stop_replay)
                else:
                    # Process đã kết thúc, monitor sẽ xử lý
                    pass
        except Exception as e:
            self.log(f"⚠️  Lỗi trong auto-stop timer: {e}")

    def monitor_replay_process(self):
        """Monitor replay process output và tự động dừng record khi replay kết thúc"""
        if not self.replay_process:
            return

        try:
            for line in iter(self.replay_process.stdout.readline, ''):
                if not line:
                    break
                line = line.strip()
                if line:
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed']):
                        self.log(f"❌ ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️  WARNING: {line}")
                    else:
                        if any(keyword in line.lower() for keyword in ['playing', 'paused', 'finished', 'topic', 'message']):
                            self.log(line)
        except Exception as e:
            self.log(f"Lỗi khi đọc output: {e}")

        # Kiểm tra xem replay process đã kết thúc chưa
        # Đợi một chút để process có thể kết thúc tự nhiên
        time.sleep(0.5)
        
        # Kiểm tra nhiều lần để đảm bảo detect khi process kết thúc
        max_checks = 10
        for i in range(max_checks):
            if self.replay_process.poll() is not None:
                exit_code = self.replay_process.poll()
                if exit_code != 0:
                    self.log(f"✗ Replay đã dừng với exit code: {exit_code}")
                else:
                    self.log(f"✓ Replay đã hoàn thành (process đã exit)")
                break
            time.sleep(0.2)
        
        # Nếu process vẫn đang chạy sau khi đã đợi đủ thời gian, kiểm tra lại
        if self.replay_process.poll() is None:
            # Process vẫn đang chạy, có thể đã phát xong nhưng chưa exit
            # Kiểm tra xem đã đủ thời gian chưa (nếu có duration)
            if self.bag_duration_seconds and self.replay_start_time:
                elapsed = time.time() - self.replay_start_time
                rate = float(self.rate_var.get()) if self.rate_var.get() else 1.0
                expected_duration = self.bag_duration_seconds / rate + 2.0
                
                if elapsed >= expected_duration:
                    self.log(f"⏱️  Đã phát {elapsed:.1f}s (dự kiến {expected_duration:.1f}s), tự động dừng replay...")
                    # Tự động dừng replay
                    if self.replay_process and self.replay_process.poll() is None:
                        self.replay_process.terminate()
                        try:
                            self.replay_process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            self.replay_process.kill()
                            self.replay_process.wait()
                    self._handle_replay_finished()
                    return
        
        # Process đã kết thúc hoặc đã được dừng
        if self.replay_process.poll() is not None:
            self._handle_replay_finished()

    def _handle_replay_finished(self):
        """Xử lý khi replay kết thúc"""
        try:
            # Đợi một chút để đảm bảo messages cuối cùng được record
            self.log("⏳ Đợi để đảm bảo tất cả messages đã được record...")
            time.sleep(2)  # Đợi 2 giây để messages cuối cùng được record

            # Dừng record khi replay kết thúc
            self.log("🛑 Đang dừng record vì replay đã kết thúc...")
            self._stop_record_process()

            self.is_replaying = False
            self.replay_start_time = None
            self.bag_duration_seconds = None
            self.after(0, partial(self._update_replay_stopped))
        except Exception as e:
            self.log(f"❌ Lỗi khi xử lý replay finished: {e}")
            self._stop_record_process()
            self.is_replaying = False
            self.after(0, partial(self._update_replay_stopped))

    def _update_replay_stopped(self):
        """Helper function để update UI sau khi replay dừng"""
        try:
            # Đảm bảo cả replay và record đều đã được dừng
            if self.replay_process:
                try:
                    if self.replay_process.poll() is None:
                        self.replay_process.terminate()
                        try:
                            self.replay_process.wait(timeout=2)
                        except subprocess.TimeoutExpired:
                            self.replay_process.kill()
                            self.replay_process.wait()
                except Exception:
                    pass
                self.replay_process = None

            # Đảm bảo record cũng đã được dừng
            if self.is_recording or self.record_process:
                self._stop_record_process()

            # Reset state
            self.is_replaying = False
            self.is_recording = False
            self.replay_start_time = None
            self.bag_duration_seconds = None

            # Update UI
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)
            self.status_label.config(
                text="Trạng thái: Đã hoàn thành",
                foreground="green"
            )
            if self.bag_path:
                bag_name = Path(self.bag_path).name
                self.info_label.config(
                    text=f"Replay đã hoàn thành\nBag: {bag_name}\nRecord đã được dừng tự động"
                )
                try:
                    messagebox.showinfo("Replay hoàn thành", f"Đã phát xong: {bag_name}\nRecord đã được dừng tự động")
                except Exception:
                    pass
            else:
                self.info_label.config(text="Replay đã hoàn thành\nRecord đã được dừng")
        except Exception as e:
            self.log(f"❌ Lỗi khi update UI: {e}")
            # Đảm bảo state được reset ngay cả khi có lỗi
            self.is_replaying = False
            self.is_recording = False
            self.replay_process = None
            self.record_process = None

