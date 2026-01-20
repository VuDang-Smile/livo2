#!/usr/bin/env python3
"""
Bag Cut Tab Module
Tạo tab để cắt 5 giây đầu của một rosbag mà không ảnh hưởng Replay cũ.
"""

import subprocess
import threading
import os
import time
from pathlib import Path
from datetime import datetime

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class BagCutTab(ttk.Frame):
    """Tab để cắt 5 giây đầu của bag file"""

    def __init__(self, parent, log_callback=None):
        super().__init__(parent)

        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"

        self.log_global_cb = log_callback

        # State
        self.is_cutting = False
        self.cut_output_path = None
        self.record_process = None
        self.play_process = None

        self.create_widgets()

    def create_widgets(self):
        title = ttk.Label(self, text="Bag Cutter (5s)", font=("Arial", 16, "bold"))
        title.pack(pady=10)

        # Bag selection
        select_frame = ttk.LabelFrame(self, text="Chọn Bag Folder", padding=10)
        select_frame.pack(fill=tk.X, padx=10, pady=5)

        self.bag_path_var = tk.StringVar()
        bag_entry = ttk.Entry(select_frame, textvariable=self.bag_path_var, width=70)
        bag_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        browse_btn = ttk.Button(select_frame, text="Browse", command=self.browse_bag_folder)
        browse_btn.pack(side=tk.LEFT, padx=5)

        # Control
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        self.cut_btn = ttk.Button(control_frame, text="Cut 5 seconds", command=self.cut_bag, style="Accent.TButton")
        self.cut_btn.pack(side=tk.LEFT, padx=5)

        self.status_label = ttk.Label(control_frame, text="Trạng thái: Sẵn sàng", foreground="gray")
        self.status_label.pack(side=tk.LEFT, padx=15)

        # Info
        self.cut_status_label = ttk.Label(self, text="Chưa cắt", foreground="gray")
        self.cut_status_label.pack(anchor=tk.W, padx=15, pady=5)

        # Log
        log_frame = ttk.LabelFrame(self, text="Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=12, wrap=tk.WORD, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        msg = f"[{timestamp}] {message}"
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        if self.log_global_cb:
            try:
                self.log_global_cb(message, "[BagCut] ")
            except Exception:
                pass

    def browse_bag_folder(self):
        directory = filedialog.askdirectory(
            title="Chọn thư mục chứa bag",
            initialdir=self.bag_path_var.get() or str(self.workspace_path)
        )
        if directory:
            self.bag_path_var.set(directory)
            self.log(f"Đã chọn bag folder: {directory}")

    def cut_bag(self):
        if self.is_cutting:
            messagebox.showinfo("Đang chạy", "Đang cắt bag, vui lòng chờ...")
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

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = bag_path_obj.parent / f"{bag_path_obj.name}_5sec_{timestamp}"

        ros2_setup = "/opt/ros/jazzy/setup.bash"
        drive_ws_setup = self.drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()

        self.is_cutting = True
        self.cut_output_path = None
        self.cut_status_label.config(text="Đang cắt...", foreground="orange")
        self.status_label.config(text="Trạng thái: Đang cắt...", foreground="orange")
        self.cut_btn.config(state=tk.DISABLED)
        self.log(f"🔧 Đang cắt bag (5s) -> {output_dir}")

        def worker():
            try:
                env = os.environ.copy()
                if "ROS_DOMAIN_ID" not in env:
                    env["ROS_DOMAIN_ID"] = "0"

                record_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag record -a -o {output_dir}"
                play_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag play {bag_path} --duration 5 --rate 1.0 --clock"
                if use_drive_ws:
                    record_cmd = f"source {ros2_setup} && source {drive_ws_setup} && source {ws_setup} && ros2 bag record -a -o {output_dir}"
                    play_cmd = f"source {ros2_setup} && source {drive_ws_setup} && source {ws_setup} && ros2 bag play {bag_path} --duration 5 --rate 1.0 --clock"

                self.log("▶️ Khởi động record...")
                self.record_process = subprocess.Popen(
                    record_cmd,
                    shell=True,
                    executable="/bin/bash",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env,
                )

                time.sleep(1.0)

                self.log("▶️ Play bag 5s...")
                self.play_process = subprocess.Popen(
                    play_cmd,
                    shell=True,
                    executable="/bin/bash",
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env,
                )

                try:
                    self.play_process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    self.log("⚠️ Play timeout, dừng play")
                    self.play_process.terminate()
                    self.play_process.wait()

                time.sleep(0.5)

                self.log("⏹️ Dừng record...")
                self.record_process.terminate()
                try:
                    self.record_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.record_process.kill()
                    self.record_process.wait()

                # Kiểm tra file và duration
                duration_ok = False
                if output_dir.exists() and (any(output_dir.glob("*.mcap")) or any(output_dir.glob("*.db3"))):
                    check_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag info {output_dir}"
                    if use_drive_ws:
                        check_cmd = f"source {ros2_setup} && source {drive_ws_setup} && source {ws_setup} && ros2 bag info {output_dir}"
                    check_result = subprocess.run(
                        check_cmd,
                        shell=True,
                        executable="/bin/bash",
                        capture_output=True,
                        text=True,
                        timeout=10,
                        env=env,
                    )
                    if check_result.returncode == 0:
                        duration_line = [line for line in check_result.stdout.split("\n") if "Duration:" in line]
                        if duration_line:
                            self.log(f"📊 {duration_line[0].strip()}")
                            try:
                                duration_str = duration_line[0].split(":")[1].strip()
                                duration_seconds = float(duration_str.replace("s", "").split()[0])
                                if duration_seconds >= 4.5:
                                    duration_ok = True
                                else:
                                    self.log(f"⚠️ Duration chỉ {duration_seconds:.2f}s (bag gốc có thể thiếu dữ liệu trong 5s đầu)")
                            except Exception:
                                pass

                if duration_ok:
                    self.cut_output_path = str(output_dir)
                    self.after(0, lambda: self._update_cut_status(True, str(output_dir)))
                else:
                    self.after(0, lambda: self._update_cut_status(False, "Duration < 5s hoặc không ghi được file"))

            except Exception as e:
                self.log(f"❌ Lỗi khi cắt bag: {e}")
                self.after(0, lambda: self._update_cut_status(False, str(e)))
            finally:
                # Dọn process nếu còn
                for p in [self.play_process, self.record_process]:
                    try:
                        if p and p.poll() is None:
                            p.terminate()
                            p.wait(timeout=2)
                    except Exception:
                        pass
                self.play_process = None
                self.record_process = None

        threading.Thread(target=worker, daemon=True).start()

    def _update_cut_status(self, success, info):
        self.is_cutting = False
        self.cut_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Trạng thái: Sẵn sàng", foreground="gray")
        if success:
            self.cut_status_label.config(text=f"Đã cắt: {info}", foreground="green")
            self.log(f"✅ Cắt thành công -> {info}")
        else:
            self.cut_status_label.config(text=f"Cắt thất bại: {info}", foreground="red")
            self.log(f"❌ Cắt thất bại: {info}")
            messagebox.showerror("Lỗi", f"Cắt bag thất bại: {info}")

    def stop_cut(self):
        """Dừng các process cắt nếu còn chạy"""
        for p in [self.play_process, self.record_process]:
            try:
                if p and p.poll() is None:
                    p.terminate()
                    p.wait(timeout=2)
            except Exception:
                pass
        self.play_process = None
        self.record_process = None
        self.is_cutting = False
        self.cut_btn.config(state=tk.NORMAL)
        self.status_label.config(text="Trạng thái: Sẵn sàng", foreground="gray")


