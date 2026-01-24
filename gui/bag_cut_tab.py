#!/usr/bin/env python3
"""
Bag Cut Tab Module
Tạo tab để cắt 5 giây đầu của một rosbag mà không ảnh hưởng Replay cũ.
"""

import subprocess
import threading
import os
import shutil
from pathlib import Path
from datetime import datetime
from typing import Callable, Dict, Optional, Tuple

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


def _parse_metadata_yaml(bag_path: Path) -> Optional[Dict]:
    """Đọc metadata.yaml để lấy storage_id, start_time_ns, duration_ns và topics."""
    metadata_file = bag_path / "metadata.yaml"
    if not metadata_file.exists():
        return None
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(metadata_file.read_text())
        info = data.get("rosbag2_bagfile_information", {}) or {}
        topics_raw = info.get("topics_with_message_count", []) or []
        topics = []
        for t in topics_raw:
            meta = (t or {}).get("topic_metadata", {}) or {}
            topics.append(
                {
                    "name": meta.get("name"),
                    "type": meta.get("type"),
                    "serialization_format": meta.get("serialization_format") or "cdr",
                    "offered_qos_profiles": meta.get("offered_qos_profiles") or "",
                }
            )
        return {
            "storage_id": info.get("storage_identifier") or "sqlite3",
            "start_time_ns": (info.get("starting_time") or {}).get("nanoseconds_since_epoch"),
            "duration_ns": (info.get("duration") or {}).get("nanoseconds"),
            "topics": topics,
        }
    except Exception:
        return None


def cut_bag_5s_data(
    bag_path: Path,
    workspace_path: Path,
    drive_ws_path: Path,
    logger: Optional[Callable[[str], None]] = None,
    duration_seconds: float = 5.0,
) -> Tuple[bool, Optional[str], str]:
    """
    Cắt chính xác 5s dữ liệu dựa trên timestamp messages bằng rosbag2_py.

    Returns:
        (ok, output_dir_str, message)
    """

    def log(msg: str):
        if logger:
            logger(msg)

    if not bag_path.exists():
        return False, None, f"Bag folder không tồn tại: {bag_path}"

    meta = _parse_metadata_yaml(bag_path)
    if not meta:
        return False, None, "Không đọc được metadata.yaml để lấy thời gian/metadata"

    start_ns = meta.get("start_time_ns")
    duration_ns = meta.get("duration_ns")
    storage_id = meta.get("storage_id") or "sqlite3"
    topics = meta.get("topics") or []

    if start_ns is None or duration_ns is None:
        return False, None, "Metadata thiếu start_time hoặc duration"

    if duration_ns < duration_seconds * 1e9:
        return False, None, f"Bag chỉ có {duration_ns/1e9:.2f}s (< {duration_seconds}s)"

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = bag_path.parent / f"{bag_path.name}_cut_{int(duration_seconds)}s_{timestamp}"

    try:
        import rosbag2_py  # type: ignore
    except Exception as exc:
        return False, None, f"Thiếu rosbag2_py, không thể cắt bằng timestamp: {exc}"

    try:
        if not topics:
            return False, None, "Metadata không chứa danh sách topics, không thể tạo topic"

        # Dọn output cũ nếu có
        if output_dir.exists():
            shutil.rmtree(output_dir)

        converter_options = rosbag2_py.ConverterOptions("", "")
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
            converter_options,
        )

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(output_dir), storage_id=storage_id),
            converter_options,
        )

        for idx, t in enumerate(topics):
            writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=idx,
                    name=t.get("name") or "",
                    type=t.get("type") or "",
                    serialization_format=t.get("serialization_format") or "cdr",
                    offered_qos_profiles=[],
                    type_description_hash="",
                )
            )

        start_cut_ns = None
        end_cut_ns = None
        last_ns = None
        duration_limit = int(duration_seconds * 1e9)

        while reader.has_next():
            topic, data, ts = reader.read_next()
            if start_cut_ns is None:
                start_cut_ns = ts
                end_cut_ns = start_cut_ns + duration_limit
            if end_cut_ns is not None and ts > end_cut_ns:
                break
            writer.write(topic, data, ts)
            last_ns = ts

        if start_cut_ns is None or last_ns is None:
            return False, None, "Không đọc được message nào trong bag"

        actual_duration = (last_ns - start_cut_ns) / 1e9
        if actual_duration < duration_seconds * 0.9:
            return False, None, f"Duration sau cắt chỉ {actual_duration:.2f}s"

        # Kiểm tra lại bằng ros2 bag info (không bắt buộc, để log)
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        ws_setup = workspace_path / "install" / "setup.sh"
        drive_ws_setup = drive_ws_path / "install" / "setup.sh"
        use_drive_ws = drive_ws_setup.exists()
        check_cmd = f"source {ros2_setup} && "
        if use_drive_ws:
            check_cmd += f"source {drive_ws_setup} && "
        check_cmd += f"source {ws_setup} && ros2 bag info {output_dir}"
        try:
            info_res = subprocess.run(
                check_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=10,
            )
            if info_res.returncode == 0:
                log(info_res.stdout.strip())
        except Exception:
            pass

        return True, str(output_dir), f"Đã cắt {actual_duration:.2f}s dữ liệu"
    except Exception as exc:
        return False, None, f"Lỗi khi cắt bag bằng rosbag2_py: {exc}"


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

        self.is_cutting = True
        self.cut_output_path = None
        self.cut_status_label.config(text="Đang cắt...", foreground="orange")
        self.status_label.config(text="Trạng thái: Đang cắt...", foreground="orange")
        self.cut_btn.config(state=tk.DISABLED)
        self.log("🔧 Đang cắt 5s dữ liệu (dựa trên timestamp) ...")

        def worker():
            try:
                ok, out_path, msg = cut_bag_5s_data(
                    bag_path=bag_path_obj,
                    workspace_path=self.workspace_path,
                    drive_ws_path=self.drive_ws_path,
                    logger=self.log,
                    duration_seconds=5.0,
                )
                if ok and out_path:
                    self.cut_output_path = out_path
                    self.after(0, lambda: self._update_cut_status(True, out_path))
                else:
                    self.after(0, lambda: self._update_cut_status(False, msg))
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


