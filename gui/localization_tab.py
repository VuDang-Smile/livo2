#!/usr/bin/env python3
"""
Localization Tab Module
Tab để thực hiện Localization bằng FAST-LIO-LOCALIZATION2
"""

import threading
import subprocess
from pathlib import Path
from datetime import datetime
import os
import signal
import sys
import time
import json
import shutil
import zipfile

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    sys.exit(1)

class LocalizationTab(ttk.Frame):
    """Tab cho Localization với FAST-LIO-LOCALIZATION2"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Paths
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.default_map_root = self.workspace_path / "src" / "FAST-LIVO2" / "Log" / "fastloc_map"
        self.backend_base_url = os.environ.get("LIVO_BACKEND_URL", "http://backend.lidar.tm")
        
        # Processes
        self.loc_process = None
        
        # State
        self.is_running = False
        self.map_root = str(self.default_map_root)
        self.use_rviz = True
        self.config_file = "mid360.yaml"
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Localization"""
        
        # Title
        title_label = ttk.Label(
            self,
            text="Localization với FAST-LIO-LOCALIZATION2",
            font=("Arial", 16, "bold")
        )
        title_label.pack(pady=10)
        
        # Frame điều khiển chính
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # --- Map Selection Frame ---
        map_frame = ttk.LabelFrame(main_frame, text="Cấu hình Bản đồ (Map)", padding="10")
        map_frame.pack(fill=tk.X, pady=5)
        
        map_select_frame = ttk.Frame(map_frame)
        map_select_frame.pack(fill=tk.X)
        
        self.map_path_var = tk.StringVar(value=str(self.default_map_root))
        map_entry = ttk.Entry(map_select_frame, textvariable=self.map_path_var, state="readonly")
        map_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        browse_btn = ttk.Button(
            map_select_frame,
            text="Chọn Thư mục Map...",
            command=self.browse_map_dir
        )
        browse_btn.pack(side=tk.RIGHT)
        
        # Download map từ backend
        self.download_btn = ttk.Button(
            map_frame,
            text="⬇️ Tải map từ backend",
            command=self.download_map_from_backend
        )
        self.download_btn.pack(anchor=tk.W, pady=5)
        
        # Map Info
        self.map_info_label = ttk.Label(
            map_frame,
            text="Đang kiểm tra bản đồ...",
            foreground="gray"
        )
        self.map_info_label.pack(pady=5, anchor=tk.W)
        self.update_map_info()

        # --- Options Frame ---
        options_frame = ttk.LabelFrame(main_frame, text="Tùy chọn", padding="10")
        options_frame.pack(fill=tk.X, pady=5)
        
        # Config Selection
        cfg_frame = ttk.Frame(options_frame)
        cfg_frame.pack(side=tk.LEFT, padx=5)
        ttk.Label(cfg_frame, text="Config:").pack(side=tk.LEFT, padx=5)
        
        self.config_var = tk.StringVar(value="mid360.yaml")
        config_dir = self.workspace_path / "src" / "fast_lio_localization" / "config"
        config_files = []
        if config_dir.exists():
            config_files = sorted([f.name for f in config_dir.glob("*.yaml") if f.is_file()])
        
        if not config_files:
            config_files = ["mid360.yaml"]
            
        config_combo = ttk.Combobox(
            cfg_frame,
            textvariable=self.config_var,
            values=config_files,
            state="readonly",
            width=20
        )
        config_combo.pack(side=tk.LEFT)
        
        # RViz Option
        self.rviz_var = tk.BooleanVar(value=True)
        rviz_check = ttk.Checkbutton(
            options_frame,
            text="Hiển thị RViz2",
            variable=self.rviz_var
        )
        rviz_check.pack(side=tk.LEFT, padx=20)

        # --- Control Frame ---
        control_frame = ttk.Frame(main_frame, padding="10")
        control_frame.pack(fill=tk.X, pady=10)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="🚀 Bắt đầu Localization",
            command=self.start_localization,
            style="Accent.TButton" if "Accent.TButton" in ttk.Style().theme_names() else "TButton"
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="⏹ Dừng",
            command=self.stop_localization,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Status
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Sẵn sàng",
            foreground="green"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # --- Log Frame ---
        log_frame = ttk.LabelFrame(main_frame, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=15,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        self.log("✅ Localization Tab đã sẵn sàng")

    def log(self, message):
        """Thêm log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, log_message)
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def browse_map_dir(self):
        """Chọn thư mục map"""
        initial_dir = self.default_map_root.parent if self.default_map_root.parent.exists() else Path.home()
        map_dir = filedialog.askdirectory(
            title="Chọn Thư mục Map (chứa pcd/ và pose.json)",
            initialdir=str(initial_dir)
        )
        
        if map_dir:
            self.map_path_var.set(map_dir)
            self.map_root = map_dir
            self.update_map_info()
            self.log(f"📂 Đã chọn map: {Path(map_dir).name}")

    def update_map_info(self):
        """Kiểm tra và hiển thị thông tin bản đồ"""
        map_path = Path(self.map_path_var.get())
        if not map_path.exists():
            self.map_info_label.config(text="❌ Thư mục không tồn tại", foreground="red")
            return

        pose_json = map_path / "pose.json"
        pcd_dir = map_path / "pcd"
        
        issues = []
        if not pose_json.exists():
            issues.append("thiếu pose.json")
        if not pcd_dir.exists() or not any(pcd_dir.glob("*.pcd")):
            issues.append("thiếu file pcd")
            
        if issues:
            self.map_info_label.config(text=f"⚠️ Bản đồ không hợp lệ: {', '.join(issues)}", foreground="orange")
        else:
            # Thử đếm số tile
            num_tiles = len(list(pcd_dir.glob("*.pcd")))
            self.map_info_label.config(text=f"✅ Bản đồ hợp lệ: {num_tiles} tiles được tìm thấy", foreground="green")

    def download_map_from_backend(self):
        """Luồng tải map zip từ backend, giải nén và chọn vào Localization."""
        if not REQUESTS_AVAILABLE:
            messagebox.showerror("Thiếu thư viện", "Thiếu requests. Vui lòng cài: pip install requests")
            return
        
        if not messagebox.askyesno("Xác nhận", "Tải map hiện tại từ backend và thay thế map cục bộ?"):
            return
        
        self.download_btn.config(state=tk.DISABLED)
        threading.Thread(target=self._download_map_worker, daemon=True).start()

    def _download_map_worker(self):
        """Worker tải map + giải nén (chạy nền)."""
        try:
            base_url = self.backend_base_url.rstrip("/")
            current_url = f"{base_url}/api/v1/maps/current"
            download_url = f"{base_url}/api/v1/maps/download"
            
            self.log("🌐 Đang lấy thông tin map hiện tại từ backend...")
            resp = requests.get(current_url, timeout=10)
            if resp.status_code != 200:
                self.log(f"❌ Không lấy được thông tin map (HTTP {resp.status_code})")
                return
            
            data = resp.json()
            upload_id = data.get("upload_id") or data.get("uploadId")
            filename = data.get("filename") or f"map_{upload_id or 'current'}.zip"
            
            dest_root = self.default_map_root.parent
            dest_root.mkdir(parents=True, exist_ok=True)
            dest_zip = dest_root / filename
            
            self.log(f"⬇️ Đang tải map: {filename}")
            with requests.get(download_url, stream=True, timeout=120) as r:
                if r.status_code != 200:
                    self.log(f"❌ Tải map thất bại (HTTP {r.status_code})")
                    return
                with open(dest_zip, "wb") as f:
                    for chunk in r.iter_content(chunk_size=8192):
                        if chunk:
                            f.write(chunk)
            
            self.log("📦 Đang giải nén map...")
            # Xóa map cũ để tránh lẫn file
            if self.default_map_root.exists():
                shutil.rmtree(self.default_map_root, ignore_errors=True)
            
            with zipfile.ZipFile(dest_zip, 'r') as zip_ref:
                zip_ref.extractall(dest_root)
            
            # Cập nhật UI và state
            self.map_root = str(self.default_map_root)
            self.after(0, lambda: self.map_path_var.set(str(self.default_map_root)))
            self.after(0, self.update_map_info)
            self.log("✅ Đã tải và giải nén map từ backend thành công")
            
        except requests.exceptions.RequestException as e:
            self.log(f"❌ Lỗi khi tải map: {e}")
        except Exception as e:
            self.log(f"❌ Lỗi giải nén/map: {e}")
        finally:
            self.after(0, lambda: self.download_btn.config(state=tk.NORMAL))

    def start_localization(self):
        """Khởi động localization node"""
        if self.is_running:
            return

        map_root = self.map_path_var.get()
        if not Path(map_root).exists() or not (Path(map_root) / "pose.json").exists():
            messagebox.showerror("Lỗi", "Thư mục map không hợp lệ hoặc thiếu pose.json")
            return

        # Script helper
        run_script = Path(__file__).parent.parent / "scripts" / "run_localization.sh"
        if not run_script.exists():
            messagebox.showerror("Lỗi", f"Không tìm thấy script chạy localization tại: {run_script}")
            return

        self.log("=" * 60)
        self.log("🚀 Đang khởi động Localization...")
        self.log(f"📁 Map: {map_root}")
        self.log(f"📋 Config: {self.config_var.get()}")
        
        try:
            rviz_arg = "True" if self.rviz_var.get() else "False"
            cfg_file = self.config_var.get()
            cmd = f"{run_script} {map_root} {rviz_arg} {cfg_file}"
            
            # Cập nhật UI
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            self.status_label.config(text="Trạng thái: 📡 Đang chạy", foreground="orange")
            
            # Start process
            self.loc_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_running = True
            threading.Thread(target=self.monitor_process, daemon=True).start()
            
        except Exception as e:
            self.log(f"❌ Lỗi khi khởi động: {e}")
            self.stop_localization()

    def monitor_process(self):
        """Theo dõi output của process localization"""
        proc = self.loc_process
        if not proc:
            return
            
        try:
            for line in iter(proc.stdout.readline, ''):
                if not line: break
                
                if "Failed to find match for field" in line:
                    continue
                    
                line_lower = line.lower()
                # Lọc bớt log, chỉ hiện log quan trọng
                if any(k in line_lower for keyword in ['error', 'warning', 'fail', 'success', 'init', 'global'] for k in [keyword]):
                    self.log(f"[LOC] {line.strip()}")
            
            proc.wait()
        except Exception as e:
            print(f"Lỗi trong monitor_process: {e}")
        finally:
            self.after(0, self.on_process_ended)

    def on_process_ended(self):
        """Xử lý khi process kết thúc"""
        if self.is_running:
            self.log("⏹ Localization process đã dừng.")
            self.stop_localization()

    def stop_localization(self):
        """Dừng localization"""
        if self.loc_process:
            try:
                if hasattr(os, 'setsid'):
                    os.killpg(os.getpgid(self.loc_process.pid), signal.SIGTERM)
                else:
                    self.loc_process.terminate()
                self.loc_process.wait(timeout=5)
            except:
                if self.loc_process:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(self.loc_process.pid), signal.SIGKILL)
                    else:
                        self.loc_process.kill()
            finally:
                self.loc_process = None
        
        self.is_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_label.config(text="Trạng thái: Đã dừng", foreground="red")

    def stop_all(self):
        """Được gọi từ Main GUI khi đóng app"""
        self.stop_localization()

