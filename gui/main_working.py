import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime
import uuid
import subprocess
import os
import sys
import requests

# Thêm project root vào sys.path để có thể import các module
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from languages.translate_engine import translator
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS


class WorkerInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Worker Interface | System Monitoring")
        self.root.geometry("1100x700")
        
        self.is_running = False

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(fill=tk.BOTH, expand=True)

        # --- PHẦN 1: SIDEBAR (Bên trái) ---
        self.sidebar = tk.Frame(self.main_container, width=320, relief=tk.SUNKEN, bd=1)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sidebar.pack_propagate(False) # Giữ cố định độ rộng cho Sidebar

        # --- PHẦN 2: SYSTEM LOG (Bên phải - Chiếm trọn phần còn lại) ---
        self.log_container = tk.Frame(self.main_container, padx=5, pady=5)
        self.log_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Cài đặt các thành phần bên trong Sidebar
        self.setup_movement_control()
        self.setup_options()
        self.setup_vehicle_info()

        # Cài đặt phần Log bên phải
        self.setup_log_area()

    def setup_movement_control(self):
        frame = tk.LabelFrame(self.sidebar, text="Movement Control", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)

        self.btn_start = tk.Button(frame, text="▶ START MOVING", command=self.toggle_movement)
        self.btn_start.pack(fill=tk.X, pady=2)

        self.btn_stop = tk.Button(frame, text="STOP", command=self.stop_system)
        self.btn_stop.pack(fill=tk.X, pady=2)

        self.status_text = tk.Label(frame, text="System: Ready to engage", font=("Arial", 9, "italic"))
        self.status_text.pack(pady=5)

    def setup_options(self):
        frame = tk.LabelFrame(self.sidebar, text="Preview", padx=5, pady=5)
        frame.pack(fill=tk.X, pady=5, padx=5)

        # self.rviz_var = tk.BooleanVar(value=False)
        # self.chk_rviz = tk.Checkbutton(frame, text="Show RViz2 Interface", 
        #                                variable=self.rviz_var, 
        #                                command=self.on_rviz_toggle)
        # self.chk_rviz.pack(anchor="w")

        self.btn_rviz = tk.Button(
            frame, 
            text="📊 " + translator.get("button.launch_rviz2"), 
            command=self.open_rviz,
            width=10
        )
        self.btn_rviz.pack(side="right", pady=10)  
    
    def open_rviz(self):
        """Mở RViz2 trực tiếp bằng subprocess"""
        rviz_config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.rviz")
        self.log(f"🚀 Đang khởi động RViz2... {rviz_config_file}")
        
        # Kiểm tra xem file có tồn tại không để tránh lỗi im lặng
        if not os.path.exists(rviz_config_file):
            self.log(f"⚠️ Cảnh báo: Không tìm thấy file cấu hình tại {rviz_config_file}. Sẽ mở RViz mặc định.")
            cmd = ['rviz2']
        else:
            cmd = ['rviz2', '-d', rviz_config_file]

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Ẩn log của RViz để đỡ rối terminal
                stderr=subprocess.STDOUT
            )
            self.log("✅ RViz2 đã được mở thành công.")
            
        except Exception as e:
            self.log(f"❌ Lỗi khi thực thi lệnh rviz2: {e}")

    def setup_vehicle_info(self):
        device_id = self.get_mac_address()
        vehicle_id = device_id.lower().replace(':', '')[:12]
        print("Vehicle ID:", vehicle_id)

        frame = tk.LabelFrame(self.sidebar, text="Vehicle Information", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        url = f'{VEHICLE_ENDPOINT}/{vehicle_id}'
        headers = {
            'accept': 'application/json'
        }
        response = requests.get(url, headers=headers, timeout=5)
        data = response.json()

        info = [
            ("Device ID:", device_id),
            ("Vehicle ID:", vehicle_id),
            ("name:", data.get("name", "N/A")),
            ("type:", data.get("type", "N/A")),
            ("status:", data.get("status", "N/A")),
        ]

        for label, value in info:
            row = tk.Frame(frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, fg="grey", font=("Arial", 8)).pack(side=tk.LEFT)
            tk.Label(row, text=value, font=("Arial", 9, "bold")).pack(side=tk.RIGHT)

    def setup_log_area(self):
        # Tiêu đề của Log
        log_header = tk.Frame(self.log_container)
        log_header.pack(fill=tk.X, pady=(0, 5))
        
        tk.Label(log_header, text="SYSTEM MONITORING LOGS", font=("Arial", 9, "bold")).pack(side=tk.LEFT)
        
        btn_clear = tk.Button(log_header, text="Clear Log", 
                              command=self.clear_log, 
                              font=("Arial", 7))
        btn_clear.pack(side=tk.RIGHT)

        # Log Panel chiếm toàn bộ không gian còn lại
        # Sử dụng scrolledtext để tự động có thanh cuộn nếu log quá dài
        self.log_panel = scrolledtext.ScrolledText(self.log_container, state='disabled', 
                                                   font=("Consolas", 10), relief=tk.SUNKEN, bd=1)
        self.log_panel.pack(fill=tk.BOTH, expand=True)
        
        self.log("INFO: Connection established with Device.")
        self.log("INFO: LiDAR sensors calibrated.")
        self.log("SUCCESS: System ready for movement.")

    def get_mac_address(self):
        try:
            mac = uuid.getnode()
            return ':'.join(['{:02x}'.format((mac >> elements) & 0xff) for elements in range(40, -1, -8)])
        except: return "unknown"

    def log(self, message):
        self.log_panel.config(state='normal')
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_panel.insert(tk.END, f"[{time_str}] {message}\n")
        self.log_panel.see(tk.END)
        self.log_panel.config(state='disabled')

    def clear_log(self):
        self.log_panel.config(state='normal')
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state='disabled')

    def on_rviz_toggle(self):
        status = "ENABLED" if self.rviz_var.get() else "DISABLED"
        self.log(f"CONFIG: RViz2 visualization {status}.")

    def toggle_movement(self):
        if not self.is_running:
            self.is_running = True
            self.btn_start.config(text="■ STOP MOVEMENT")
            self.status_text.config(text="System: Moving & Scanning...")
            self.log("COMMAND: Start movement initiated.")
        else:
            self.stop_system()

    def stop_system(self):
        self.is_running = False
        self.btn_start.config(text="▶ START MOVING")
        self.status_text.config(text="System: Ready to engage")
        self.log("COMMAND: Stop movement requested.")

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkerInterface(root)
    root.mainloop()