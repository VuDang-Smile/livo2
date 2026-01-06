import tkinter as tk
from tkinter import ttk
import threading
import time
from datetime import datetime

class LivoxApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Livox Control Panel")
        self.root.geometry("1000x800")
        
        self.is_recording = False
        self._setup_layout()

    def _setup_layout(self):
        # 1. Main Container (Sidebar + Workspace)
        self.main_container = tk.Frame(self.root)
        self.main_container.pack(side="top", fill="both", expand=True)

        # 2. Sidebar (Trái)
        self.sidebar = tk.LabelFrame(self.main_container, text="LIVOX DRIVER 2", padx=10, pady=10)
        self.sidebar.pack(side="left", fill="y", padx=5, pady=5)

        tk.Checkbutton(self.sidebar, text="Enable Converter").pack(anchor="w")
        
        btn_driver_frame = tk.Frame(self.sidebar)
        btn_driver_frame.pack(fill="x", pady=10)
        tk.Button(btn_driver_frame, text="Start", width=10).pack(side="left", padx=2)
        tk.Button(btn_driver_frame, text="Stop", width=10).pack(side="left", padx=2)

        tk.Label(self.sidebar, text="● Lidar Topic: Active", fg="green").pack(anchor="w", pady=5)
        tk.Label(self.sidebar, text="● IMU Topic: Active", fg="green").pack(anchor="w")

        # 3. Workspace (Phải)
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side="right", fill="both", expand=True, padx=5, pady=5)

        # --- Card 1: Configuration ---
        self.conf_frame = tk.LabelFrame(self.workspace, text="RECORDING CONFIGURATION", padx=10, pady=10)
        self.conf_frame.pack(fill="x", pady=5)
        
        tk.Label(self.conf_frame, text="Storage Directory:").pack(anchor="w")
        dir_frame = tk.Frame(self.conf_frame)
        dir_frame.pack(fill="x")
        self.ent_dir = tk.Entry(dir_frame)
        self.ent_dir.insert(0, "/home/khanhbv/Desktop/recordings")
        self.ent_dir.pack(side="left", fill="x", expand=True, padx=(0, 5))
        tk.Button(dir_frame, text="Browse").pack(side="right")

        # tk.Label(self.conf_frame, text="Max File Size (GB):").pack(anchor="w", pady=(5, 0))
        # tk.Entry(self.conf_frame, width=10).pack(anchor="w")

        # # --- Card 2: Topic Selection ---
        # self.topic_frame = tk.LabelFrame(self.workspace, text="TOPIC SELECTION", padx=10, pady=10)
        # self.topic_frame.pack(fill="both", expand=True, pady=5)

        # columns = ("record", "topic", "type", "status")
        # self.tree = ttk.Treeview(self.topic_frame, columns=columns, show="headings", height=5)
        # for col in columns: self.tree.heading(col, text=col.capitalize())
        # self.tree.insert("", "end", values=("X", "/livox/lidar", "CustomMsg", "Ready"))
        # self.tree.insert("", "end", values=("X", "/livox/imu", "Imu", "Ready"))
        # self.tree.pack(fill="both", expand=True)

        # --- Card 3: Control Area (Record & Upload) ---
        self.ctrl_frame = tk.Frame(self.workspace, relief="groove", borderwidth=2, padx=10, pady=10)
        self.ctrl_frame.pack(fill="x", pady=5)

        self.btn_record = tk.Button(self.ctrl_frame, text="● START RECORDING", 
                                    font=("Arial", 10, "bold"), fg="red", command=self.toggle_recording)
        self.btn_record.pack(side="left", padx=10)

        self.btn_upload = tk.Button(self.ctrl_frame, text="↑ UPLOAD TO SERVER", command=self.start_upload_thread)
        
        self.lbl_status = tk.Label(self.ctrl_frame, text="Status: Ready")
        self.lbl_status.pack(side="left", padx=20)

        self.lbl_percent = tk.Label(self.ctrl_frame, text="0%")

        self.progress = ttk.Progressbar(self.workspace, orient="horizontal", mode="determinate")

       # 4. Log Panel (Dưới cùng)
        self.log_header_frame = tk.Frame(self.root)
        self.log_header_frame.pack(side="top", fill="x", padx=10)
        
        tk.Label(self.log_header_frame, text="System Logs", font=("Arial", 9, "bold")).pack(side="left")
        
        self.btn_clear_log = tk.Button(self.log_header_frame, text="Clear Log", 
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

    def add_log(self, msg):
        self.log_text.config(state="normal")
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {msg}\n")
        self.log_text.config(state="disabled")
        self.log_text.see("end")

    def clear_logs(self):
        """Xóa toàn bộ nội dung trong Log Text"""
        self.log_text.config(state="normal")
        self.log_text.delete('1.0', tk.END)
        self.log_text.config(state="disabled")
        self.add_log("Log cleared.")

    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.btn_record.config(text="■ STOP RECORDING")
            self.lbl_status.config(text="Status: Recording...", fg="red")
            self.btn_upload.pack_forget()
            self.add_log("Recording started...")
        else:
            self.is_recording = False
            self.btn_record.config(text="● START RECORDING")
            self.lbl_status.config(text="Status: File Saved", fg="green")
            self.btn_upload.pack(side="left", padx=10)
            self.add_log("Recording saved.")

    def start_upload_thread(self):
        self.btn_upload.config(state="disabled")
        self.progress.pack(fill="x", padx=20, pady=5)
        self.lbl_percent.pack(side="right", padx=20)
        threading.Thread(target=self.simulate_upload, daemon=True).start()

    def simulate_upload(self):
        self.add_log("Starting upload to server...")
        for i in range(101):
            time.sleep(0.04)
            self.progress['value'] = i
            self.lbl_percent.config(text=f"Uploading: {i}%")
        
        self.add_log("SUCCESS: Upload complete.")
        self.btn_upload.config(state="normal")
        time.sleep(1)
        self.progress.pack_forget()
        self.lbl_percent.pack_forget()

if __name__ == "__main__":
    root = tk.Tk()
    app = LivoxApp(root)
    root.mainloop()