import tkinter as tk
from datetime import datetime

class WorkerInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Worker Interface | System Monitoring")
        self.root.geometry("1100x750")
        
        self.is_running = False

        # --- Top Menu Bar ---
        # self.menu_bar = tk.Label(root, text="Worker Interface | System Monitoring", 
        #                          anchor="w", padx=10, relief=tk.RAISED, bd=1)
        # self.menu_bar.pack(side=tk.TOP, fill=tk.X)

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Sidebar ---
        self.sidebar = tk.Frame(self.main_container, width=320, relief=tk.SUNKEN, bd=1)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sidebar.pack_propagate(False)

        # Các thành phần Sidebar
        self.setup_movement_control()
        self.setup_options()
        self.setup_vehicle_info()
        self.setup_sensor_health()

        # --- Workspace (Map Area) ---
        self.workspace = tk.Frame(self.main_container, relief=tk.SUNKEN, bd=1)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.setup_workspace()

        # --- Log Section ---
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
        frame = tk.LabelFrame(self.sidebar, text="System Options", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)

        self.rviz_var = tk.BooleanVar(value=False)
        # Sử dụng Checkbutton mặc định
        self.chk_rviz = tk.Checkbutton(frame, text="Show RViz2 Interface", 
                                       variable=self.rviz_var, 
                                       command=self.on_rviz_toggle)
        self.chk_rviz.pack(anchor="w")

    def setup_vehicle_info(self):
        frame = tk.LabelFrame(self.sidebar, text="Vehicle Information", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)

        info = [
            ("Device ID:", "cc28aa1238c5"),
            ("Vehicle ID:", "cc28aa1238c5"),
            ("Category:", "UGV / Robot"),
            ("License Plate:", "VN-2026-X1")
        ]

        for label, value in info:
            row = tk.Frame(frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=label, fg="grey", font=("Arial", 8)).pack(side=tk.LEFT)
            tk.Label(row, text=value, font=("Arial", 9, "bold")).pack(side=tk.RIGHT)

    def setup_sensor_health(self):
        frame = tk.LabelFrame(self.sidebar, text="Sensor Health", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)

        tk.Label(frame, text="LiDAR: Healthy").pack(anchor="w")
        tk.Label(frame, text="IMU: Active").pack(anchor="w")

    def setup_workspace(self):
        self.canvas = tk.Canvas(self.workspace, bg="white", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Grid
        for i in range(0, 1200, 30):
            self.canvas.create_line(i, 0, i, 1000, fill="#f0f0f0")
            self.canvas.create_line(0, i, 1200, i, fill="#f0f0f0")

        # Telemetry Overlay
        # self.telemetry_frame = tk.Frame(self.canvas, relief=tk.SOLID, bd=1, padx=5, pady=5)
        # self.canvas.create_window(10, 10, window=self.telemetry_frame, anchor="nw")
        
        # tk.Label(self.telemetry_frame, text="TELEMETRY DATA", font=("Arial", 8, "bold")).pack()
        # self.vel_label = tk.Label(self.telemetry_frame, text="VELOCITY: 0.00 m/s")
        # self.vel_label.pack(anchor="w")
        # tk.Label(self.telemetry_frame, text="HEADING: 184.2°").pack(anchor="w")
        # self.lidar_stat_label = tk.Label(self.telemetry_frame, text="LIDAR: Standby")
        # self.lidar_stat_label.pack(anchor="w")

        # self.canvas.create_text(20, 480, text="LIVOX REAL-TIME VIEWPORT [READY]", 
        #                         anchor="sw", font=("Arial", 10, "bold"))

    def setup_log_area(self):
        log_container = tk.Frame(self.root, padx=5, pady=5)
        log_container.pack(side=tk.BOTTOM, fill=tk.X)

        log_header = tk.Frame(log_container)
        log_header.pack(fill=tk.X, pady=(0, 2))
        
        tk.Label(log_header, text="SYSTEM LOGS", font=("Arial", 8, "bold")).pack(side=tk.LEFT)
        
        # SỬA LỖI TẠI ĐÂY: Bỏ tham số size=10, thay bằng font
        btn_clear = tk.Button(log_header, text="Clear Log", 
                              command=self.clear_log, 
                              font=("Arial", 7))
        btn_clear.pack(side=tk.RIGHT)

        self.log_panel = tk.Text(log_container, height=8, state='disabled', 
                                 font=("Consolas", 10), relief=tk.SUNKEN, bd=1)
        self.log_panel.pack(fill=tk.X)
        
        self.add_log("INFO: Connection established with Device cc28aa12.")
        self.add_log("INFO: LiDAR sensors calibrated.")
        self.add_log("SUCCESS: System ready for movement.")

    def on_rviz_toggle(self):
        status = "ENABLED" if self.rviz_var.get() else "DISABLED"
        self.add_log(f"CONFIG: RViz2 visualization {status}.")

    def clear_log(self):
        self.log_panel.config(state='normal')
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state='disabled')

    def add_log(self, message):
        self.log_panel.config(state='normal')
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_panel.insert(tk.END, f"[{time_str}] {message}\n")
        self.log_panel.see(tk.END)
        self.log_panel.config(state='disabled')

    def toggle_movement(self):
        if not self.is_running:
            self.is_running = True
            self.btn_start.config(text="■ STOP MOVEMENT")
            self.status_text.config(text="System: Moving & Scanning...")
            self.vel_label.config(text="VELOCITY: 0.85 m/s")
            self.lidar_stat_label.config(text="LIDAR: Active (10Hz)")
            self.add_log("COMMAND: Start movement initiated.")
            if self.rviz_var.get():
                self.add_log("PROCESS: Executing 'ros2 launch rviz2'...")
        else:
            self.stop_system()

    def stop_system(self):
        self.is_running = False
        self.btn_start.config(text="▶ START MOVING")
        self.status_text.config(text="System: Ready to engage")
        self.vel_label.config(text="VELOCITY: 0.00 m/s")
        self.lidar_stat_label.config(text="LIDAR: Standby")
        self.add_log("COMMAND: Stop movement requested.")

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkerInterface(root)
    root.mainloop()