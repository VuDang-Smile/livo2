import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
import time
from pathlib import Path

class BagMappingInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Bag Mapping System")
        self.root.geometry("1150x850")
        
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.bag_path = None

        # --- Top Menu Bar ---
        # self.menu_bar = tk.Label(root, text="Bag Mapping | Data Processing & Upload", 
        #                          anchor="w", padx=15, relief=tk.RAISED, bd=1)
        # self.menu_bar.pack(side=tk.TOP, fill=tk.X)

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- Workspace ---
        self.workspace = tk.Frame(self.main_container)
        self.workspace.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 1. Card Control (Start Mapping)
        self.setup_control_card()

        # 2. Card Visualization (Preview / RViz)
        self.setup_preview_rviz_section()

        # 3. Card Server Upload (Ẩn cho đến khi mapping xong)
        self.setup_server_upload_section()

        # --- Log Section ---
        self.setup_log_section()

    def setup_control_card(self):
        card = tk.LabelFrame(self.workspace, text="Execution Control", padx=15, pady=10)
        card.pack(fill=tk.X, pady=(0, 10))
        
        # File Path
        tk.Label(card, text="Select Bag File:").pack(anchor="w")
        path_frame = tk.Frame(card)
        path_frame.pack(fill=tk.X, pady=5)
        self.bag_path_var = tk.StringVar()
        tk.Entry(path_frame, textvariable=self.bag_path_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(path_frame, text="Browse", command=self.browse_file).pack(side=tk.LEFT, padx=5)

        # Control Buttons
        btn_row = tk.Frame(card)
        btn_row.pack(fill=tk.X, pady=5)
        self.btn_start = tk.Button(btn_row, text="🚀 START MAPPING", width=20, command=self.start_mapping)
        self.btn_start.pack(side=tk.LEFT, padx=5)
        tk.Button(btn_row, text="■ STOP", width=10, command=self.stop_process).pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(btn_row, text="Status: Ready", font=("Arial", 9, "bold"))
        self.status_label.pack(side=tk.RIGHT, padx=10)

    def setup_preview_rviz_section(self):
        card = tk.LabelFrame(self.workspace, text="Visualization Preview", padx=15, pady=15)
        card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        tool_frame = tk.Frame(card)
        tool_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.rviz_var = tk.BooleanVar(value=False)
        tk.Checkbutton(tool_frame, text="Use RViz2 External View", 
                       variable=self.rviz_var, command=self.update_preview_mode).pack(side=tk.LEFT)
        
        self.btn_open_rviz = tk.Button(tool_frame, text="Launch RViz2", 
                                       state=tk.DISABLED, command=self.launch_rviz_cmd)
        self.btn_open_rviz.pack(side=tk.RIGHT)

        self.preview_container = tk.Frame(card, bg="#ffffff", relief=tk.SUNKEN, bd=1)
        self.preview_container.pack(fill=tk.BOTH, expand=True)
        self.preview_label = tk.Label(self.preview_container, text="INTERNAL VIEWPORT ACTIVE", 
                                     bg="white", font=("Arial", 10, "bold"))
        self.preview_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    def setup_server_upload_section(self):
        self.upload_card = tk.LabelFrame(self.workspace, text="Server Synchronization", padx=15, pady=10)
        self.upload_card.pack(fill=tk.X)
        
        # Ban đầu làm mờ card này
        self.btn_upload = tk.Button(self.upload_card, text="☁ UPLOAD TO SERVER", 
                                    state=tk.DISABLED, command=self.start_upload)
        self.btn_upload.pack(side=tk.LEFT, padx=5)

        self.progress = ttk.Progressbar(self.upload_card, orient=tk.HORIZONTAL, mode='determinate')
        self.progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.upload_stat = tk.Label(self.upload_card, text="Waiting...", font=("Arial", 8))
        self.upload_stat.pack(side=tk.RIGHT)

    def setup_log_section(self):
        log_container = tk.Frame(self.root, padx=5, pady=5)
        log_container.pack(side=tk.BOTTOM, fill=tk.X)
        header = tk.Frame(log_container)
        header.pack(fill=tk.X, pady=(0, 2))
        tk.Label(header, text="SYSTEM LOG", font=("Arial", 8, "bold")).pack(side=tk.LEFT)
        tk.Button(header, text="Clear Log", font=("Arial", 7), command=self.clear_log).pack(side=tk.RIGHT)

        self.log_panel = tk.Text(log_container, height=8, bg="white", fg="black", 
                                 font=("Consolas", 10), state=tk.DISABLED, bd=1, relief=tk.SUNKEN)
        self.log_panel.pack(fill=tk.X)

    # --- Logic ---
    def update_preview_mode(self):
        if self.rviz_var.get():
            self.preview_container.config(bg="#f0f0f0")
            self.preview_label.config(text="RViz2 REDIRECT ACTIVE", fg="blue", bg="#f0f0f0")
            self.btn_open_rviz.config(state=tk.NORMAL)
            self.add_log("CONFIG: Visualization mode set to RViz2.")
        else:
            self.preview_container.config(bg="white")
            self.preview_label.config(text="INTERNAL VIEWPORT ACTIVE", fg="black", bg="white")
            self.btn_open_rviz.config(state=tk.DISABLED)
            self.add_log("CONFIG: Visualization mode set to Internal.")

    def start_mapping(self):
        if not self.bag_path_var.get():
            messagebox.showwarning("Warning", "Please select a bag file first!")
            return
        
        self.add_log("MAPPING: Fast-Livo2 algorithm started...")
        self.status_label.config(text="Status: Mapping...")
        self.btn_start.config(state=tk.DISABLED)
        
        # Giả lập thời gian mapping hoàn tất sau 2 giây
        self.root.after(2000, self.on_mapping_complete)

    def on_mapping_complete(self):
        self.add_log("SUCCESS: Mapping complete. Ready for server upload.")
        self.status_label.config(text="Status: Map Created")
        self.btn_upload.config(state=tk.NORMAL)
        self.upload_stat.config(text="Ready to upload")

    def start_upload(self):
        self.btn_upload.config(state=tk.DISABLED)
        self.add_log("SERVER: Beginning data synchronization...")
        self.simulate_upload_progress(0)

    def simulate_upload_progress(self, val):
        if val <= 100:
            self.progress['value'] = val
            self.upload_stat.config(text=f"Uploading: {val}%")
            self.root.after(30, lambda: self.simulate_upload_progress(val + 2))
        else:
            self.add_log("SERVER: Upload successful. Data stored in cloud.")
            self.upload_stat.config(text="100% - Done")
            messagebox.showinfo("Success", "All map data has been uploaded to the server!")

    def browse_file(self):
        initial_dir = "/media/an/01DC80D9DB838380/"
        bag_path = filedialog.askdirectory(
            title="Chọn Bag Folder",
            initialdir=initial_dir
        )
        
        if bag_path:
            bag_path_obj = Path(bag_path)
            if bag_path_obj.exists():
                self.bag_path_var.set(str(bag_path_obj))
                self.bag_path = str(bag_path_obj)
                self.add_log(f"✅ Đã chọn bag: {bag_path_obj.name}")
            else:
                messagebox.showerror("Lỗi", f"Bag folder không tồn tại: {bag_path}")

    def launch_rviz_cmd(self):
        self.add_log("SYSTEM: Launching RViz2...")

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

    def stop_process(self):
        self.btn_start.config(state=tk.NORMAL)
        self.btn_upload.config(state=tk.DISABLED)
        self.progress['value'] = 0
        self.status_label.config(text="Status: Stopped")
        self.add_log("PROCESS: Mapping and Upload terminated.")

if __name__ == "__main__":
    root = tk.Tk()
    app = BagMappingInterface(root)
    root.mainloop()