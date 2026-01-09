import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from datetime import datetime
import time
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "languages"))
from translate_engine import Translator

class BagMappingInterface:
    def __init__(self, root):
        self.root = root
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.bag_path = None
        self.config_path = None
        
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        self.setup_language_button()
        self.update_ui_texts()
        
        self.root.geometry("1150x850")

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
        
        # Set default config
        self.set_default_config()
    
    def setup_language_button(self):
        lang_frame = tk.Frame(self.root)
        lang_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button = tk.Button(
            lang_frame,
            text=f"🌐 {lang_names.get(self.current_lang, 'English')}",
            font=("Arial", 9, "bold"),
            fg="#0066cc",
            bg="#e8e8e8",
            activebackground="#d0d0d0",
            activeforeground="#0066cc",
            relief=tk.RAISED,
            bd=1,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.toggle_language
        )
        self.lang_button.pack(side=tk.RIGHT)
    
    def toggle_language(self):
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        self.root.title(self.translator.get('title.bag_mapping_system', 'Bag Mapping System'))
        
        if hasattr(self, 'control_card'):
            self.control_card.config(text=self.translator.get('label.execution_control', 'Execution Control'))
        
        if hasattr(self, 'select_bag_label'):
            self.select_bag_label.config(text=self.translator.get('label.select_bag_file', 'Select Bag File:'))
        
        if hasattr(self, 'browse_btn'):
            self.browse_btn.config(text=self.translator.get('button.browse', 'Browse'))
        
        if hasattr(self, 'btn_stop'):
            self.btn_stop.config(text=self.translator.get('button.stop_process', '■ STOP'))
        
        if hasattr(self, 'status_label'):
            current_status = self.status_label.cget('text')
            if 'Mapping' in current_status:
                self.status_label.config(text=self.translator.get('label.status_mapping', 'Status: Mapping...'))
            elif 'Map Created' in current_status:
                self.status_label.config(text=self.translator.get('label.status_map_created', 'Status: Map Created'))
            elif 'Stopped' in current_status:
                self.status_label.config(text=self.translator.get('label.status_stopped', 'Status: Stopped'))
            else:
                self.status_label.config(text=self.translator.get('label.status_ready', 'Status: Ready'))
        
        if hasattr(self, 'btn_start'):
            self.btn_start.config(text=self.translator.get('button.start_mapping', '🚀 START MAPPING'))
        
        if hasattr(self, 'preview_card'):
            self.preview_card.config(text=self.translator.get('label.visualization_preview', 'Visualization Preview'))
        
        if hasattr(self, 'rviz_check'):
            self.rviz_check.config(text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'))
        
        if hasattr(self, 'btn_open_rviz'):
            self.btn_open_rviz.config(text=self.translator.get('button.launch_rviz2', 'Launch RViz2'))
        
        if hasattr(self, 'preview_label'):
            current_text = self.preview_label.cget('text')
            if 'RViz2' in current_text:
                self.preview_label.config(text=self.translator.get('label.rviz2_redirect_active', 'RViz2 REDIRECT ACTIVE'))
            else:
                self.preview_label.config(text=self.translator.get('label.internal_viewport_active', 'INTERNAL VIEWPORT ACTIVE'))
        
        if hasattr(self, 'upload_card'):
            self.upload_card.config(text=self.translator.get('label.server_synchronization', 'Server Synchronization'))
        
        if hasattr(self, 'btn_upload'):
            self.btn_upload.config(text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'))
        
        if hasattr(self, 'upload_stat'):
            current_text = self.upload_stat.cget('text')
            if 'Ready' in current_text:
                self.upload_stat.config(text=self.translator.get('label.ready_to_upload', 'Ready to upload'))
            elif 'Done' in current_text or '100%' in current_text:
                self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            else:
                self.upload_stat.config(text=self.translator.get('label.waiting', 'Waiting...'))
        
        if hasattr(self, 'system_log_label'):
            self.system_log_label.config(text=self.translator.get('label.system_log', 'SYSTEM LOG'))
        
        if hasattr(self, 'clear_log_btn'):
            self.clear_log_btn.config(text=self.translator.get('button.clear_log', 'Clear Log'))

    def setup_control_card(self):
        self.control_card = tk.LabelFrame(self.workspace, text=self.translator.get('label.execution_control', 'Execution Control'), padx=15, pady=10)
        self.control_card.pack(fill=tk.X, pady=(0, 10))
        
        # File Path
        self.select_bag_label = tk.Label(self.control_card, text=self.translator.get('label.select_bag_file', 'Select Bag File:'))
        self.select_bag_label.pack(anchor="w")
        path_frame = tk.Frame(self.control_card)
        path_frame.pack(fill=tk.X, pady=5)
        self.bag_path_var = tk.StringVar()
        tk.Entry(path_frame, textvariable=self.bag_path_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.browse_btn = tk.Button(path_frame, text=self.translator.get('button.browse', 'Browse'), command=self.browse_file)
        self.browse_btn.pack(side=tk.LEFT, padx=5)

        # Control Buttons
        btn_row = tk.Frame(self.control_card)
        btn_row.pack(fill=tk.X, pady=5)
        self.btn_start = tk.Button(btn_row, text=self.translator.get('button.start_mapping', '🚀 START MAPPING'), width=20, command=self.start_mapping)
        self.btn_start.pack(side=tk.LEFT, padx=5)
        self.btn_stop = tk.Button(btn_row, text=self.translator.get('button.stop_process', '■ STOP'), width=10, command=self.stop_process)
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(btn_row, text=self.translator.get('label.status_ready', 'Status: Ready'), font=("Arial", 9, "bold"))
        self.status_label.pack(side=tk.RIGHT, padx=10)

    def setup_preview_rviz_section(self):
        self.preview_card = tk.LabelFrame(self.workspace, text=self.translator.get('label.visualization_preview', 'Visualization Preview'), padx=15, pady=15)
        self.preview_card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        tool_frame = tk.Frame(self.preview_card)
        tool_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.rviz_var = tk.BooleanVar(value=False)
        self.rviz_check = tk.Checkbutton(tool_frame, text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'), 
                       variable=self.rviz_var, command=self.update_preview_mode)
        self.rviz_check.pack(side=tk.LEFT)
        
        self.btn_open_rviz = tk.Button(tool_frame, text=self.translator.get('button.launch_rviz2', 'Launch RViz2'), 
                                       state=tk.DISABLED, command=self.launch_rviz_cmd)
        self.btn_open_rviz.pack(side=tk.RIGHT)

        self.preview_container = tk.Frame(self.preview_card, bg="#ffffff", relief=tk.SUNKEN, bd=1)
        self.preview_container.pack(fill=tk.BOTH, expand=True)
        self.preview_label = tk.Label(self.preview_container, text=self.translator.get('label.internal_viewport_active', 'INTERNAL VIEWPORT ACTIVE'), 
                                     bg="white", font=("Arial", 10, "bold"))
        self.preview_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    def setup_server_upload_section(self):
        self.upload_card = tk.LabelFrame(self.workspace, text=self.translator.get('label.server_synchronization', 'Server Synchronization'), padx=15, pady=10)
        self.upload_card.pack(fill=tk.X)
        
        self.btn_upload = tk.Button(self.upload_card, text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'), 
                                    state=tk.DISABLED, command=self.start_upload)
        self.btn_upload.pack(side=tk.LEFT, padx=5)

        self.progress = ttk.Progressbar(self.upload_card, orient=tk.HORIZONTAL, mode='determinate')
        self.progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.upload_stat = tk.Label(self.upload_card, text=self.translator.get('label.waiting', 'Waiting...'), font=("Arial", 8))
        self.upload_stat.pack(side=tk.RIGHT)

    def setup_log_section(self):
        log_container = tk.Frame(self.root, padx=5, pady=5)
        log_container.pack(side=tk.BOTTOM, fill=tk.X)
        header = tk.Frame(log_container)
        header.pack(fill=tk.X, pady=(0, 2))
        self.system_log_label = tk.Label(header, text=self.translator.get('label.system_log', 'SYSTEM LOG'), font=("Arial", 8, "bold"))
        self.system_log_label.pack(side=tk.LEFT)
        self.clear_log_btn = tk.Button(header, text=self.translator.get('button.clear_log', 'Clear Log'), font=("Arial", 7), command=self.clear_log)
        self.clear_log_btn.pack(side=tk.RIGHT)

        self.log_panel = tk.Text(log_container, height=8, bg="white", fg="black", 
                                 font=("Consolas", 10), state=tk.DISABLED, bd=1, relief=tk.SUNKEN)
        self.log_panel.pack(fill=tk.X)

    # --- Logic ---
    def update_preview_mode(self):
        if self.rviz_var.get():
            self.preview_container.config(bg="#f0f0f0")
            self.preview_label.config(text=self.translator.get('label.rviz2_redirect_active', 'RViz2 REDIRECT ACTIVE'), fg="blue", bg="#f0f0f0")
            self.btn_open_rviz.config(state=tk.NORMAL)
            self.add_log(self.translator.get('log.config_rviz2_mode', 'CONFIG: Visualization mode set to RViz2.'))
        else:
            self.preview_container.config(bg="white")
            self.preview_label.config(text=self.translator.get('label.internal_viewport_active', 'INTERNAL VIEWPORT ACTIVE'), fg="black", bg="white")
            self.btn_open_rviz.config(state=tk.DISABLED)
            self.add_log(self.translator.get('log.config_internal_mode', 'CONFIG: Visualization mode set to Internal.'))

    def start_mapping(self):
        if not self.bag_path_var.get():
            messagebox.showwarning(
                self.translator.get('dialog.warning', 'Warning'),
                self.translator.get('message.select_bag_file_first', 'Please select a bag file first!')
            )
            return
        
        self.add_log(self.translator.get('log.mapping_started', 'MAPPING: Fast-Livo2 algorithm started...'))
        self.status_label.config(text=self.translator.get('label.status_mapping', 'Status: Mapping...'))
        self.btn_start.config(state=tk.DISABLED)
        
        self.root.after(2000, self.on_mapping_complete)

    def on_mapping_complete(self):
        self.add_log(self.translator.get('log.mapping_complete', 'SUCCESS: Mapping complete. Ready for server upload.'))
        self.status_label.config(text=self.translator.get('label.status_map_created', 'Status: Map Created'))
        self.btn_upload.config(state=tk.NORMAL)
        self.upload_stat.config(text=self.translator.get('label.ready_to_upload', 'Ready to upload'))

    def start_upload(self):
        self.btn_upload.config(state=tk.DISABLED)
        self.add_log(self.translator.get('log.server_sync_started', 'SERVER: Beginning data synchronization...'))
        self.simulate_upload_progress(0)

    def simulate_upload_progress(self, val):
        if val <= 100:
            self.progress['value'] = val
            upload_text = self.translator.get('label.uploading', 'Uploading: {val}%').replace('{val}', str(val))
            self.upload_stat.config(text=upload_text)
            self.root.after(30, lambda: self.simulate_upload_progress(val + 2))
        else:
            self.add_log(self.translator.get('log.server_upload_success', 'SERVER: Upload successful. Data stored in cloud.'))
            self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            messagebox.showinfo(
                self.translator.get('dialog.success', 'Success'),
                self.translator.get('message.all_map_data_uploaded', 'All map data has been uploaded to the server!')
            )

    def browse_file(self):
        initial_dir = "/media/an/01DC80D9DB838380/"
        bag_path = filedialog.askdirectory(
            title=self.translator.get('dialog.choose_bag_folder', 'Choose Bag Folder'),
            initialdir=initial_dir
        )
        
        if bag_path:
            bag_path_obj = Path(bag_path)
            if bag_path_obj.exists():
                self.bag_path_var.set(str(bag_path_obj))
                self.bag_path = str(bag_path_obj)
                self.add_log_success('message.bag_selected', name=bag_path_obj.name)
            else:
                error_msg = self.translator.get('message.bag_folder_not_exists', 'Bag folder does not exist: {path}').replace('{path}', bag_path)
                messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)

    def launch_rviz_cmd(self):
        self.add_log(self.translator.get('log.system_launching_rviz2', 'SYSTEM: Launching RViz2...'))

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
    
    def add_log_success(self, message_key, **kwargs):
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"✅ {msg}")
    
    def add_log_warning(self, message_key, **kwargs):
        msg = self.translator.get(message_key, message_key)
        for key, value in kwargs.items():
            msg = msg.replace(f'{{{key}}}', str(value))
        self.add_log(f"⚠️ {msg}")

    def set_default_config(self):
        default_config = self.workspace_path / "src" / "FAST-LIVO2" / "config" / "mid360_equirectangular_stable.yaml"
        if default_config.exists():
            self.config_path = str(default_config)
            self.add_log_success('message.default_config_selected', name=default_config.name)
        else:
            self.add_log_warning('message.default_config_not_found', name='mid360_equirectangular_stable.yaml')

    def stop_process(self):
        self.btn_start.config(state=tk.NORMAL)
        self.btn_upload.config(state=tk.DISABLED)
        self.progress['value'] = 0
        self.status_label.config(text=self.translator.get('label.status_stopped', 'Status: Stopped'))
        self.add_log(self.translator.get('log.process_terminated', 'PROCESS: Mapping and Upload terminated.'))

if __name__ == "__main__":
    root = tk.Tk()
    app = BagMappingInterface(root)
    root.mainloop()