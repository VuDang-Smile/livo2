"""
UI Components Module
Chứa các function setup UI components cho Bag Mapping Interface
"""
import tkinter as tk
from tkinter import ttk


class UIComponentsBuilder:
    """Builder class để tạo UI components"""
    
    def __init__(self, workspace_frame, translator, config, callbacks):
        """
        Initialize UI builder
        
        Args:
            workspace_frame: Tkinter Frame để chứa UI
            translator: Translator instance
            config: BagMappingConfig instance
            callbacks: Dict chứa các callback functions:
                - browse_file: function để browse bag file
                - start_mapping: function để start mapping
                - stop_process: function để stop process
                - start_upload: function để start upload
                - update_preview_mode: function để update preview mode
                - clear_log: function để clear log
                - clear_qr_codes: function để clear QR codes
                - update_design_type: function để update design type
                - browse_design_file: function để browse design file
        """
        self.workspace = workspace_frame
        self.translator = translator
        self.config = config
        self.callbacks = callbacks
        
        # Widgets sẽ được tạo
        self.control_card = None
        self.preview_card = None
        self.select_bag_label = None
        self.bag_path_var = None
        self.browse_btn = None
        self.vehicle_info_var = None
        self.btn_start = None
        self.btn_stop = None
        self.status_label = None
        self.rviz_var = None
        self.rviz_check = None
        self.btn_upload = None
        self.progress = None
        self.upload_stat = None
        self.progress_label = None
        self.qr_label = None
        self.btn_clear_qr = None
        self.comparison_frame = None
        self.design_file_type_label = None
        self.design_type_var = None
        self.design_file_label = None
        self.design_path_var = None
        self.design_browse_btn = None
        self.comparison_status_label = None
        self.tunnel_entrance_x_var = None
        self.tunnel_entrance_y_var = None
        self.tunnel_entrance_z_var = None
        self.tunnel_entrance_coords_label = None
        self.tunnel_entrance_x_label = None
        self.tunnel_entrance_y_label = None
        self.tunnel_entrance_z_label = None
        self.qr_listbox = None
        self.system_log_label = None
        self.clear_log_btn = None
        self.log_panel = None
    
    def setup_control_card(self, get_mac_callback):
        """Setup control card với bag file selection và control buttons"""
        self.control_card = tk.LabelFrame(
            self.workspace, 
            text=self.translator.get('label.execution_control', 'Execution Control'), 
            padx=15, 
            pady=10
        )
        self.control_card.pack(fill=tk.X, pady=(0, 10))
        
        # File Path
        self.select_bag_label = tk.Label(
            self.control_card, 
            text=self.translator.get('label.select_bag_file', 'Select Bag File:')
        )
        self.select_bag_label.pack(anchor="w")
        path_frame = tk.Frame(self.control_card)
        path_frame.pack(fill=tk.X, pady=5)
        self.bag_path_var = tk.StringVar()
        tk.Entry(path_frame, textvariable=self.bag_path_var).pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.browse_btn = tk.Button(
            path_frame, 
            text=self.translator.get('button.browse', 'Browse'), 
            command=self.callbacks['browse_file']
        )
        self.browse_btn.pack(side=tk.LEFT, padx=5)

        # Vehicle Info input
        meta_frame = tk.Frame(self.control_card)
        meta_frame.pack(fill=tk.X, pady=(5, 5))

        # Vehicle info (Name - ID)
        vehicle_label = tk.Label(meta_frame, text=self.translator.get('label.vehicle_info', 'Vehicle (Name - ID):'))
        vehicle_label.grid(row=0, column=0, sticky="w", padx=(0, 5), pady=2)
        self.vehicle_info_var = tk.StringVar()
        vehicle_entry = tk.Entry(meta_frame, textvariable=self.vehicle_info_var)
        vehicle_entry.grid(row=0, column=1, sticky="we", padx=(0, 5), pady=2)

        # Auto-fill vehicle_id từ MAC local và đặt read-only nếu có
        local_vehicle_id = get_mac_callback()
        if local_vehicle_id:
            self.vehicle_info_var.set(local_vehicle_id)
            vehicle_entry.config(state="readonly")
        else:
            self.vehicle_info_var.set(self.translator.get('label.vehicle_mac_not_found', 'MAC not found'))

        meta_frame.columnconfigure(1, weight=1)

        # RViz checkbox section - đặt sau Vehicle Info
        rviz_frame = tk.Frame(self.control_card)
        rviz_frame.pack(fill=tk.X, pady=(5, 5))
        
        self.rviz_var = tk.BooleanVar(value=False)
        self.rviz_check = tk.Checkbutton(
            rviz_frame,
            text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'),
            variable=self.rviz_var,
            command=self.callbacks['update_preview_mode'],
        )
        self.rviz_check.pack(side=tk.LEFT)

        # Control Buttons
        btn_row = tk.Frame(self.control_card)
        btn_row.pack(fill=tk.X, pady=5)
        self.btn_start = tk.Button(
            btn_row, 
            text=self.translator.get('button.start_mapping', '🚀 START MAPPING'), 
            width=self.config.button_start_width, 
            command=self.callbacks['start_mapping']
        )
        self.btn_start.pack(side=tk.LEFT, padx=5)
        self.btn_stop = tk.Button(
            btn_row, 
            text=self.translator.get('button.stop_process', '■ STOP'), 
            width=self.config.button_stop_width, 
            command=self.callbacks['stop_process'],
            state=tk.DISABLED  # Disable khi khởi tạo
        )
        self.btn_stop.pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(
            btn_row, 
            text=self.translator.get('label.status_ready', 'Status: Ready'), 
            font=self.config.font_primary
        )
        self.status_label.pack(side=tk.RIGHT, padx=10)
    
    def setup_preview_rviz_section(self, design_file_type):
        """Setup preview/RViz section với progress, design comparison, QR codes, và log"""
        self.preview_card = tk.LabelFrame(
            self.workspace,
            text="",
            padx=15,
            pady=10,
        )
        self.preview_card.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        # Design map comparison section
        self.comparison_frame = tk.LabelFrame(
            self.preview_card,
            text=self.translator.get('label.design_map_comparison', 'Design Map Comparison'),
            padx=10,
            pady=8,
        )
        self.comparison_frame.pack(fill=tk.X, pady=(0, 10))

        # Loại file thiết kế: PCD / OBJ
        design_type_frame = tk.Frame(self.comparison_frame)
        design_type_frame.pack(fill=tk.X, pady=(0, 5))

        self.design_file_type_label = tk.Label(
            design_type_frame,
            text=self.translator.get('label.design_file_type', 'Design file type:'),
            font=self.config.font_primary,
        )
        self.design_file_type_label.pack(side=tk.LEFT)

        self.design_type_var = tk.StringVar(value=design_file_type)
        design_pcd_radio = tk.Radiobutton(
            design_type_frame,
            text="PCD",
            variable=self.design_type_var,
            value="pcd",
            command=self.callbacks['update_design_type'],
        )
        design_pcd_radio.pack(side=tk.LEFT, padx=5)

        design_obj_radio = tk.Radiobutton(
            design_type_frame,
            text="OBJ",
            variable=self.design_type_var,
            value="obj",
            command=self.callbacks['update_design_type'],
        )
        design_obj_radio.pack(side=tk.LEFT, padx=5)

        # Chọn file thiết kế
        design_file_frame = tk.Frame(self.comparison_frame)
        design_file_frame.pack(fill=tk.X, pady=(0, 5))

        self.design_file_label = tk.Label(
            design_file_frame,
            text=self.translator.get('label.design_file', 'Design file:'),
            font=self.config.font_secondary,
        )
        self.design_file_label.pack(side=tk.LEFT)

        self.design_path_var = tk.StringVar()
        design_entry = tk.Entry(design_file_frame, textvariable=self.design_path_var)
        design_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

        self.design_browse_btn = tk.Button(
            design_file_frame,
            text=self.translator.get('button.browse', 'Browse'),
            command=self.callbacks['browse_design_file'],
        )
        self.design_browse_btn.pack(side=tk.LEFT, padx=5)

        # Tunnel Entrance Coordinates - đặt trong Design Map Comparison, sau phần chọn path
        coords_frame = tk.Frame(self.comparison_frame)
        coords_frame.pack(fill=tk.X, pady=(0, 5))

        # Label cho tunnel entrance coordinates
        self.tunnel_entrance_coords_label = tk.Label(
            coords_frame,
            text=self.translator.get('label.tunnel_entrance_coordinates', 'Tunnel Entrance Coordinates:'),
            font=self.config.font_secondary,
        )
        self.tunnel_entrance_coords_label.pack(side=tk.LEFT, padx=(0, 5))

        # X coordinate
        self.tunnel_entrance_x_label = tk.Label(coords_frame, text=self.translator.get('label.coordinate_x', 'X:'), font=self.config.font_secondary)
        self.tunnel_entrance_x_label.pack(side=tk.LEFT, padx=(5, 2))
        self.tunnel_entrance_x_var = tk.StringVar()
        x_entry = tk.Entry(coords_frame, textvariable=self.tunnel_entrance_x_var, width=12)
        x_entry.pack(side=tk.LEFT, padx=(0, 5))

        # Y coordinate
        self.tunnel_entrance_y_label = tk.Label(coords_frame, text=self.translator.get('label.coordinate_y', 'Y:'), font=self.config.font_secondary)
        self.tunnel_entrance_y_label.pack(side=tk.LEFT, padx=(5, 2))
        self.tunnel_entrance_y_var = tk.StringVar()
        y_entry = tk.Entry(coords_frame, textvariable=self.tunnel_entrance_y_var, width=12)
        y_entry.pack(side=tk.LEFT, padx=(0, 5))

        # Z coordinate
        self.tunnel_entrance_z_label = tk.Label(coords_frame, text=self.translator.get('label.coordinate_z', 'Z:'), font=self.config.font_secondary)
        self.tunnel_entrance_z_label.pack(side=tk.LEFT, padx=(5, 2))
        self.tunnel_entrance_z_var = tk.StringVar()
        z_entry = tk.Entry(coords_frame, textvariable=self.tunnel_entrance_z_var, width=12)
        z_entry.pack(side=tk.LEFT, padx=(0, 5))

        # Nhãn hiển thị kết quả so sánh
        self.comparison_status_label = tk.Label(
            self.comparison_frame,
            text=self.translator.get('label.design_comparison_not_configured', 'Design comparison: not configured'),
            font=self.config.font_large,
            fg="gray",
        )
        self.comparison_status_label.pack(fill=tk.X, pady=(2, 0))

        # Progress section (Server Upload) - đặt sau Design Map Comparison
        progress_frame = tk.Frame(self.preview_card)
        progress_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.progress_label = tk.Label(
            progress_frame, 
            text=self.translator.get('label.server_synchronization', 'Server Synchronization'), 
            font=self.config.font_primary
        )
        self.progress_label.pack(anchor=tk.W, pady=(0, 5))
        
        progress_controls = tk.Frame(progress_frame)
        progress_controls.pack(fill=tk.X)
        
        self.btn_upload = tk.Button(
            progress_controls, 
            text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'), 
            state=tk.NORMAL, 
            command=self.callbacks['start_upload']
        )
        self.btn_upload.pack(side=tk.LEFT, padx=5)
        
        self.progress = ttk.Progressbar(progress_controls, orient=tk.HORIZONTAL, mode='determinate')
        self.progress.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=10)
        
        self.upload_stat = tk.Label(
            progress_controls, 
            text=self.translator.get('label.waiting', 'Waiting...'), 
            font=self.config.font_small
        )
        self.upload_stat.pack(side=tk.RIGHT)

        # Separator
        separator2 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator2.pack(fill=tk.X, pady=(0, 10))
        
        # 2.5. QR Code section
        qr_frame = tk.Frame(self.preview_card)
        qr_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.qr_label = tk.Label(
            qr_frame, 
            text=self.translator.get('label.qr_codes_detected', 'QR Codes Detected'), 
            font=self.config.font_primary
        )
        self.qr_label.pack(anchor=tk.W, pady=(0, 5))
        
        qr_controls = tk.Frame(qr_frame)
        qr_controls.pack(fill=tk.X)
        
        # QR codes listbox với scrollbar
        qr_list_frame = tk.Frame(qr_controls)
        qr_list_frame.pack(fill=tk.BOTH, expand=True)
        
        scrollbar_qr = tk.Scrollbar(qr_list_frame)
        scrollbar_qr.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.qr_listbox = tk.Listbox(
            qr_list_frame, 
            height=self.config.qr_listbox_height, 
            yscrollcommand=scrollbar_qr.set, 
            font=self.config.font_log_small
        )
        self.qr_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_qr.config(command=self.qr_listbox.yview)
        
        # Button to clear QR codes
        self.btn_clear_qr = tk.Button(
            qr_controls, 
            text=self.translator.get('button.clear_qr_codes', 'Clear QR Codes'), 
            font=self.config.font_small, 
            command=self.callbacks['clear_qr_codes']
        )
        self.btn_clear_qr.pack(side=tk.RIGHT, padx=5)
        
        # Separator
        separator3 = ttk.Separator(self.preview_card, orient=tk.HORIZONTAL)
        separator3.pack(fill=tk.X, pady=(0, 10))
        
        # 3. Log section
        log_container = tk.Frame(self.preview_card)
        log_container.pack(fill=tk.BOTH, expand=True)
        
        header = tk.Frame(log_container)
        header.pack(fill=tk.X, pady=(0, 2))
        self.system_log_label = tk.Label(
            header, 
            text=self.translator.get('label.system_log', 'SYSTEM LOG'), 
            font=self.config.font_small
        )
        self.system_log_label.pack(side=tk.LEFT)
        self.clear_log_btn = tk.Button(
            header, 
            text=self.translator.get('button.clear_log', 'Clear Log'), 
            font=self.config.font_small, 
            command=self.callbacks['clear_log']
        )
        self.clear_log_btn.pack(side=tk.RIGHT)

        self.log_panel = tk.Text(
            log_container, 
            height=self.config.log_panel_height, 
            bg="white", 
            fg="black", 
            font=self.config.font_log, 
            state=tk.DISABLED, 
            bd=1, 
            relief=tk.SUNKEN,
            wrap=tk.WORD, 
            padx=5, 
            pady=5
        )
        # Thêm scrollbar cho log
        log_scrollbar = tk.Scrollbar(log_container, orient=tk.VERTICAL, command=self.log_panel.yview)
        self.log_panel.config(yscrollcommand=log_scrollbar.set)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    
    def update_ui_texts(self):
        """Update tất cả UI texts khi thay đổi ngôn ngữ"""
        # Control card
        if self.control_card:
            self.control_card.config(text=self.translator.get('label.execution_control', 'Execution Control'))
        
        if self.select_bag_label:
            self.select_bag_label.config(text=self.translator.get('label.select_bag_file', 'Select Bag File:'))
        
        if self.browse_btn:
            self.browse_btn.config(text=self.translator.get('button.browse', 'Browse'))
        
        if self.btn_stop:
            self.btn_stop.config(text=self.translator.get('button.stop_process', '■ STOP'))
        
        if self.status_label:
            current_status = self.status_label.cget('text')
            if 'Mapping node running' in current_status or 'マッピングノード実行中' in current_status:
                self.status_label.config(text=self.translator.get('label.status_mapping_running', 'Status: 📡 Mapping node running'))
            elif 'Mapping' in current_status and 'node' not in current_status:
                self.status_label.config(text=self.translator.get('label.status_mapping', 'Status: Mapping...'))
            elif 'Map Created' in current_status or 'マップ作成済み' in current_status:
                self.status_label.config(text=self.translator.get('label.status_map_created', 'Status: Map Created'))
            elif 'Stopped' in current_status or '停止' in current_status:
                self.status_label.config(text=self.translator.get('label.status_stopped', 'Status: Stopped'))
            else:
                self.status_label.config(text=self.translator.get('label.status_ready', 'Status: Ready'))
        
        if self.btn_start:
            self.btn_start.config(text=self.translator.get('button.start_mapping', '🚀 START MAPPING'))
        
        # Preview card
        if self.preview_card:
            self.preview_card.config(text="")
        
        if self.rviz_check:
            self.rviz_check.config(text=self.translator.get('label.use_rviz2_external_view', 'Use RViz2 External View'))
        
        if self.progress_label:
            self.progress_label.config(text=self.translator.get('label.server_synchronization', 'Server Synchronization'))
        
        if self.btn_upload:
            self.btn_upload.config(text=self.translator.get('button.upload_to_server', '☁ UPLOAD TO SERVER'))
        
        if self.upload_stat:
            current_text = self.upload_stat.cget('text')
            if 'Ready' in current_text:
                self.upload_stat.config(text=self.translator.get('label.ready_to_upload', 'Ready to upload'))
            elif 'Done' in current_text or '100%' in current_text:
                self.upload_stat.config(text=self.translator.get('label.upload_done', '100% - Done'))
            else:
                self.upload_stat.config(text=self.translator.get('label.waiting', 'Waiting...'))
        
        if self.qr_label:
            self.qr_label.config(text=self.translator.get('label.qr_codes_detected', 'QR Codes Detected'))
        
        if self.btn_clear_qr:
            self.btn_clear_qr.config(text=self.translator.get('button.clear_qr_codes', 'Clear QR Codes'))
        
        if self.system_log_label:
            self.system_log_label.config(text=self.translator.get('label.system_log', 'SYSTEM LOG'))
        
        if self.clear_log_btn:
            self.clear_log_btn.config(text=self.translator.get('button.clear_log', 'Clear Log'))
        
        # Tunnel entrance coordinates section
        if self.tunnel_entrance_coords_label:
            self.tunnel_entrance_coords_label.config(
                text=self.translator.get('label.tunnel_entrance_coordinates', 'Tunnel Entrance Coordinates:')
            )
        
        if self.tunnel_entrance_x_label:
            self.tunnel_entrance_x_label.config(text=self.translator.get('label.coordinate_x', 'X:'))
        
        if self.tunnel_entrance_y_label:
            self.tunnel_entrance_y_label.config(text=self.translator.get('label.coordinate_y', 'Y:'))
        
        if self.tunnel_entrance_z_label:
            self.tunnel_entrance_z_label.config(text=self.translator.get('label.coordinate_z', 'Z:'))
        
        # Design comparison section
        if self.comparison_frame:
            self.comparison_frame.config(
                text=self.translator.get('label.design_map_comparison', 'Design Map Comparison')
            )
        
        if self.design_file_type_label:
            self.design_file_type_label.config(
                text=self.translator.get('label.design_file_type', 'Design file type:')
            )
        
        if self.design_file_label:
            self.design_file_label.config(
                text=self.translator.get('label.design_file', 'Design file:')
            )
        
        if self.design_browse_btn:
            self.design_browse_btn.config(
                text=self.translator.get('button.browse', 'Browse')
            )
        
        if self.comparison_status_label:
            current_text = self.comparison_status_label.cget('text')
            # Only update if it's the default "not configured" text
            if 'not configured' in current_text.lower() or '未設定' in current_text:
                self.comparison_status_label.config(
                    text=self.translator.get('label.design_comparison_not_configured', 
                        'Design comparison: not configured')
                )
