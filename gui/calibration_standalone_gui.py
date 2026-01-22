#!/usr/bin/env python3
"""
Calibration Standalone GUI Module
GUI riêng biệt cho Calibration với Converter, Replay và đầy đủ tính năng Calibration
"""

import threading
import subprocess
import json
import re
import yaml
from pathlib import Path
from datetime import datetime
import os
import sys
from functools import partial

# Thêm project root vào sys.path để có thể import các module
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext, filedialog
except ImportError as e:
    print(f"Lỗi import: {e}")
    sys.exit(1)

# Import các tabs
from replay_calibration_tab import ReplayCalibrationTab
from languages.translate_engine import Translator


class CalibrationStandaloneGUI(ttk.Frame):
    """GUI chính cho Calibration Standalone với Notebook chứa Converter, Replay và Calibration tabs"""
    
    def __init__(self, parent, translator=None):
        super().__init__(parent)
        
        # Translator for multi-language support
        self.translator = translator if translator else Translator('en')
        self.current_lang = self.translator.lang_code
        
        self.workspace_path = Path(__file__).parent.parent / "ws"
        self.drive_ws_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
        
        # Processes
        self.record_process = None
        self.preprocess_process = None
        self.initial_guess_process = None
        self.calibrate_process = None
        self.camera_info_publisher_process = None
        self.converter_process = None
        
        # Paths
        self.bag_output_dir = None
        self.preprocessed_dir = None
        
        # State
        self.is_recording = False
        self.is_camera_info_publisher_running = False
        self.is_converter_running = False
        self.current_step = "idle"  # idle, recording, preprocessing, calibrating
        
        # References to tabs
        self.replay_tab = None
        
        self.create_widgets()
        
        # Tự động khởi động Converter và camera_info_publisher sau khi GUI được tạo
        self.after(500, self.start_converter)
        self.after(1200, self.launch_camera_info_publisher_auto)
    
    def create_widgets(self):
        """Tạo các widgets cho GUI"""
        
        # Title
        self.title_label = ttk.Label(
            self,
            text=self.translator.get("title.calibration_standalone_gui", "Calibration Standalone GUI"),
            font=("Arial", 18, "bold")
        )
        self.title_label.pack(pady=10)
        
        # Main container với notebook để chia thành các tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Tab 0: Replay Calibration
        replay_calib_frame = ttk.Frame(self.notebook)
        self.notebook.add(replay_calib_frame, text=self.translator.get("tab.replay_calibration", "Replay Calibration"))
        self.replay_calibration_tab = ReplayCalibrationTab(replay_calib_frame, translator=self.translator)
        self.replay_calibration_tab.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Preprocessing
        preprocess_frame = ttk.Frame(self.notebook)
        self.notebook.add(preprocess_frame, text=self.translator.get("tab.preprocessing", "Preprocessing"))
        self.create_preprocess_tab(preprocess_frame)
        
        # Tab 2: Initial Guess
        initial_guess_frame = ttk.Frame(self.notebook)
        self.notebook.add(initial_guess_frame, text=self.translator.get("tab.initial_guess", "Initial Guess"))
        self.create_initial_guess_tab(initial_guess_frame)
        
        # Tab 3: Calibration
        calibrate_frame = ttk.Frame(self.notebook)
        self.notebook.add(calibrate_frame, text=self.translator.get("tab.calibration", "Calibration"))
        self.create_calibrate_tab(calibrate_frame)
        
        # Tab 4: Export Results
        export_frame = ttk.Frame(self.notebook)
        self.notebook.add(export_frame, text=self.translator.get("tab.export_results", "Export Results"))
        self.create_export_tab(export_frame)
        
        # Status bar
        self.status_label = ttk.Label(
            self,
            text=self.translator.get("status.ready", "Status: Ready"),
            font=("Arial", 10)
        )
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        # Bỏ Log Tổng UI theo yêu cầu; vẫn giữ biến để tránh lỗi tham chiếu
        self.global_log = None
    
    def update_ui_texts(self):
        """Update all UI texts based on current language"""
        # Update title
        if hasattr(self, 'title_label'):
            self.title_label.config(text=self.translator.get("title.calibration_standalone_gui", "Calibration Standalone GUI"))
        
        # Update status label
        if hasattr(self, 'status_label'):
            # Keep current status text if it's a dynamic status, otherwise set to ready
            current_text = self.status_label.cget("text")
            if "Ready" in current_text or "Sẵn sàng" in current_text:
                self.status_label.config(text=self.translator.get("status.ready", "Status: Ready"))
        
        # Update notebook tabs
        if hasattr(self, 'notebook'):
            try:
                self.notebook.tab(0, text=self.translator.get("tab.replay_calibration", "Replay Calibration"))
                self.notebook.tab(1, text=self.translator.get("tab.preprocessing", "Preprocessing"))
                self.notebook.tab(2, text=self.translator.get("tab.initial_guess", "Initial Guess"))
                self.notebook.tab(3, text=self.translator.get("tab.calibration", "Calibration"))
                self.notebook.tab(4, text=self.translator.get("tab.export_results", "Export Results"))
            except:
                pass
        
        # Update preprocess tab
        if hasattr(self, 'preprocess_instructions'):
            self.preprocess_instructions.config(text=self.translator.get("calibration.preprocess_instructions", "Preprocessing dữ liệu từ rosbag để chuẩn bị cho calibration (equirectangular camera)"))
        if hasattr(self, 'preprocess_input_label'):
            self.preprocess_input_label.config(text=self.translator.get("calibration.label.bags_directory", "Thư mục bags:"))
        if hasattr(self, 'preprocess_output_label'):
            self.preprocess_output_label.config(text=self.translator.get("calibration.label.output_directory", "Thư mục output:"))
        if hasattr(self, 'browse_preprocess_input_btn'):
            self.browse_preprocess_input_btn.config(text=self.translator.get("button.browse", "Browse"))
        if hasattr(self, 'browse_preprocess_output_btn'):
            self.browse_preprocess_output_btn.config(text=self.translator.get("button.browse", "Browse"))
        if hasattr(self, 'camera_model_frame'):
            self.camera_model_frame.config(text=self.translator.get("calibration.label.camera_model", "Camera Model"))
        if hasattr(self, 'camera_model_label'):
            self.camera_model_label.config(text=self.translator.get("calibration.label.select_camera_model", "Select camera projection model:"))
        if hasattr(self, 'camera_model_equirectangular'):
            self.camera_model_equirectangular.config(text=self.translator.get("calibration.camera_model.equirectangular", "Equirectangular (default for 360° cameras)"))
        if hasattr(self, 'camera_model_auto'):
            self.camera_model_auto.config(text=self.translator.get("calibration.camera_model.auto", "Auto-detect from camera_info topic"))
        if hasattr(self, 'camera_model_plumb_bob'):
            self.camera_model_plumb_bob.config(text=self.translator.get("calibration.camera_model.plumb_bob", "Plumb Bob (pinhole with radial/tangential distortion)"))
        if hasattr(self, 'camera_model_fisheye'):
            self.camera_model_fisheye.config(text=self.translator.get("calibration.camera_model.fisheye", "Fisheye (equidistant)"))
        if hasattr(self, 'camera_model_omnidir'):
            self.camera_model_omnidir.config(text=self.translator.get("calibration.camera_model.omnidir", "Omnidirectional"))
        if hasattr(self, 'preprocess_options_frame'):
            self.preprocess_options_frame.config(text=self.translator.get("calibration.label.options", "Options"))
        if hasattr(self, 'auto_topic_check'):
            self.auto_topic_check.config(text=self.translator.get("calibration.option.auto_detect_topics", "Auto-detect topics (-a)"))
        if hasattr(self, 'dynamic_lidar_check'):
            self.dynamic_lidar_check.config(text=self.translator.get("calibration.option.dynamic_lidar", "Dynamic LiDAR integration (-d) - cho spinning LiDAR"))
        if hasattr(self, 'visualize_check'):
            self.visualize_check.config(text=self.translator.get("calibration.option.visualize", "Visualize (-v)"))
        if hasattr(self, 'preprocess_btn'):
            self.preprocess_btn.config(text=self.translator.get("calibration.button.run_preprocessing", "Chạy Preprocessing"))
        if hasattr(self, 'preprocess_log_frame'):
            self.preprocess_log_frame.config(text=self.translator.get("label.log", "Log"))
        
        # Update initial guess tab
        if hasattr(self, 'initial_guess_instructions'):
            self.initial_guess_instructions.config(text=self.translator.get("calibration.initial_guess_instructions", "Tạo initial guess cho calibration (manual hoặc automatic)"))
        if hasattr(self, 'initial_guess_dir_label'):
            self.initial_guess_dir_label.config(text=self.translator.get("calibration.label.preprocessed_directory", "Thư mục preprocessed:"))
        if hasattr(self, 'browse_initial_guess_btn'):
            self.browse_initial_guess_btn.config(text=self.translator.get("button.browse", "Browse"))
        if hasattr(self, 'initial_guess_mode_frame'):
            self.initial_guess_mode_frame.config(text=self.translator.get("calibration.label.mode", "Mode"))
        if hasattr(self, 'initial_guess_manual_radio'):
            self.initial_guess_manual_radio.config(text=self.translator.get("calibration.mode.manual", "Manual - Chọn correspondences thủ công"))
        if hasattr(self, 'initial_guess_auto_radio'):
            self.initial_guess_auto_radio.config(text=self.translator.get("calibration.mode.automatic", "Automatic - Sử dụng SuperGlue (cần license)"))
        if hasattr(self, 'initial_guess_btn'):
            self.initial_guess_btn.config(text=self.translator.get("calibration.button.run_initial_guess", "Chạy Initial Guess"))
        if hasattr(self, 'initial_guess_log_frame'):
            self.initial_guess_log_frame.config(text=self.translator.get("label.log", "Log"))
        
        # Update calibrate tab
        if hasattr(self, 'calibrate_instructions'):
            self.calibrate_instructions.config(text=self.translator.get("calibration.calibrate_instructions", "Chạy fine registration để tinh chỉnh calibration"))
        if hasattr(self, 'calibrate_dir_label'):
            self.calibrate_dir_label.config(text=self.translator.get("calibration.label.preprocessed_directory", "Thư mục preprocessed:"))
        if hasattr(self, 'browse_calibrate_btn'):
            self.browse_calibrate_btn.config(text=self.translator.get("button.browse", "Browse"))
        if hasattr(self, 'calibrate_options_frame'):
            self.calibrate_options_frame.config(text=self.translator.get("calibration.label.options", "Options"))
        if hasattr(self, 'auto_quit_check'):
            self.auto_quit_check.config(text=self.translator.get("calibration.option.auto_quit", "Auto quit sau khi calibration xong (khuyến nghị)"))
        if hasattr(self, 'background_check'):
            self.background_check.config(text=self.translator.get("calibration.option.background", "Chạy background (không hiển thị viewer) - khuyến nghị"))
        if hasattr(self, 'registration_type_label'):
            self.registration_type_label.config(text=self.translator.get("calibration.label.registration_type", "Registration type:"))
        if hasattr(self, 'registration_nid_bfgs'):
            self.registration_nid_bfgs.config(text=self.translator.get("calibration.registration_type.nid_bfgs", "NID-BFGS (nhanh hơn, khuyến nghị)"))
        if hasattr(self, 'registration_nid_nelder_mead'):
            self.registration_nid_nelder_mead.config(text=self.translator.get("calibration.registration_type.nid_nelder_mead", "NID-Nelder-Mead (chậm hơn nhưng ổn định hơn)"))
        if hasattr(self, 'calibrate_btn'):
            self.calibrate_btn.config(text=self.translator.get("calibration.button.run_calibration", "Chạy Calibration"))
        if hasattr(self, 'stop_calibrate_btn'):
            self.stop_calibrate_btn.config(text=self.translator.get("calibration.button.stop_calibration", "Dừng Calibration"))
        if hasattr(self, 'calibrate_log_frame'):
            self.calibrate_log_frame.config(text=self.translator.get("label.log", "Log"))
        
        # Update export tab
        if hasattr(self, 'export_instructions'):
            self.export_instructions.config(text=self.translator.get("calibration.export_instructions", "Export và convert kết quả calibration sang format FAST-LIVO2"))
        if hasattr(self, 'calib_json_label'):
            self.calib_json_label.config(text=self.translator.get("calibration.label.calib_json_file", "File calib.json:"))
        if hasattr(self, 'browse_calib_btn'):
            self.browse_calib_btn.config(text=self.translator.get("button.browse", "Browse"))
        if hasattr(self, 'output_yaml_label'):
            self.output_yaml_label.config(text=self.translator.get("calibration.label.output_yaml", "Output YAML:"))
        if hasattr(self, 'convert_btn'):
            self.convert_btn.config(text=self.translator.get("calibration.button.convert_to_fast_livo2", "Convert sang FAST-LIVO2"))
        if hasattr(self, 'results_frame'):
            self.results_frame.config(text=self.translator.get("calibration.label.results", "Kết quả"))
        
        # Update ReplayCalibrationTab texts if it exists
        if hasattr(self, 'replay_calibration_tab') and hasattr(self.replay_calibration_tab, 'update_ui_texts'):
            self.replay_calibration_tab.update_ui_texts()
    
    def log_global(self, message, prefix=""):
        """Log message vào log tổng (thread-safe)"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        full_message = f"[{timestamp}] {prefix}{message}" if prefix else f"[{timestamp}] {message}"
        self.after(0, partial(self._log_global_impl, full_message))

    def _append_results_text(self, content, clear=False):
        """Cập nhật ô kết quả theo cách thread-safe"""
        def _update():
            try:
                self.results_text.config(state=tk.NORMAL)
                if clear:
                    self.results_text.delete(1.0, tk.END)
                text_to_insert = content if content.endswith("\n") else f"{content}\n"
                self.results_text.insert(tk.END, text_to_insert)
                self.results_text.see(tk.END)
                self.results_text.config(state=tk.DISABLED)
            except Exception as exc:
                print(f"Lỗi khi cập nhật kết quả: {exc}")
        self.after(0, _update)
    
    def _log_global_impl(self, message):
        """Implementation của log_global (chạy trong main thread)"""
        # Không còn UI log tổng; fallback in ra console
        print(message)
    
    def create_record_tab(self, parent):
        """Tạo tab record bag (chỉ cho equirectangular camera)"""
        # Instructions
        instructions = ttk.Label(
            parent,
            text="Ghi lại rosbag với topics /image_raw (equirectangular) và /livox/points2",
            font=("Arial", 10)
        )
        instructions.pack(pady=10)
        
        # Output directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        ttk.Label(dir_frame, text="Thư mục output:").pack(side=tk.LEFT, padx=5)
        self.bag_dir_var = tk.StringVar(value=str(self.workspace_path / "calibration_data" / "bags"))
        dir_entry = ttk.Entry(dir_frame, textvariable=self.bag_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        browse_btn = ttk.Button(
            dir_frame,
            text="Browse",
            command=self.browse_bag_directory
        )
        browse_btn.pack(side=tk.LEFT, padx=5)
        
        # Topics mặc định (ẩn, không hiển thị)
        self.topics_var = tk.StringVar(value="/image_raw /camera_info /livox/points2")
        
        # Record button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.record_btn = ttk.Button(
            button_frame,
            text="Bắt đầu Record",
            command=self.start_record,
            style="Accent.TButton"
        )
        self.record_btn.pack(side=tk.LEFT, padx=10)
        
        self.stop_record_btn = ttk.Button(
            button_frame,
            text="Dừng Record",
            command=self.stop_record,
            state=tk.DISABLED
        )
        self.stop_record_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        log_frame = ttk.LabelFrame(parent, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.record_log = scrolledtext.ScrolledText(
            log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.record_log.pack(fill=tk.BOTH, expand=True)
    
    def create_preprocess_tab(self, parent):
        """Tạo tab preprocessing"""
        # Instructions
        self.preprocess_instructions = ttk.Label(
            parent,
            text=self.translator.get("calibration.preprocess_instructions", "Preprocessing dữ liệu từ rosbag để chuẩn bị cho calibration (equirectangular camera)"),
            font=("Arial", 10)
        )
        self.preprocess_instructions.pack(pady=10)
        
        # Input bag directory
        input_frame = ttk.Frame(parent)
        input_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.preprocess_input_label = ttk.Label(input_frame, text=self.translator.get("calibration.label.bags_directory", "Thư mục bags:"))
        self.preprocess_input_label.pack(side=tk.LEFT, padx=5)
        self.preprocess_input_var = tk.StringVar()
        input_entry = ttk.Entry(input_frame, textvariable=self.preprocess_input_var, width=50)
        input_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.browse_preprocess_input_btn = ttk.Button(
            input_frame,
            text=self.translator.get("button.browse", "Browse"),
            command=self.browse_preprocess_input
        )
        self.browse_preprocess_input_btn.pack(side=tk.LEFT, padx=5)
        
        # Output preprocessed directory
        output_frame = ttk.Frame(parent)
        output_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.preprocess_output_label = ttk.Label(output_frame, text=self.translator.get("calibration.label.output_directory", "Thư mục output:"))
        self.preprocess_output_label.pack(side=tk.LEFT, padx=5)
        self.preprocess_output_var = tk.StringVar(value=str(self.workspace_path / "calibration_data" / "preprocessed"))
        output_entry = ttk.Entry(output_frame, textvariable=self.preprocess_output_var, width=50)
        output_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.browse_preprocess_output_btn = ttk.Button(
            output_frame,
            text=self.translator.get("button.browse", "Browse"),
            command=self.browse_preprocess_output
        )
        self.browse_preprocess_output_btn.pack(side=tk.LEFT, padx=5)
        
        # Camera model selection
        self.camera_model_frame = ttk.LabelFrame(parent, text=self.translator.get("calibration.label.camera_model", "Camera Model"), padding=10)
        self.camera_model_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.camera_model_label = ttk.Label(
            self.camera_model_frame,
            text=self.translator.get("calibration.label.select_camera_model", "Select camera projection model:"),
            font=("Arial", 9)
        )
        self.camera_model_label.pack(anchor=tk.W, pady=(0, 5))
        
        model_selection_frame = ttk.Frame(self.camera_model_frame)
        model_selection_frame.pack(fill=tk.X)
        
        self.camera_model_var = tk.StringVar(value="equirectangular")
        
        self.camera_model_equirectangular = ttk.Radiobutton(
            model_selection_frame,
            text=self.translator.get("calibration.camera_model.equirectangular", "Equirectangular (default for 360° cameras)"),
            variable=self.camera_model_var,
            value="equirectangular"
        )
        self.camera_model_equirectangular.pack(anchor=tk.W, pady=2)
        
        self.camera_model_auto = ttk.Radiobutton(
            model_selection_frame,
            text=self.translator.get("calibration.camera_model.auto", "Auto-detect from camera_info topic"),
            variable=self.camera_model_var,
            value="auto"
        )
        self.camera_model_auto.pack(anchor=tk.W, pady=2)
        
        self.camera_model_plumb_bob = ttk.Radiobutton(
            model_selection_frame,
            text=self.translator.get("calibration.camera_model.plumb_bob", "Plumb Bob (pinhole with radial/tangential distortion)"),
            variable=self.camera_model_var,
            value="plumb_bob"
        )
        self.camera_model_plumb_bob.pack(anchor=tk.W, pady=2)
        
        self.camera_model_fisheye = ttk.Radiobutton(
            model_selection_frame,
            text=self.translator.get("calibration.camera_model.fisheye", "Fisheye (equidistant)"),
            variable=self.camera_model_var,
            value="fisheye"
        )
        self.camera_model_fisheye.pack(anchor=tk.W, pady=2)
        
        self.camera_model_omnidir = ttk.Radiobutton(
            model_selection_frame,
            text=self.translator.get("calibration.camera_model.omnidir", "Omnidirectional"),
            variable=self.camera_model_var,
            value="omnidir"
        )
        self.camera_model_omnidir.pack(anchor=tk.W, pady=2)
        
        # Options
        self.preprocess_options_frame = ttk.LabelFrame(parent, text=self.translator.get("calibration.label.options", "Options"), padding=10)
        self.preprocess_options_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.auto_topic_var = tk.BooleanVar(value=True)
        self.auto_topic_check = ttk.Checkbutton(
            self.preprocess_options_frame,
            text=self.translator.get("calibration.option.auto_detect_topics", "Auto-detect topics (-a)"),
            variable=self.auto_topic_var
        )
        self.auto_topic_check.pack(anchor=tk.W)
        
        self.dynamic_lidar_var = tk.BooleanVar(value=False)
        self.dynamic_lidar_check = ttk.Checkbutton(
            self.preprocess_options_frame,
            text=self.translator.get("calibration.option.dynamic_lidar", "Dynamic LiDAR integration (-d) - cho spinning LiDAR"),
            variable=self.dynamic_lidar_var
        )
        self.dynamic_lidar_check.pack(anchor=tk.W)
        
        self.visualize_var = tk.BooleanVar(value=True)
        self.visualize_check = ttk.Checkbutton(
            self.preprocess_options_frame,
            text=self.translator.get("calibration.option.visualize", "Visualize (-v)"),
            variable=self.visualize_var
        )
        self.visualize_check.pack(anchor=tk.W)
        
        # Preprocess button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.preprocess_btn = ttk.Button(
            button_frame,
            text=self.translator.get("calibration.button.run_preprocessing", "Chạy Preprocessing"),
            command=self.start_preprocess,
            style="Accent.TButton"
        )
        self.preprocess_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        self.preprocess_log_frame = ttk.LabelFrame(parent, text=self.translator.get("label.log", "Log"), padding=10)
        self.preprocess_log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.preprocess_log = scrolledtext.ScrolledText(
            self.preprocess_log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.preprocess_log.pack(fill=tk.BOTH, expand=True)
    
    def create_initial_guess_tab(self, parent):
        """Tạo tab initial guess"""
        # Instructions
        self.initial_guess_instructions = ttk.Label(
            parent,
            text=self.translator.get("calibration.initial_guess_instructions", "Tạo initial guess cho calibration (manual hoặc automatic)"),
            font=("Arial", 10)
        )
        self.initial_guess_instructions.pack(pady=10)
        
        # Preprocessed directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.initial_guess_dir_label = ttk.Label(dir_frame, text=self.translator.get("calibration.label.preprocessed_directory", "Thư mục preprocessed:"))
        self.initial_guess_dir_label.pack(side=tk.LEFT, padx=5)
        self.initial_guess_dir_var = tk.StringVar()
        dir_entry = ttk.Entry(dir_frame, textvariable=self.initial_guess_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.browse_initial_guess_btn = ttk.Button(
            dir_frame,
            text=self.translator.get("button.browse", "Browse"),
            command=self.browse_initial_guess_dir
        )
        self.browse_initial_guess_btn.pack(side=tk.LEFT, padx=5)
        
        # Mode selection
        self.initial_guess_mode_frame = ttk.LabelFrame(parent, text=self.translator.get("calibration.label.mode", "Mode"), padding=10)
        self.initial_guess_mode_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.initial_guess_mode = tk.StringVar(value="manual")
        
        self.initial_guess_manual_radio = ttk.Radiobutton(
            self.initial_guess_mode_frame,
            text=self.translator.get("calibration.mode.manual", "Manual - Chọn correspondences thủ công"),
            variable=self.initial_guess_mode,
            value="manual"
        )
        self.initial_guess_manual_radio.pack(anchor=tk.W, pady=5)
        
        self.initial_guess_auto_radio = ttk.Radiobutton(
            self.initial_guess_mode_frame,
            text=self.translator.get("calibration.mode.automatic", "Automatic - Sử dụng SuperGlue (cần license)"),
            variable=self.initial_guess_mode,
            value="auto"
        )
        self.initial_guess_auto_radio.pack(anchor=tk.W, pady=5)
        
        # Buttons
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.initial_guess_btn = ttk.Button(
            button_frame,
            text=self.translator.get("calibration.button.run_initial_guess", "Chạy Initial Guess"),
            command=self.start_initial_guess,
            style="Accent.TButton"
        )
        self.initial_guess_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        self.initial_guess_log_frame = ttk.LabelFrame(parent, text=self.translator.get("label.log", "Log"), padding=10)
        self.initial_guess_log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.initial_guess_log = scrolledtext.ScrolledText(
            self.initial_guess_log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.initial_guess_log.pack(fill=tk.BOTH, expand=True)
    
    def create_calibrate_tab(self, parent):
        """Tạo tab calibration"""
        # Instructions
        self.calibrate_instructions = ttk.Label(
            parent,
            text=self.translator.get("calibration.calibrate_instructions", "Chạy fine registration để tinh chỉnh calibration"),
            font=("Arial", 10)
        )
        self.calibrate_instructions.pack(pady=10)
        
        # Preprocessed directory
        dir_frame = ttk.Frame(parent)
        dir_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.calibrate_dir_label = ttk.Label(dir_frame, text=self.translator.get("calibration.label.preprocessed_directory", "Thư mục preprocessed:"))
        self.calibrate_dir_label.pack(side=tk.LEFT, padx=5)
        self.calibrate_dir_var = tk.StringVar()
        dir_entry = ttk.Entry(dir_frame, textvariable=self.calibrate_dir_var, width=50)
        dir_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.browse_calibrate_btn = ttk.Button(
            dir_frame,
            text=self.translator.get("button.browse", "Browse"),
            command=self.browse_calibrate_dir
        )
        self.browse_calibrate_btn.pack(side=tk.LEFT, padx=5)
        
        # Options
        self.calibrate_options_frame = ttk.LabelFrame(parent, text=self.translator.get("calibration.label.options", "Options"), padding=10)
        self.calibrate_options_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.auto_quit_var = tk.BooleanVar(value=True)
        self.auto_quit_check = ttk.Checkbutton(
            self.calibrate_options_frame,
            text=self.translator.get("calibration.option.auto_quit", "Auto quit sau khi calibration xong (khuyến nghị)"),
            variable=self.auto_quit_var
        )
        self.auto_quit_check.pack(anchor=tk.W)
        
        self.background_var = tk.BooleanVar(value=True)
        self.background_check = ttk.Checkbutton(
            self.calibrate_options_frame,
            text=self.translator.get("calibration.option.background", "Chạy background (không hiển thị viewer) - khuyến nghị"),
            variable=self.background_var
        )
        self.background_check.pack(anchor=tk.W)
        
        # Registration type
        reg_frame = ttk.Frame(self.calibrate_options_frame)
        reg_frame.pack(fill=tk.X, pady=5)
        self.registration_type_label = ttk.Label(reg_frame, text=self.translator.get("calibration.label.registration_type", "Registration type:"))
        self.registration_type_label.pack(side=tk.LEFT, padx=5)
        self.registration_type_var = tk.StringVar(value="nid_bfgs")
        self.registration_nid_bfgs = ttk.Radiobutton(
            reg_frame,
            text=self.translator.get("calibration.registration_type.nid_bfgs", "NID-BFGS (nhanh hơn, khuyến nghị)"),
            variable=self.registration_type_var,
            value="nid_bfgs"
        )
        self.registration_nid_bfgs.pack(side=tk.LEFT, padx=5)
        self.registration_nid_nelder_mead = ttk.Radiobutton(
            reg_frame,
            text=self.translator.get("calibration.registration_type.nid_nelder_mead", "NID-Nelder-Mead (chậm hơn nhưng ổn định hơn)"),
            variable=self.registration_type_var,
            value="nid_nelder_mead"
        )
        self.registration_nid_nelder_mead.pack(side=tk.LEFT, padx=5)
        
        # Button
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.calibrate_btn = ttk.Button(
            button_frame,
            text=self.translator.get("calibration.button.run_calibration", "Chạy Calibration"),
            command=self.start_calibrate,
            style="Accent.TButton"
        )
        self.calibrate_btn.pack(side=tk.LEFT, padx=10)
        
        self.stop_calibrate_btn = ttk.Button(
            button_frame,
            text=self.translator.get("calibration.button.stop_calibration", "Dừng Calibration"),
            command=self.stop_calibrate,
            state=tk.DISABLED
        )
        self.stop_calibrate_btn.pack(side=tk.LEFT, padx=10)
        
        # Log
        self.calibrate_log_frame = ttk.LabelFrame(parent, text=self.translator.get("label.log", "Log"), padding=10)
        self.calibrate_log_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.calibrate_log = scrolledtext.ScrolledText(
            self.calibrate_log_frame,
            height=8,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.calibrate_log.pack(fill=tk.BOTH, expand=True)
    
    def create_export_tab(self, parent):
        """Tạo tab export results"""
        # Instructions
        self.export_instructions = ttk.Label(
            parent,
            text=self.translator.get("calibration.export_instructions", "Export và convert kết quả calibration sang format FAST-LIVO2"),
            font=("Arial", 10)
        )
        self.export_instructions.pack(pady=10)
        
        # Calib.json path
        calib_frame = ttk.Frame(parent)
        calib_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.calib_json_label = ttk.Label(calib_frame, text=self.translator.get("calibration.label.calib_json_file", "File calib.json:"))
        self.calib_json_label.pack(side=tk.LEFT, padx=5)
        self.calib_json_var = tk.StringVar()
        calib_entry = ttk.Entry(calib_frame, textvariable=self.calib_json_var, width=50)
        calib_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        self.browse_calib_btn = ttk.Button(
            calib_frame,
            text=self.translator.get("button.browse", "Browse"),
            command=self.browse_calib_json
        )
        self.browse_calib_btn.pack(side=tk.LEFT, padx=5)
        
        # Output YAML path
        yaml_frame = ttk.Frame(parent)
        yaml_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.output_yaml_label = ttk.Label(yaml_frame, text=self.translator.get("calibration.label.output_yaml", "Output YAML:"))
        self.output_yaml_label.pack(side=tk.LEFT, padx=5)
        self.output_yaml_var = tk.StringVar()
        yaml_entry = ttk.Entry(yaml_frame, textvariable=self.output_yaml_var, width=50)
        yaml_entry.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # Buttons
        button_frame = ttk.Frame(parent)
        button_frame.pack(pady=20)
        
        self.convert_btn = ttk.Button(
            button_frame,
            text=self.translator.get("calibration.button.convert_to_fast_livo2", "Convert sang FAST-LIVO2"),
            command=self.convert_to_fast_livo2,
            style="Accent.TButton"
        )
        self.convert_btn.pack(side=tk.LEFT, padx=10)
        
        # Results display
        self.results_frame = ttk.LabelFrame(parent, text=self.translator.get("calibration.label.results", "Kết quả"), padding=10)
        self.results_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        self.results_text = scrolledtext.ScrolledText(
            self.results_frame,
            height=10,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.results_text.pack(fill=tk.BOTH, expand=True)
    
    # ========== Record Bag Methods ==========
    
    def browse_bag_directory(self):
        """Browse cho bag output directory"""
        directory = filedialog.askdirectory(
            title=self.translator.get("calibration.dialog.choose_bag_directory", "Chọn thư mục để lưu bag"),
            initialdir=self.bag_dir_var.get()
        )
        if directory:
            self.bag_dir_var.set(directory)
    
    def start_record(self):
        """Bắt đầu record rosbag"""
        if self.is_recording:
            messagebox.showwarning(
                self.translator.get("dialog.warning", "Warning"),
                self.translator.get("calibration.message.recording_in_progress", "Đang record, vui lòng dừng trước")
            )
            return
        
        bag_dir = Path(self.bag_dir_var.get())
        if not bag_dir.exists():
            try:
                bag_dir.mkdir(parents=True, exist_ok=True)
            except Exception as e:
                messagebox.showerror(
                    self.translator.get("dialog.error", "Error"),
                    self.translator.get("calibration.message.cannot_create_directory", "Không thể tạo thư mục: {error}").replace("{error}", str(e))
                )
                return
        
        topics = self.topics_var.get().split()
        if not topics:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_enter_topic", "Vui lòng nhập ít nhất một topic")
            )
            return
        
        # Tạo tên bag file với timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"calibration_{timestamp}"
        bag_path = bag_dir / bag_name
        
        # Build command
        cmd = f"ros2 bag record -o {bag_path} {' '.join(topics)}"
        
        self.log_record(f"Bắt đầu record bag: {bag_name}")
        self.log_record(f"Topics: {', '.join(topics)}")
        self.log_record(f"Output: {bag_path}")
        
        try:
            self.record_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            self.is_recording = True
            self.record_btn.config(state=tk.DISABLED)
            self.stop_record_btn.config(state=tk.NORMAL)
            self.status_label.config(text=self.translator.get("calibration.status.recording_bag", "Trạng thái: Đang record bag..."), foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_record_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_start_record", "Không thể bắt đầu record: {error}").replace("{error}", str(e))
            )
            self.log_record(f"Lỗi: {e}")
    
    def stop_record(self):
        """Dừng record rosbag"""
        if self.record_process:
            try:
                self.record_process.terminate()
                self.record_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.record_process.kill()
            except Exception as e:
                self.log_record(f"Lỗi khi dừng record: {e}")
            
            self.record_process = None
            self.is_recording = False
            self.record_btn.config(state=tk.NORMAL)
            self.stop_record_btn.config(state=tk.DISABLED)
            self.status_label.config(text=self.translator.get("calibration.status.record_stopped", "Trạng thái: Đã dừng record"), foreground="green")
            self.log_record("Đã dừng record")
    
    def monitor_record_process(self):
        """Monitor record process output"""
        proc = self.record_process
        if not proc:
            return

        for line in iter(proc.stdout.readline, ''):
            if not line:
                break
            self.log_record(line.strip())

        # Chỉ kiểm tra poll trên handle cục bộ để tránh None khi đã dừng
        if proc.poll() is not None:
            self.is_recording = False
            self.after(0, partial(self._update_record_complete))
    
    def _update_record_complete(self):
        """Helper function để update UI sau khi record hoàn thành"""
        try:
            self.record_btn.config(state=tk.NORMAL)
            self.stop_record_btn.config(state=tk.DISABLED)
            self.status_label.config(text=self.translator.get("calibration.status.record_completed", "Trạng thái: Record đã hoàn thành"), foreground="green")
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_record(self, message):
        """Log message vào record log và log tổng (thread-safe)"""
        self.log_global(message, "[Record] ")
        self.after(0, partial(self._log_record_impl, message))
    
    def _log_record_impl(self, message):
        """Implementation của log_record (chạy trong main thread)"""
        try:
            self.record_log.config(state=tk.NORMAL)
            self.record_log.insert(tk.END, f"{message}\n")
            self.record_log.see(tk.END)
            self.record_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    def launch_camera_info_publisher_auto(self):
        """Tự động launch camera_info_publisher khi mở app"""
        try:
            self.launch_camera_info_publisher()
        except Exception as e:
            self.log_global(f"Lỗi auto launch camera_info_publisher: {e}", "[CameraInfoPublisher] ")
    
    def launch_camera_info_publisher(self):
        """Launch camera_info_publisher node cho equirectangular camera"""
        if self.is_camera_info_publisher_running:
            messagebox.showwarning(
                self.translator.get("dialog.warning", "Warning"),
                self.translator.get("calibration.message.camera_info_publisher_running", "Camera info publisher đang chạy")
            )
            return
        
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.setup_not_found", "Không tìm thấy setup.sh tại: {path}").replace("{path}", str(setup_script))
            )
            return
        
        # Build command để launch camera_info_publisher node
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        cmd = f"source {ros2_setup} && source {setup_script} && ros2 run theta_driver camera_info_publisher_node --ros-args -p image_topic:=\"/image_raw\" -p camera_info_topic:=\"/camera_info\" -p camera_frame:=\"camera_link\" -p use_calibration_params:=false"
        
        self.log_global("🚀 Đang launch camera_info_publisher cho equirectangular camera...", "[CameraInfoPublisher] ")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.camera_info_publisher_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.is_camera_info_publisher_running = True
            self.log_global("Camera info publisher đã được launch (equirectangular)", "[CameraInfoPublisher] ")
            
            # Start thread để monitor process
            threading.Thread(target=self.monitor_camera_info_publisher_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_launch_camera_info_publisher", "Không thể launch camera_info_publisher: {error}").replace("{error}", str(e))
            )
            self.log_global(f"Lỗi: {e}", "[CameraInfoPublisher] ")
    
    def stop_camera_info_publisher(self):
        """Stop camera_info_publisher node"""
        if self.camera_info_publisher_process:
            try:
                self.camera_info_publisher_process.terminate()
                self.camera_info_publisher_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_info_publisher_process.kill()
            except Exception as e:
                self.log_record(f"Lỗi khi dừng camera_info_publisher: {e}")
            
            self.camera_info_publisher_process = None
            self.is_camera_info_publisher_running = False
            self.log_global("Camera info publisher đã dừng", "[CameraInfoPublisher] ")
    
    def monitor_camera_info_publisher_process(self):
        """Monitor camera_info_publisher process"""
        if not self.camera_info_publisher_process:
            return
        
        for line in iter(self.camera_info_publisher_process.stdout.readline, ''):
            if not line:
                break
            # Log important messages
            if "error" in line.lower() or "warn" in line.lower() or "info" in line.lower():
                self.log_global(line.strip(), "[CameraInfoPublisher] ")
        
        if self.camera_info_publisher_process.poll() is not None:
            self.is_camera_info_publisher_running = False
            self.after(0, partial(self._update_camera_info_publisher_stopped))
    
    def _update_camera_info_publisher_stopped(self):
        """Helper function để update UI sau khi camera_info_publisher dừng"""
        try:
            self.log_global("Camera info publisher đã dừng", "[CameraInfoPublisher] ")
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    # ========== Preprocessing Methods ==========
    
    def browse_preprocess_input(self):
        """Browse cho preprocess input directory"""
        directory = filedialog.askdirectory(
            title=self.translator.get("calibration.dialog.choose_bags_directory", "Chọn thư mục chứa bags"),
            initialdir=self.preprocess_input_var.get() or str(self.workspace_path)
        )
        if directory:
            self.preprocess_input_var.set(directory)
    
    def browse_preprocess_output(self):
        """Browse cho preprocess output directory"""
        directory = filedialog.askdirectory(
            title=self.translator.get("calibration.dialog.choose_output_directory", "Chọn thư mục output"),
            initialdir=self.preprocess_output_var.get() or str(self.workspace_path)
        )
        if directory:
            self.preprocess_output_var.set(directory)
    
    def start_preprocess(self):
        """Chạy preprocessing"""
        input_dir = self.preprocess_input_var.get()
        output_dir = self.preprocess_output_var.get()
        
        if not input_dir or not Path(input_dir).exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_select_valid_input", "Vui lòng chọn thư mục input hợp lệ")
            )
            return
        
        if not output_dir:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_select_output", "Vui lòng chọn thư mục output")
            )
            return
        
        # Tạo output directory nếu chưa có
        Path(output_dir).mkdir(parents=True, exist_ok=True)
        
        # Build command
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.setup_not_found", "Không tìm thấy setup.sh tại: {path}").replace("{path}", str(setup_script))
            )
            return
        
        cmd_parts = ["ros2", "run", "direct_visual_lidar_calibration", "preprocess"]
        
        # Add camera model parameter
        camera_model = self.camera_model_var.get()
        if camera_model:
            cmd_parts.extend(["--camera_model", camera_model])
        
        if self.auto_topic_var.get():
            cmd_parts.append("-a")
        if self.dynamic_lidar_var.get():
            cmd_parts.append("-d")
        if self.visualize_var.get():
            cmd_parts.append("-v")
        
        cmd_parts.extend([input_dir, output_dir])
        
        # Build command với đầy đủ ROS2 environment
        # Cần source cả ROS2 base và workspace setup
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        cmd = f"source {ros2_setup} && source {setup_script} && {' '.join(cmd_parts)}"
        
        self.log_preprocess(f"Bắt đầu preprocessing...")
        self.log_preprocess(f"Input: {input_dir}")
        self.log_preprocess(f"Output: {output_dir}")
        self.log_preprocess(f"Camera model: {camera_model}")
        self.log_preprocess(f"Command: {' '.join(cmd_parts)}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            # Đảm bảo không có ROS_DOMAIN_ID conflict
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.preprocess_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.preprocess_btn.config(state=tk.DISABLED)
            self.status_label.config(text=self.translator.get("calibration.status.preprocessing", "Trạng thái: Đang preprocessing..."), foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_preprocess_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_run_preprocessing", "Không thể chạy preprocessing: {error}").replace("{error}", str(e))
            )
            self.log_preprocess(f"Lỗi: {e}")
    
    def monitor_preprocess_process(self):
        """Monitor preprocess process output"""
        if not self.preprocess_process:
            return
        
        try:
            for line in iter(self.preprocess_process.stdout.readline, ''):
                if not line:
                    break
                self.log_preprocess(line.strip())
        except Exception as e:
            self.log_preprocess(f"Lỗi khi đọc output: {e}")
        
        if self.preprocess_process.poll() is not None:
            exit_code = self.preprocess_process.poll()
            if exit_code == 0:
                # Sử dụng helper functions thay vì lambda để tránh closure issues
                self.after(0, partial(self._update_preprocess_success))
            else:
                self.after(0, partial(self._update_preprocess_failure))
    
    def _update_preprocess_success(self):
        """Helper function để update UI sau khi preprocessing thành công"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.preprocessing_completed", "Trạng thái: Preprocessing hoàn thành"), foreground="green")
            self.preprocess_btn.config(state=tk.NORMAL)
            # Auto-fill initial guess directory
            output_dir = self.preprocess_output_var.get()
            self.initial_guess_dir_var.set(output_dir)
            self.calibrate_dir_var.set(output_dir)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_preprocess_failure(self):
        """Helper function để update UI sau khi preprocessing thất bại"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.preprocessing_failed", "Trạng thái: Preprocessing thất bại"), foreground="red")
            self.preprocess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_preprocess(self, message):
        """Log message vào preprocess log và log tổng (thread-safe)"""
        self.log_global(message, "[Preprocessing] ")
        # Sử dụng after để đảm bảo thread-safe
        self.after(0, partial(self._log_preprocess_impl, message))
    
    def _log_preprocess_impl(self, message):
        """Implementation của log_preprocess (chạy trong main thread)"""
        try:
            self.preprocess_log.config(state=tk.NORMAL)
            self.preprocess_log.insert(tk.END, f"{message}\n")
            self.preprocess_log.see(tk.END)
            self.preprocess_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Initial Guess Methods ==========
    
    def browse_initial_guess_dir(self):
        """Browse cho initial guess directory"""
        directory = filedialog.askdirectory(
            title=self.translator.get("calibration.dialog.choose_preprocessed_directory", "Chọn thư mục preprocessed"),
            initialdir=self.initial_guess_dir_var.get() or str(self.workspace_path)
        )
        if directory:
            self.initial_guess_dir_var.set(directory)
    
    def start_initial_guess(self):
        """Chạy initial guess"""
        preprocessed_dir = self.initial_guess_dir_var.get()
        
        if not preprocessed_dir or not Path(preprocessed_dir).exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_select_valid_preprocessed", "Vui lòng chọn thư mục preprocessed hợp lệ")
            )
            return
        
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.setup_not_found", "Không tìm thấy setup.sh tại: {path}").replace("{path}", str(setup_script))
            )
            return
        
        mode = self.initial_guess_mode.get()
        
        if mode == "manual":
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            cmd = f"source {ros2_setup} && source {setup_script} && ros2 run direct_visual_lidar_calibration initial_guess_manual {preprocessed_dir}"
        else:  # auto
            # Cần chạy find_matches_superglue trước
            messagebox.showinfo(
                self.translator.get("dialog.info", "Information"),
                self.translator.get("calibration.message.automatic_mode_needs_superglue", "Automatic mode cần SuperGlue. Vui lòng chạy find_matches_superglue trước.")
            )
            return
        
        self.log_initial_guess(f"Bắt đầu initial guess ({mode})...")
        self.log_initial_guess(f"Directory: {preprocessed_dir}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.initial_guess_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.initial_guess_btn.config(state=tk.DISABLED)
            self.status_label.config(text=self.translator.get("calibration.status.running_initial_guess", "Trạng thái: Đang chạy initial guess..."), foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_initial_guess_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_run_initial_guess", "Không thể chạy initial guess: {error}").replace("{error}", str(e))
            )
            self.log_initial_guess(f"Lỗi: {e}")
    
    def monitor_initial_guess_process(self):
        """Monitor initial guess process output"""
        if not self.initial_guess_process:
            return
        
        try:
            for line in iter(self.initial_guess_process.stdout.readline, ''):
                if not line:
                    break
                self.log_initial_guess(line.strip())
        except Exception as e:
            self.log_initial_guess(f"Lỗi khi đọc output: {e}")
        
        if self.initial_guess_process.poll() is not None:
            exit_code = self.initial_guess_process.poll()
            if exit_code == 0:
                self.after(0, partial(self._update_initial_guess_success))
            else:
                self.after(0, partial(self._update_initial_guess_failure))
    
    def _update_initial_guess_success(self):
        """Helper function để update UI sau khi initial guess thành công"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.initial_guess_completed", "Trạng thái: Initial guess hoàn thành"), foreground="green")
            self.initial_guess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_initial_guess_failure(self):
        """Helper function để update UI sau khi initial guess thất bại"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.initial_guess_failed", "Trạng thái: Initial guess thất bại"), foreground="red")
            self.initial_guess_btn.config(state=tk.NORMAL)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_initial_guess(self, message):
        """Log message vào initial guess log và log tổng (thread-safe)"""
        self.log_global(message, "[Initial Guess] ")
        self.after(0, partial(self._log_initial_guess_impl, message))
    
    def _log_initial_guess_impl(self, message):
        """Implementation của log_initial_guess (chạy trong main thread)"""
        try:
            self.initial_guess_log.config(state=tk.NORMAL)
            self.initial_guess_log.insert(tk.END, f"{message}\n")
            self.initial_guess_log.see(tk.END)
            self.initial_guess_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Calibration Methods ==========
    
    def browse_calibrate_dir(self):
        """Browse cho calibrate directory"""
        directory = filedialog.askdirectory(
            title=self.translator.get("calibration.dialog.choose_preprocessed_directory", "Chọn thư mục preprocessed"),
            initialdir=self.calibrate_dir_var.get() or str(self.workspace_path)
        )
        if directory:
            self.calibrate_dir_var.set(directory)
    
    def start_calibrate(self):
        """Chạy calibration"""
        preprocessed_dir = self.calibrate_dir_var.get()
        
        if not preprocessed_dir or not Path(preprocessed_dir).exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_select_valid_preprocessed", "Vui lòng chọn thư mục preprocessed hợp lệ")
            )
            return
        
        setup_script = self.workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.setup_not_found", "Không tìm thấy setup.sh tại: {path}").replace("{path}", str(setup_script))
            )
            return
        
        cmd_parts = ["ros2", "run", "direct_visual_lidar_calibration", "calibrate", preprocessed_dir]
        
        # Add registration type
        registration_type = self.registration_type_var.get()
        cmd_parts.extend(["--registration_type", registration_type])
        
        if self.auto_quit_var.get():
            cmd_parts.append("--auto_quit")
        if self.background_var.get():
            cmd_parts.append("--background")
        
        # Build command với đầy đủ ROS2 environment
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        cmd = f"source {ros2_setup} && source {setup_script} && {' '.join(cmd_parts)}"
        
        self.log_calibrate(f"Bắt đầu calibration...")
        self.log_calibrate(f"Directory: {preprocessed_dir}")
        
        try:
            # Sử dụng env để đảm bảo clean environment
            env = os.environ.copy()
            # Đảm bảo không có ROS_DOMAIN_ID conflict
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.calibrate_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.calibrate_btn.config(state=tk.DISABLED)
            self.stop_calibrate_btn.config(state=tk.NORMAL)
            self.status_label.config(text=self.translator.get("calibration.status.calibrating", "Trạng thái: Đang calibration..."), foreground="orange")
            
            # Start thread để đọc output
            threading.Thread(target=self.monitor_calibrate_process, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_run_calibration", "Không thể chạy calibration: {error}").replace("{error}", str(e))
            )
            self.log_calibrate(f"Lỗi: {e}")
    
    def monitor_calibrate_process(self):
        """Monitor calibrate process output"""
        if not self.calibrate_process:
            return
        
        try:
            for line in iter(self.calibrate_process.stdout.readline, ''):
                if not line:
                    break
                self.log_calibrate(line.strip())
        except Exception as e:
            self.log_calibrate(f"Lỗi khi đọc output: {e}")
        
        if self.calibrate_process.poll() is not None:
            exit_code = self.calibrate_process.poll()
            if exit_code == 0:
                self.after(0, partial(self._update_calibrate_success))
            else:
                self.after(0, partial(self._update_calibrate_failure))
    
    def stop_calibrate(self):
        """Dừng calibration process"""
        if self.calibrate_process:
            try:
                self.log_calibrate("Đang dừng calibration...")
                self.calibrate_process.terminate()
                self.calibrate_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    self.calibrate_process.kill()
                except:
                    pass
            except Exception as e:
                self.log_calibrate(f"Lỗi khi dừng calibration: {e}")
            finally:
                self.calibrate_process = None
                self.calibrate_btn.config(state=tk.NORMAL)
                self.stop_calibrate_btn.config(state=tk.DISABLED)
                self.status_label.config(text=self.translator.get("calibration.status.calibration_stopped", "Trạng thái: Calibration đã dừng"), foreground="orange")
                self.log_calibrate("Calibration đã dừng")
    
    def _update_calibrate_success(self):
        """Helper function để update UI sau khi calibration thành công"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.calibration_completed", "Trạng thái: Calibration hoàn thành"), foreground="green")
            self.calibrate_btn.config(state=tk.NORMAL)
            self.stop_calibrate_btn.config(state=tk.DISABLED)
            # Auto-fill calib.json path
            calib_json_path = Path(self.calibrate_dir_var.get()) / "calib.json"
            if calib_json_path.exists():
                self.calib_json_var.set(str(calib_json_path))
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def _update_calibrate_failure(self):
        """Helper function để update UI sau khi calibration thất bại"""
        try:
            self.status_label.config(text=self.translator.get("calibration.status.calibration_failed", "Trạng thái: Calibration thất bại"), foreground="red")
            self.calibrate_btn.config(state=tk.NORMAL)
            self.stop_calibrate_btn.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi update UI: {e}")
    
    def log_calibrate(self, message):
        """Log message vào calibrate log và log tổng (thread-safe)"""
        self.log_global(message, "[Calibration] ")
        self.after(0, partial(self._log_calibrate_impl, message))
    
    def _log_calibrate_impl(self, message):
        """Implementation của log_calibrate (chạy trong main thread)"""
        try:
            self.calibrate_log.config(state=tk.NORMAL)
            self.calibrate_log.insert(tk.END, f"{message}\n")
            self.calibrate_log.see(tk.END)
            self.calibrate_log.config(state=tk.DISABLED)
        except Exception as e:
            print(f"Lỗi khi log: {e}")
    
    # ========== Export Methods ==========
    
    def browse_calib_json(self):
        """Browse cho calib.json file"""
        file_path = filedialog.askopenfilename(
            title=self.translator.get("calibration.dialog.choose_calib_json", "Chọn file calib.json"),
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self.calib_json_var.get() or str(self.workspace_path)
        )
        if file_path:
            self.calib_json_var.set(file_path)
    
    def view_calib_json(self):
        """Xem nội dung calib.json"""
        calib_json_path = self.calib_json_var.get()
        
        if not calib_json_path or not Path(calib_json_path).exists():
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.please_select_valid_calib_json", "Vui lòng chọn file calib.json hợp lệ")
            )
            return
        
        try:
            with open(calib_json_path, 'r') as f:
                calib_data = json.load(f)
            
            self.results_text.config(state=tk.NORMAL)
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, json.dumps(calib_data, indent=2))
            self.results_text.config(state=tk.DISABLED)
            
        except Exception as e:
            messagebox.showerror(
                self.translator.get("dialog.error", "Error"),
                self.translator.get("calibration.message.cannot_read_file", "Không thể đọc file: {error}").replace("{error}", str(e))
            )
    
    def convert_to_fast_livo2(self):
        """Load calib từ fast_livo2_calib.yaml, ghi đè Rcl/Pcl và build"""
        self.convert_btn.config(state=tk.DISABLED)
        self._append_results_text("=== Bắt đầu convert và build ===", clear=True)
        self.after(0, lambda: self.status_label.config(text=self.translator.get("calibration.status.converting_building", "Trạng thái: Đang convert & build..."), foreground="orange"))
        threading.Thread(target=self._convert_and_build, daemon=True).start()

    def _convert_and_build(self):
        try:
            preprocessed_dir = self.workspace_path / "calibration_data" / "preprocessed"
            calib_path = preprocessed_dir / "fast_livo2_calib.yaml"
            calib_json_path = preprocessed_dir / "calib.json"
            config_path = self.workspace_path / "src" / "FAST-LIVO2" / "config" / "mid360_equirectangular_stable.yaml"
            
            # Kiểm tra và convert từ calib.json nếu fast_livo2_calib.yaml chưa tồn tại
            if not calib_path.exists():
                self._append_results_text(f"⚠️  Không tìm thấy file fast_livo2_calib.yaml, đang tìm calib.json để convert...")
                
                if not calib_json_path.exists():
                    raise FileNotFoundError(
                        f"Không tìm thấy file calib.json tại: {calib_json_path}\n"
                        f"Vui lòng chạy calibration trước để tạo file calib.json"
                    )
                
                # Tự động convert từ calib.json sang fast_livo2_calib.yaml
                convert_script = self.workspace_path / "src" / "FAST-LIVO2" / "scripts" / "convert_calib_to_fast_livo2.py"
                if not convert_script.exists():
                    raise FileNotFoundError(f"Không tìm thấy script conversion tại: {convert_script}")
                
                self._append_results_text(f"🔄 Đang convert từ calib.json sang fast_livo2_calib.yaml...")
                cmd = f"python3 {convert_script} {calib_json_path} --output {calib_path}"
                
                result = subprocess.run(
                    cmd,
                    shell=True,
                    capture_output=True,
                    text=True,
                    timeout=30
                )
                
                if result.returncode != 0:
                    raise RuntimeError(f"Convert thất bại:\n{result.stderr}")
                
                self._append_results_text(f"✅ Đã convert thành công: {calib_path}")
                self._append_results_text(result.stdout)
            
            if not config_path.exists():
                raise FileNotFoundError(f"Không tìm thấy file config: {config_path}")

            rcl_values, pcl_values = self._load_calib_values(calib_path)
            self._update_mid360_config(config_path, rcl_values, pcl_values)
            self._append_results_text("✅ Đã ghi đè Rcl/Pcl vào mid360_equirectangular_stable.yaml")

            build_output = self._run_build_script()
            self._append_results_text(build_output)

            self.after(0, lambda: messagebox.showinfo(
                self.translator.get("dialog.success", "Success"),
                self.translator.get("calibration.message.convert_build_completed", "Hoàn tất convert và build")
            ))
            self.after(0, lambda: self.status_label.config(text=self.translator.get("calibration.status.completed", "Trạng thái: Hoàn tất"), foreground="green"))
        except Exception as e:
            error_msg = f"Lỗi: {e}"
            self._append_results_text(error_msg)
            self.after(0, lambda: messagebox.showerror(self.translator.get("dialog.error", "Error"), error_msg))
            self.after(0, lambda: self.status_label.config(text=self.translator.get("calibration.status.error", "Trạng thái: Lỗi"), foreground="red"))
        finally:
            self.after(0, lambda: self.convert_btn.config(state=tk.NORMAL))

    def _format_num(self, value):
        """Định dạng số gọn gàng"""
        try:
            return f"{float(value):.8f}".rstrip("0").rstrip(".")
        except Exception:
            return str(value)

    def _load_calib_values(self, calib_path: Path):
        """Đọc Rcl/Pcl từ fast_livo2_calib.yaml"""
        with open(calib_path, "r") as f:
            data = yaml.safe_load(f) or {}
        extrin = data.get("extrin_calib", {})
        rcl = extrin.get("Rcl")
        pcl = extrin.get("Pcl")
        if not rcl or not pcl:
            raise ValueError("Thiếu Rcl/Pcl trong fast_livo2_calib.yaml")
        if len(rcl) != 9 or len(pcl) != 3:
            raise ValueError("Rcl phải có 9 phần tử và Pcl phải có 3 phần tử")
        return rcl, pcl

    def _update_mid360_config(self, config_path: Path, rcl_values, pcl_values):
        """Ghi đè block Rcl/Pcl trong file config"""
        with open(config_path, "r") as f:
            content = f.read()

        rcl_formatted = [self._format_num(v) for v in rcl_values]
        pcl_formatted = [self._format_num(v) for v in pcl_values]

        rcl_pattern = r"(?P<indent>\s*)Rcl:\s*\[.*?\n\s*.*?\n\s*.*?\]"
        pcl_pattern = r"(?P<indent>\s*)Pcl:\s*\[.*?\]"
        pcl_comment_pattern = r"\n\s*# Pcl: translation vector from lidar to camera"

        rcl_match = re.search(rcl_pattern, content, flags=re.DOTALL)
        pcl_match = re.search(pcl_pattern, content, flags=re.DOTALL)

        if not rcl_match:
            raise ValueError("Không tìm thấy block Rcl trong file config")
        if not pcl_match:
            raise ValueError("Không tìm thấy Pcl trong file config")

        rcl_indent = rcl_match.group("indent")
        pcl_indent = pcl_match.group("indent")
        cont_indent = rcl_indent + "      "  # tăng thêm 6 spaces cho dòng tiếp

        new_rcl_block = (
            f"{rcl_indent}Rcl: [{rcl_formatted[0]}, {rcl_formatted[1]}, {rcl_formatted[2]},\n"
            f"{cont_indent}{rcl_formatted[3]}, {rcl_formatted[4]}, {rcl_formatted[5]},\n"
            f"{cont_indent}{rcl_formatted[6]}, {rcl_formatted[7]}, {rcl_formatted[8]}]"
        )
        new_pcl_line = f"{pcl_indent}Pcl: [{pcl_formatted[0]}, {pcl_formatted[1]}, {pcl_formatted[2]}]"

        content = re.sub(rcl_pattern, new_rcl_block, content, count=1, flags=re.DOTALL)
        content = re.sub(pcl_pattern, new_pcl_line, content, count=1, flags=re.DOTALL)
        content = re.sub(pcl_comment_pattern, f"\n{pcl_indent}# Pcl: translation vector from lidar to camera", content, count=1)
        # Loại bỏ dòng trống giữa Rcl và Pcl
        content = re.sub(
            r"\]\n\s*\n(\s*# Pcl: translation vector from lidar to camera)",
            r"]\n\1",
            content,
            count=1
        )
        # Loại bỏ dòng trống sau Pcl (trước key kế tiếp)
        content = re.sub(
            r"(Pcl:\s*\[.*?\])\n\s*\n",
            r"\1\n",
            content,
            count=1,
            flags=re.DOTALL
        )

        with open(config_path, "w") as f:
            f.write(content)

    def _run_build_script(self):
        """Chạy build.sh và trả về log"""
        build_script = self.workspace_path.parent / "build.sh"
        if not build_script.exists():
            raise FileNotFoundError(f"Không tìm thấy build.sh tại: {build_script}")

        cmd = f"cd {build_script.parent} && printf '\\n' | bash {build_script.name}"
        self._append_results_text(f"▶️ Running: {cmd}")

        process = subprocess.Popen(
            cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1
        )

        logs = []
        if process.stdout:
            for line in process.stdout:
                line = line.rstrip("\n")
                logs.append(line)
                self._append_results_text(line)

        return_code = process.wait()
        summary = f"✅ Build thành công" if return_code == 0 else f"❌ Build thất bại (exit {return_code})"
        return "\n".join(logs + [summary])
    
    def start_converter(self):
        """Tự động khởi động Converter khi mở GUI"""
        if self.is_converter_running:
            return
        
        workspace_path = self.workspace_path
        launch_file = Path("src/livox_msg_converter/launch/livox_msg_converter.launch.py")
        
        setup_script = workspace_path / "install" / "setup.sh"
        if not setup_script.exists():
            print(f"Cảnh báo: Không tìm thấy setup.sh tại: {setup_script}")
            return
        
        launch_path = workspace_path / launch_file
        if not launch_path.exists():
            print(f"Cảnh báo: Không tìm thấy launch file tại: {launch_path}")
            return
        
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        if not Path(ros2_setup).exists():
            print(f"Cảnh báo: Không tìm thấy ROS2 setup tại: {ros2_setup}")
            return
        
        # Xác định package name
        parts = launch_file.parts
        if len(parts) >= 2 and parts[0] == "src":
            package_name = parts[1]
        else:
            package_name = launch_path.parent.parent.name
        
        launch_filename = launch_path.name
        cmd = f"source {ros2_setup} && source {setup_script} && ros2 launch {package_name} {launch_filename}"
        
        self.log_global("🚀 Đang khởi động Converter...", "[Converter] ")
        
        try:
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.converter_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            self.is_converter_running = True
            self.log_global("✓ Converter đã được khởi động", "[Converter] ")
            
            # Start thread để monitor (không cần log, chỉ để biết khi nào dừng)
            threading.Thread(target=self._monitor_converter_process, daemon=True).start()
            
        except Exception as e:
            self.log_global(f"❌ Lỗi khi khởi động Converter: {e}", "[Converter] ")
    
    def _monitor_converter_process(self):
        """Monitor converter process (silent)"""
        proc = self.converter_process
        if not proc:
            return

        try:
            for line in iter(proc.stdout.readline, ''):
                if not line:
                    break
                # Không log, chỉ monitor
                pass
        except Exception:
            pass
        
        # Kiểm tra exit code
        if proc.poll() is not None:
            self.is_converter_running = False
    
    def stop_converter(self):
        """Dừng Converter"""
        if not self.is_converter_running or not self.converter_process:
            return
        
        try:
            self.converter_process.terminate()
            try:
                self.converter_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.converter_process.kill()
                self.converter_process.wait()
        except Exception as e:
            print(f"Lỗi khi dừng Converter: {e}")
        
        self.converter_process = None
        self.is_converter_running = False
        self.log_global("✓ Converter đã dừng", "[Converter] ")
    
    def cleanup_all_processes(self):
        """Dừng tất cả processes khi đóng GUI"""
        # Dừng Converter
        if self.is_converter_running:
            self.stop_converter()
        
        # Dừng Replay Calibration
        if hasattr(self, "replay_calibration_tab") and self.replay_calibration_tab and self.replay_calibration_tab.is_replaying:
            self.replay_calibration_tab.stop_replay()

        # Dừng Camera Info Publisher
        if self.is_camera_info_publisher_running and self.camera_info_publisher_process:
            self.stop_camera_info_publisher()
        
        # Dừng Preprocessing
        if self.preprocess_process:
            try:
                self.preprocess_process.terminate()
                self.preprocess_process.wait(timeout=2)
            except:
                try:
                    self.preprocess_process.kill()
                except:
                    pass
            self.preprocess_process = None
        
        # Dừng Initial Guess
        if self.initial_guess_process:
            try:
                self.initial_guess_process.terminate()
                self.initial_guess_process.wait(timeout=2)
            except:
                try:
                    self.initial_guess_process.kill()
                except:
                    pass
            self.initial_guess_process = None
        
        # Dừng Calibration
        if self.calibrate_process:
            try:
                self.calibrate_process.terminate()
                self.calibrate_process.wait(timeout=2)
            except:
                try:
                    self.calibrate_process.kill()
                except:
                    pass
            self.calibrate_process = None

