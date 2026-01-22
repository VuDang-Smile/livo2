#!/usr/bin/env python3
"""
Main entry point cho Calibration Standalone GUI
Tự động khởi động Converter khi mở GUI
"""

import os
import sys
from pathlib import Path

# Thêm project root vào sys.path để có thể import các module
project_root = Path(__file__).parent.parent
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

# Fix Qt plugin issue - set environment variables trước khi import bất kỳ module nào
# Disable Qt plugin path từ OpenCV để tránh xung đột
if 'QT_PLUGIN_PATH' in os.environ:
    paths = os.environ['QT_PLUGIN_PATH'].split(':')
    # Loại bỏ các path chứa cv2 hoặc opencv
    paths = [p for p in paths if 'cv2' not in p and 'opencv' not in p.lower()]
    if paths:
        os.environ['QT_PLUGIN_PATH'] = ':'.join(paths)
    else:
        os.environ.pop('QT_PLUGIN_PATH', None)

# Set QT_QPA_PLATFORM_PLUGIN_PATH nếu cần
if 'QT_QPA_PLATFORM_PLUGIN_PATH' not in os.environ:
    # Để trống hoặc set về system Qt plugins
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as e:
    print(f"Lỗi import: {e}")
    print("Vui lòng cài đặt: pip install pillow")
    sys.exit(1)

# Import GUI
from calibration_standalone_gui import CalibrationStandaloneGUI
from languages.translate_engine import Translator


class CalibrationStandaloneApp:
    """Application class cho Calibration Standalone GUI"""
    
    def __init__(self, root):
        self.root = root
        
        # Translator for multi-language support
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        self.root.title(self.translator.get("title.calibration_standalone_gui", "Calibration Standalone GUI"))
        self.root.geometry("1600x900")
        
        # Setup language button FIRST (before main container)
        self.setup_language_button()
        
        # Tạo GUI
        self.gui = CalibrationStandaloneGUI(self.root, translator=self.translator)
        self.gui.pack(fill=tk.BOTH, expand=True)
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Update UI texts after all components are set up
        self.update_ui_texts()
    
    def setup_language_button(self):
        """Setup language toggle button"""
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
        """Toggle between English and Japanese"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """Change language and update UI"""
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        """Update all UI texts based on current language"""
        self.root.title(self.translator.get("title.calibration_standalone_gui", "Calibration Standalone GUI"))
        
        # Update GUI texts if it has update_ui_texts method
        if hasattr(self.gui, 'update_ui_texts'):
            self.gui.update_ui_texts()
    
    def on_closing(self):
        """Xử lý khi đóng window"""
        # Dừng tất cả processes
        if hasattr(self.gui, 'cleanup_all_processes'):
            self.gui.cleanup_all_processes()
        
        self.root.destroy()


def main():
    """Hàm main"""
    # Kiểm tra ROS2 environment
    if 'ROS_DISTRO' not in os.environ:
        print("Cảnh báo: ROS2 environment chưa được source")
        print("Vui lòng chạy: source /opt/ros/jazzy/setup.bash")
    
    # Tạo GUI
    root = tk.Tk()
    app = CalibrationStandaloneApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()

