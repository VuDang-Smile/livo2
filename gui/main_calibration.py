#!/usr/bin/env python3
"""
Main entry point cho Calibration Standalone GUI
Tự động khởi động Converter khi mở GUI
"""

import os
import sys

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


class CalibrationStandaloneApp:
    """Application class cho Calibration Standalone GUI"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Calibration Standalone GUI")
        self.root.geometry("1600x900")
        
        # Tạo GUI
        self.gui = CalibrationStandaloneGUI(self.root)
        self.gui.pack(fill=tk.BOTH, expand=True)
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
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

