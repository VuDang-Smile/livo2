#!/usr/bin/env python3
"""
Multi-Tab GUI Viewer
GUI với 2 tabs: Theta Driver và Livox Driver 2
"""

import os
import sys

import rclpy

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as e:
    print(f"Lỗi import: {e}")
    print("Vui lòng cài đặt: pip install pillow")
    sys.exit(1)

# Import các tab modules
from theta_tab import ThetaTab
from livox_tab import LivoxTab


class MainGUI:
    """GUI chính với multi-tab interface"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Tab ROS2 Viewer - Theta & Livox")
        self.root.geometry("1600x900")
        
        # Tạo notebook cho tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Tạo tab Theta
        self.theta_tab = ThetaTab(self.notebook)
        self.notebook.add(self.theta_tab, text="Theta Driver")
        
        # Tạo tab Livox
        self.livox_tab = LivoxTab(self.notebook)
        self.notebook.add(self.livox_tab, text="Livox Driver 2")
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def on_closing(self):
        """Xử lý khi đóng window"""
        # Dừng tất cả trong các tabs
        if hasattr(self.theta_tab, 'stop_all'):
            self.theta_tab.stop_all()
        
        if hasattr(self.livox_tab, 'stop_livox_driver'):
            self.livox_tab.stop_livox_driver()
        if hasattr(self.livox_tab, 'stop_ros_subscriber'):
            self.livox_tab.stop_ros_subscriber()
        
        # Shutdown ROS
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        
        self.root.destroy()


def main():
    """Hàm main"""
    # Kiểm tra ROS2 environment
    if 'ROS_DISTRO' not in os.environ:
        print("Cảnh báo: ROS2 environment chưa được source")
        print("Vui lòng chạy: source /opt/ros/jazzy/setup.bash")
    
    # Tạo GUI
    root = tk.Tk()
    app = MainGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
