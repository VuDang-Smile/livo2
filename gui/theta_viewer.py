#!/usr/bin/env python3
"""
Multi-Tab GUI Viewer
GUI với 2 tabs: Theta Driver và Livox Driver 2
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

# Import rclpy nhưng không khởi tạo ngay
# Chỉ khởi tạo khi cần (trong các tabs)
try:
    import rclpy
except ImportError:
    rclpy = None
    print("Cảnh báo: rclpy không thể import. Một số tính năng có thể không hoạt động.")

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
from calibration_tab import CalibrationTab
from mapping_tab import MappingTab
from recording_tab import RecordingTab
from replay_tab import ReplayTab
from bag_mapping_tab import BagMappingTab
from pcd_viewer_tab import PCDViewerTab


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
        
        # Tạo tab Calibration
        self.calibration_tab = CalibrationTab(self.notebook)
        self.notebook.add(self.calibration_tab, text="Calibration")
        
        # Tạo tab Mapping
        self.mapping_tab = MappingTab(self.notebook)
        self.notebook.add(self.mapping_tab, text="Mapping")
        
        # Tạo tab Recording
        self.recording_tab = RecordingTab(self.notebook)
        self.notebook.add(self.recording_tab, text="Recording")
        
        # Tạo tab Replay
        self.replay_tab = ReplayTab(self.notebook)
        self.notebook.add(self.replay_tab, text="Replay")
        
        # Tạo tab Bag Mapping
        self.bag_mapping_tab = BagMappingTab(self.notebook)
        self.notebook.add(self.bag_mapping_tab, text="Bag Mapping")
        
        # Tạo tab PCD Viewer
        self.pcd_viewer_tab = PCDViewerTab(self.notebook)
        self.notebook.add(self.pcd_viewer_tab, text="PCD Viewer")
        
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
        
        # Dừng các process trong calibration tab
        if hasattr(self.calibration_tab, 'stop_record'):
            if self.calibration_tab.is_recording:
                self.calibration_tab.stop_record()
        
        # Dừng mapping nếu đang chạy
        if hasattr(self.mapping_tab, 'stop_mapping'):
            if self.mapping_tab.is_mapping_running:
                self.mapping_tab.stop_mapping()
        if hasattr(self.mapping_tab, 'stop_rviz'):
            if self.mapping_tab.is_rviz_running:
                self.mapping_tab.stop_rviz()
        
        # Dừng recording nếu đang chạy
        if hasattr(self.recording_tab, 'stop_recording'):
            if self.recording_tab.is_recording:
                self.recording_tab.stop_recording()
        
        # Dừng replay nếu đang chạy
        if hasattr(self.replay_tab, 'stop_replay'):
            if self.replay_tab.is_replaying:
                self.replay_tab.stop_replay()
        
        # Dừng bag mapping nếu đang chạy
        if hasattr(self.bag_mapping_tab, 'stop_mapping'):
            if self.bag_mapping_tab.is_mapping_running or self.bag_mapping_tab.is_bag_playing:
                self.bag_mapping_tab.stop_mapping()
        
        # Dừng PCD viewer nếu đang chạy
        if hasattr(self.pcd_viewer_tab, 'stop_viewer'):
            if self.pcd_viewer_tab.is_viewer_running:
                self.pcd_viewer_tab.stop_viewer()
        
        # Shutdown ROS nếu đã được khởi tạo
        if rclpy and rclpy.ok():
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
