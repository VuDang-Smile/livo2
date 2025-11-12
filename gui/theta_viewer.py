#!/usr/bin/env python3
"""
Theta Driver GUI Viewer
GUI để mở theta_driver và hiển thị ảnh từ topic /image_raw
"""

import sys
import threading
import subprocess
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    from PIL import Image as PILImage
    from PIL import ImageTk
except ImportError as e:
    print(f"Lỗi import: {e}")
    print("Vui lòng cài đặt: pip install pillow")
    sys.exit(1)


class ImageSubscriber(Node):
    """ROS2 Node để subscribe topic /image_raw"""
    
    def __init__(self, callback):
        super().__init__('theta_viewer_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.callback = callback
        self.get_logger().info('Đã subscribe topic /image_raw')
    
    def image_callback(self, msg):
        """Callback khi nhận được ảnh mới"""
        try:
            # Log để debug (chỉ log mỗi 30 frame để không spam)
            if not hasattr(self, '_frame_count'):
                self._frame_count = 0
            self._frame_count += 1
            if self._frame_count % 30 == 0:
                self.get_logger().info(f'Đã nhận {self._frame_count} frames')
            
            # Chuyển đổi từ ROS Image sang OpenCV Mat
            # Encoding từ theta_driver là rgb8, giữ nguyên RGB
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # cv_image đã là RGB, truyền trực tiếp
            self.callback(cv_image)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class ThetaViewerGUI:
    """GUI chính để hiển thị ảnh từ theta_driver"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Theta Driver Viewer")
        self.root.geometry("1200x800")
        
        # Biến trạng thái
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.theta_driver_process = None
        self.current_image = None
        self.is_ros_running = False
        self.frame_count = 0
        
        # Tạo UI
        self.create_widgets()
        
        # Bind events
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def create_widgets(self):
        """Tạo các widget cho GUI"""
        
        # Frame điều khiển
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Nút Launch Theta Driver
        self.launch_btn = ttk.Button(
            control_frame,
            text="Launch Theta Driver",
            command=self.launch_theta_driver
        )
        self.launch_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start ROS Subscriber
        self.start_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_ros_subscriber,
            state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop
        self.stop_btn = ttk.Button(
            control_frame,
            text="Stop",
            command=self.stop_all,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Chưa kết nối",
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame hiển thị ảnh
        image_frame = ttk.Frame(self.root, padding="10")
        image_frame.pack(fill=tk.BOTH, expand=True)
        
        # Canvas để hiển thị ảnh
        self.canvas = tk.Canvas(
            image_frame,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbar (nếu cần)
        scrollbar_v = ttk.Scrollbar(image_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        scrollbar_h = ttk.Scrollbar(image_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.canvas.configure(yscrollcommand=scrollbar_v.set, xscrollcommand=scrollbar_h.set)
        
        # Label thông tin ảnh
        info_frame = ttk.Frame(self.root, padding="5")
        info_frame.pack(fill=tk.X)
        
        self.info_label = ttk.Label(
            info_frame,
            text="Chưa có ảnh",
            font=("Arial", 10)
        )
        self.info_label.pack()
        
    def launch_theta_driver(self):
        """Launch theta_driver node"""
        try:
            # Kiểm tra xem ROS2 đã được source chưa
            workspace_path = Path(__file__).parent.parent / "ws"
            setup_script = workspace_path / "install" / "setup.sh"
            
            if not setup_script.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy setup.sh tại: {setup_script}\n"
                    "Vui lòng build workspace trước."
                )
                return
            
            # Source setup.sh và chạy theta_driver_node
            cmd = f"source {setup_script} && ros2 run theta_driver theta_driver_node"
            
            self.theta_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.status_label.config(
                text="Trạng thái: Theta Driver đang chạy",
                foreground="orange"
            )
            self.launch_btn.config(state=tk.DISABLED)
            self.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.root.after(2000, self.check_theta_driver_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch theta_driver: {e}")
    
    def check_theta_driver_process(self):
        """Kiểm tra xem theta_driver process còn chạy không"""
        if self.theta_driver_process:
            if self.theta_driver_process.poll() is not None:
                # Process đã dừng
                self.status_label.config(
                    text="Trạng thái: Theta Driver đã dừng",
                    foreground="red"
                )
                self.launch_btn.config(state=tk.NORMAL)
                self.start_btn.config(state=tk.DISABLED)
                if self.is_ros_running:
                    self.stop_all()
    
    def start_ros_subscriber(self):
        """Bắt đầu ROS subscriber"""
        if self.is_ros_running:
            return
        
        try:
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                print("Khởi tạo ROS2...")
                rclpy.init()
            
            # Tạo node subscriber
            print("Tạo ROS2 node...")
            self.ros_node = ImageSubscriber(self.on_image_received)
            
            # Tạo executor và add node
            print("Tạo executor...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topic có tồn tại không
            print("Kiểm tra topic /image_raw...")
            import subprocess
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if '/image_raw' in result.stdout:
                print("✓ Topic /image_raw đã tồn tại")
            else:
                print("⚠️  Cảnh báo: Topic /image_raw chưa tồn tại")
                print("Các topics có sẵn:")
                print(result.stdout)
            
            # Chạy ROS trong thread riêng
            print("Khởi động ROS thread...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_ros_running = True
            self.frame_count = 0
            self.status_label.config(
                text="Trạng thái: Đang subscribe /image_raw...",
                foreground="green"
            )
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            print("✓ ROS subscriber đã khởi động")
            
        except Exception as e:
            error_msg = f"Không thể start subscriber: {e}"
            print(f"✗ Lỗi: {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Lỗi", error_msg)
    
    def ros_spin(self):
        """Spin ROS node trong thread riêng"""
        try:
            print("ROS spin thread đã bắt đầu")
            while rclpy.ok() and self.is_ros_running:
                # Dùng executor để spin
                if self.ros_executor is not None:
                    self.ros_executor.spin_once(timeout_sec=0.1)
                else:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            error_msg = f"Lỗi trong ROS spin: {e}"
            print(f"✗ {error_msg}")
            import traceback
            traceback.print_exc()
            # Cập nhật UI để báo lỗi
            if self.is_ros_running:
                self.root.after(0, lambda: self.status_label.config(
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
    
    def on_image_received(self, cv_image):
        """Callback khi nhận được ảnh từ ROS"""
        self.current_image = cv_image
        self.frame_count += 1
        
        # Cập nhật UI trong main thread
        self.root.after(0, self.update_image_display, cv_image)
        
        # Cập nhật status mỗi 30 frame
        if self.frame_count % 30 == 0:
            self.root.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.frame_count} frames",
                foreground="green"
            ))
    
    def update_image_display(self, cv_image):
        """Cập nhật hiển thị ảnh trên canvas"""
        try:
            # Lấy kích thước canvas
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                # Canvas chưa được render, thử lại sau
                self.root.after(100, lambda: self.update_image_display(cv_image))
                return
            
            # Resize ảnh để vừa với canvas (giữ tỷ lệ)
            img_height, img_width = cv_image.shape[:2]
            
            # Tính toán kích thước mới
            scale = min(canvas_width / img_width, canvas_height / img_height)
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            # Resize ảnh
            resized = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            # cv_image đã là RGB từ theta_driver, không cần convert
            # Chuyển đổi sang PIL Image (PIL cũng dùng RGB)
            pil_image = PILImage.fromarray(resized)
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Xóa canvas cũ và vẽ ảnh mới
            self.canvas.delete("all")
            self.canvas.create_image(
                canvas_width // 2,
                canvas_height // 2,
                image=photo,
                anchor=tk.CENTER
            )
            
            # Lưu reference để tránh garbage collection
            self.canvas.image = photo
            
            # Cập nhật thông tin
            self.info_label.config(
                text=f"Kích thước: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f}"
            )
            
        except Exception as e:
            print(f"Lỗi cập nhật hiển thị: {e}")
    
    def stop_all(self):
        """Dừng tất cả"""
        # Dừng ROS subscriber
        self.is_ros_running = False
        
        if self.ros_executor:
            try:
                self.ros_executor.shutdown()
            except:
                pass
            self.ros_executor = None
        
        if self.ros_node:
            try:
                self.ros_node.destroy_node()
            except:
                pass
            self.ros_node = None
        
        # Dừng theta_driver process
        if self.theta_driver_process:
            try:
                self.theta_driver_process.terminate()
                self.theta_driver_process.wait(timeout=5)
            except:
                try:
                    self.theta_driver_process.kill()
                except:
                    pass
            self.theta_driver_process = None
        
        # Cập nhật UI
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="red"
        )
        self.launch_btn.config(state=tk.NORMAL)
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        
        # Xóa ảnh
        self.canvas.delete("all")
        self.info_label.config(text="Chưa có ảnh")
    
    def on_closing(self):
        """Xử lý khi đóng window"""
        self.stop_all()
        
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
    app = ThetaViewerGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()

