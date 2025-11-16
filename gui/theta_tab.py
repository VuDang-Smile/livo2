#!/usr/bin/env python3
"""
Theta Tab Module
Chứa ThetaTab và ImageSubscriber cho Theta Driver
"""

import os
# Fix Qt plugin issue với OpenCV
# Disable Qt backend của OpenCV để tránh xung đột với Qt plugins
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''
# Hoặc có thể unset QT_PLUGIN_PATH nếu nó đang trỏ đến cv2/qt/plugins
if 'QT_PLUGIN_PATH' in os.environ:
    # Loại bỏ cv2/qt/plugins khỏi QT_PLUGIN_PATH
    paths = os.environ['QT_PLUGIN_PATH'].split(':')
    paths = [p for p in paths if 'cv2' not in p and 'opencv' not in p.lower()]
    if paths:
        os.environ['QT_PLUGIN_PATH'] = ':'.join(paths)
    else:
        os.environ.pop('QT_PLUGIN_PATH', None)

import threading
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# Set OpenCV backend để không dùng Qt
os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '0'
import cv2
# Disable Qt GUI backend của OpenCV nếu có
try:
    cv2.setNumThreads(1)  # Giảm thread để tránh xung đột
except:
    pass

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    from PIL import Image as PILImage
    from PIL import ImageTk
except ImportError as e:
    print(f"Lỗi import: {e}")
    print("Vui lòng cài đặt: pip install pillow")
    import sys
    sys.exit(1)


class ImageSubscriber(Node):
    """ROS2 Node để subscribe topic /image_raw và /image_perspective"""
    
    def __init__(self, callback_raw, callback_perspective):
        super().__init__('theta_viewer_subscriber')
        
        # Subscribe topic equirectangular
        self.subscription_raw = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback_raw,
            10
        )
        
        # Subscribe topic perspective
        self.subscription_perspective = self.create_subscription(
            Image,
            'image_perspective',
            self.image_callback_perspective,
            10
        )
        
        self.bridge = CvBridge()
        self.callback_raw = callback_raw
        self.callback_perspective = callback_perspective
        self.get_logger().info('Đã subscribe topics /image_raw và /image_perspective')
    
    def image_callback_raw(self, msg):
        """Callback khi nhận được ảnh equirectangular"""
        try:
            # Log để debug (chỉ log mỗi 30 frame để không spam)
            if not hasattr(self, '_frame_count_raw'):
                self._frame_count_raw = 0
            self._frame_count_raw += 1
            if self._frame_count_raw % 30 == 0:
                self.get_logger().info(f'Đã nhận {self._frame_count_raw} frames từ /image_raw')
            
            # Chuyển đổi từ ROS Image sang OpenCV Mat
            # Encoding từ theta_driver là rgb8, giữ nguyên RGB
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            # cv_image đã là RGB, truyền trực tiếp
            self.callback_raw(cv_image)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh equirectangular: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def image_callback_perspective(self, msg):
        """Callback khi nhận được ảnh perspective"""
        try:
            # Log để debug (chỉ log mỗi 30 frame để không spam)
            if not hasattr(self, '_frame_count_perspective'):
                self._frame_count_perspective = 0
            self._frame_count_perspective += 1
            if self._frame_count_perspective % 30 == 0:
                self.get_logger().info(f'Đã nhận {self._frame_count_perspective} frames từ /image_perspective')
            
            # Chuyển đổi từ ROS Image sang OpenCV Mat
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.callback_perspective(cv_image)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh perspective: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class ThetaTab(ttk.Frame):
    """Tab cho Theta Driver"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Biến trạng thái
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.theta_driver_process = None
        self.perspective_converter_process = None
        self.current_image_raw = None
        self.current_image_perspective = None
        self.is_ros_running = False
        self.frame_count_raw = 0
        self.frame_count_perspective = 0
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Theta"""
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Nút Launch Theta Driver
        self.launch_btn = ttk.Button(
            control_frame,
            text="Launch Theta Driver",
            command=self.launch_theta_driver
        )
        self.launch_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Launch Perspective Converter
        self.launch_perspective_btn = ttk.Button(
            control_frame,
            text="Launch Perspective Converter",
            command=self.launch_perspective_converter,
            state=tk.DISABLED
        )
        self.launch_perspective_btn.pack(side=tk.LEFT, padx=5)
        
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
        
        # Frame hiển thị ảnh - chia làm 2 phần
        image_frame = ttk.Frame(self, padding="10")
        image_frame.pack(fill=tk.BOTH, expand=True)
        
        # Frame cho Equirectangular (bên trái)
        equirect_frame = ttk.LabelFrame(image_frame, text="Equirectangular (360°)", padding="5")
        equirect_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.canvas_raw = tk.Canvas(
            equirect_frame,
            bg="black",
            highlightthickness=0
        )
        self.canvas_raw.pack(fill=tk.BOTH, expand=True)
        
        # Frame cho Perspective (bên phải)
        perspective_frame = ttk.LabelFrame(image_frame, text="Perspective (Front View)", padding="5")
        perspective_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        
        self.canvas_perspective = tk.Canvas(
            perspective_frame,
            bg="black",
            highlightthickness=0
        )
        self.canvas_perspective.pack(fill=tk.BOTH, expand=True)
        
        # Label thông tin ảnh
        info_frame = ttk.Frame(self, padding="5")
        info_frame.pack(fill=tk.X)
        
        self.info_label_raw = ttk.Label(
            info_frame,
            text="Equirectangular: Chưa có ảnh",
            font=("Arial", 9)
        )
        self.info_label_raw.pack(side=tk.LEFT, padx=10)
        
        self.info_label_perspective = ttk.Label(
            info_frame,
            text="Perspective: Chưa có ảnh",
            font=("Arial", 9)
        )
        self.info_label_perspective.pack(side=tk.LEFT, padx=10)
    
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
            self.launch_perspective_btn.config(state=tk.NORMAL)
            self.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_theta_driver_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch theta_driver: {e}")
    
    def launch_perspective_converter(self):
        """Launch perspective_converter node"""
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
            
            # Source setup.sh và chạy perspective_converter_node
            cmd = f"source {setup_script} && ros2 run theta_driver perspective_converter_node"
            
            self.perspective_converter_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.status_label.config(
                text="Trạng thái: Perspective Converter đang chạy",
                foreground="orange"
            )
            self.launch_perspective_btn.config(state=tk.DISABLED)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_perspective_converter_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch perspective_converter: {e}")
    
    def check_perspective_converter_process(self):
        """Kiểm tra xem perspective_converter process còn chạy không"""
        if self.perspective_converter_process:
            if self.perspective_converter_process.poll() is not None:
                # Process đã dừng
                self.status_label.config(
                    text="Trạng thái: Perspective Converter đã dừng",
                    foreground="orange"
                )
                self.launch_perspective_btn.config(state=tk.NORMAL)
    
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
                self.launch_perspective_btn.config(state=tk.DISABLED)
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
            self.ros_node = ImageSubscriber(
                self.on_image_received_raw,
                self.on_image_received_perspective
            )
            
            # Tạo executor và add node
            print("Tạo executor...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topic có tồn tại không
            print("Kiểm tra topic /image_raw...")
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
            self.frame_count_raw = 0
            self.frame_count_perspective = 0
            self.status_label.config(
                text="Trạng thái: Đang subscribe /image_raw và /image_perspective...",
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
                self.after(0, lambda: self.status_label.config(
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
    
    def on_image_received_raw(self, cv_image):
        """Callback khi nhận được ảnh equirectangular từ ROS"""
        self.current_image_raw = cv_image
        self.frame_count_raw += 1
        
        # Cập nhật UI trong main thread
        self.after(0, self.update_image_display_raw, cv_image)
        
        # Cập nhật status mỗi 30 frame
        if self.frame_count_raw % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.frame_count_raw} frames (raw), {self.frame_count_perspective} frames (perspective)",
                foreground="green"
            ))
    
    def on_image_received_perspective(self, cv_image):
        """Callback khi nhận được ảnh perspective từ ROS"""
        self.current_image_perspective = cv_image
        self.frame_count_perspective += 1
        
        # Cập nhật UI trong main thread
        self.after(0, self.update_image_display_perspective, cv_image)
        
        # Cập nhật status mỗi 30 frame
        if self.frame_count_perspective % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.frame_count_raw} frames (raw), {self.frame_count_perspective} frames (perspective)",
                foreground="green"
            ))
    
    def update_image_display_raw(self, cv_image):
        """Cập nhật hiển thị ảnh equirectangular trên canvas"""
        try:
            # Lấy kích thước canvas
            canvas_width = self.canvas_raw.winfo_width()
            canvas_height = self.canvas_raw.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                # Canvas chưa được render, thử lại sau
                self.after(100, lambda: self.update_image_display_raw(cv_image))
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
            self.canvas_raw.delete("all")
            self.canvas_raw.create_image(
                canvas_width // 2,
                canvas_height // 2,
                image=photo,
                anchor=tk.CENTER
            )
            
            # Lưu reference để tránh garbage collection
            self.canvas_raw.image = photo
            
            # Cập nhật thông tin
            self.info_label_raw.config(
                text=f"Equirectangular: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f}"
            )
            
        except Exception as e:
            print(f"Lỗi cập nhật hiển thị equirectangular: {e}")
    
    def update_image_display_perspective(self, cv_image):
        """Cập nhật hiển thị ảnh perspective trên canvas"""
        try:
            # Lấy kích thước canvas
            canvas_width = self.canvas_perspective.winfo_width()
            canvas_height = self.canvas_perspective.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                # Canvas chưa được render, thử lại sau
                self.after(100, lambda: self.update_image_display_perspective(cv_image))
                return
            
            # Resize ảnh để vừa với canvas (giữ tỷ lệ)
            img_height, img_width = cv_image.shape[:2]
            
            # Tính toán kích thước mới
            scale = min(canvas_width / img_width, canvas_height / img_height)
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            # Resize ảnh
            resized = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            # cv_image đã là RGB, không cần convert
            # Chuyển đổi sang PIL Image (PIL cũng dùng RGB)
            pil_image = PILImage.fromarray(resized)
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Xóa canvas cũ và vẽ ảnh mới
            self.canvas_perspective.delete("all")
            self.canvas_perspective.create_image(
                canvas_width // 2,
                canvas_height // 2,
                image=photo,
                anchor=tk.CENTER
            )
            
            # Lưu reference để tránh garbage collection
            self.canvas_perspective.image = photo
            
            # Cập nhật thông tin
            self.info_label_perspective.config(
                text=f"Perspective: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f}"
            )
            
        except Exception as e:
            print(f"Lỗi cập nhật hiển thị perspective: {e}")
    
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
        
        # Dừng perspective_converter process
        if self.perspective_converter_process:
            try:
                self.perspective_converter_process.terminate()
                self.perspective_converter_process.wait(timeout=5)
            except:
                try:
                    self.perspective_converter_process.kill()
                except:
                    pass
            self.perspective_converter_process = None
        
        # Cập nhật UI
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="red"
        )
        self.launch_btn.config(state=tk.NORMAL)
        self.launch_perspective_btn.config(state=tk.DISABLED)
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        
        # Xóa ảnh
        self.canvas_raw.delete("all")
        self.canvas_perspective.delete("all")
        self.info_label_raw.config(text="Equirectangular: Chưa có ảnh")
        self.info_label_perspective.config(text="Perspective: Chưa có ảnh")

