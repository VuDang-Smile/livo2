#!/usr/bin/env python3
"""
Multi-Tab GUI Viewer
GUI với 2 tabs: Theta Driver và Livox Driver 2
"""

import sys
import threading
import subprocess
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2, Imu
from cv_bridge import CvBridge
try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None
import cv2
import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
    from PIL import Image as PILImage
    from PIL import ImageTk
except ImportError as e:
    print(f"Lỗi import: {e}")
    print("Vui lòng cài đặt: pip install pillow")
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


class LivoxSubscriber(Node):
    """ROS2 Node để subscribe topics /livox/lidar và /livox/imu"""
    
    def __init__(self, callback_lidar, callback_imu):
        super().__init__('livox_viewer_subscriber')
        
        # Kiểm tra xem có CustomMsg không (cần source setup.sh trước)
        if CustomMsg is None:
            self.get_logger().error(
                "Không thể import CustomMsg. "
                "Vui lòng source ws/install/setup.sh trước khi chạy GUI."
            )
            # Fallback về PointCloud2 nếu không có CustomMsg
            lidar_msg_type = PointCloud2
        else:
            # Sử dụng CustomMsg vì launch file dùng xfer_format=1
            lidar_msg_type = CustomMsg
        
        # Subscribe topic lidar với đúng message type
        self.subscription_lidar = self.create_subscription(
            lidar_msg_type,
            'livox/lidar',
            self.lidar_callback,
            10
        )
        
        # Subscribe topic IMU
        self.subscription_imu = self.create_subscription(
            Imu,
            'livox/imu',
            self.imu_callback,
            10
        )
        
        self.callback_lidar = callback_lidar
        self.callback_imu = callback_imu
        self.lidar_msg_type = lidar_msg_type
        # Không log để tránh spam log
    
    def lidar_callback(self, msg):
        """Callback khi nhận được point cloud từ lidar"""
        try:
            if not hasattr(self, '_lidar_count'):
                self._lidar_count = 0
            self._lidar_count += 1
            # Không log để tránh spam log
            
            self.callback_lidar(msg)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý lidar data: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def imu_callback(self, msg):
        """Callback khi nhận được IMU data"""
        try:
            if not hasattr(self, '_imu_count'):
                self._imu_count = 0
            self._imu_count += 1
            # Không log để tránh spam log
            
            self.callback_imu(msg)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý IMU data: {e}')
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


class LivoxTab(ttk.Frame):
    """Tab cho Livox Driver 2"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Biến trạng thái
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.livox_driver_process = None
        self.is_ros_running = False
        self.lidar_count = 0
        self.imu_count = 0
        
        # Tạo UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo các widget cho tab Livox"""
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Nút Start Livox Driver
        self.start_driver_btn = ttk.Button(
            control_frame,
            text="Start Livox MID 360",
            command=self.start_livox_driver
        )
        self.start_driver_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Livox Driver
        self.stop_driver_btn = ttk.Button(
            control_frame,
            text="Stop Livox Driver",
            command=self.stop_livox_driver,
            state=tk.DISABLED
        )
        self.stop_driver_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start ROS Subscriber
        self.start_subscriber_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_ros_subscriber,
            state=tk.DISABLED
        )
        self.start_subscriber_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Subscriber
        self.stop_subscriber_btn = ttk.Button(
            control_frame,
            text="Stop Subscriber",
            command=self.stop_ros_subscriber,
            state=tk.DISABLED
        )
        self.stop_subscriber_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Chưa kết nối",
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin topics
        info_frame = ttk.LabelFrame(self, text="Thông tin Topics", padding="10")
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Frame cho Lidar info
        lidar_frame = ttk.LabelFrame(info_frame, text="Lidar Topic: /livox/lidar", padding="5")
        lidar_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.lidar_info_label = ttk.Label(
            lidar_frame,
            text="Chưa nhận dữ liệu",
            font=("Arial", 10)
        )
        self.lidar_info_label.pack(anchor=tk.W, padx=5)
        
        # Frame cho IMU info
        imu_frame = ttk.LabelFrame(info_frame, text="IMU Topic: /livox/imu", padding="5")
        imu_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.imu_info_label = ttk.Label(
            imu_frame,
            text="Chưa nhận dữ liệu",
            font=("Arial", 10)
        )
        self.imu_info_label.pack(anchor=tk.W, padx=5)
        
        # Text area để hiển thị log
        log_frame = ttk.LabelFrame(self, text="Log", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=10,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message):
        """Thêm message vào log"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"{message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    def start_livox_driver(self):
        """Start Livox MID 360 driver"""
        try:
            workspace_path = Path(__file__).parent.parent / "ws"
            setup_script = workspace_path / "install" / "setup.sh"
            
            if not setup_script.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy setup.sh tại: {setup_script}\n"
                    "Vui lòng build workspace trước."
                )
                return
            
            # Source setup.sh và chạy livox driver với launch file
            livox_launch_path = workspace_path / "src" / "livox_ros_driver2" / "launch_ROS2" / "msg_MID360_launch.py"
            
            if not livox_launch_path.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy launch file tại: {livox_launch_path}"
                )
                return
            
            # Source setup.sh trước để lấy class, sau đó chạy livox driver với ros2 launch
            # Cần source setup.sh để có đầy đủ dependencies từ livox_ros_driver2
            cmd = f"source {setup_script} && ros2 launch livox_ros_driver2 msg_MID360_launch.py"
            
            self.log(f"Đang khởi động Livox MID 360 driver...")
            # Redirect output vào /dev/null để tránh spam log
            self.livox_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            self.status_label.config(
                text="Trạng thái: Livox Driver đang chạy",
                foreground="orange"
            )
            self.start_driver_btn.config(state=tk.DISABLED)
            self.stop_driver_btn.config(state=tk.NORMAL)
            self.start_subscriber_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_livox_driver_process)
            
        except Exception as e:
            error_msg = f"Không thể start Livox driver: {e}"
            self.log(f"Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
    
    def check_livox_driver_process(self):
        """Kiểm tra xem livox driver process còn chạy không"""
        if self.livox_driver_process:
            if self.livox_driver_process.poll() is not None:
                # Process đã dừng
                self.status_label.config(
                    text="Trạng thái: Livox Driver đã dừng",
                    foreground="red"
                )
                self.start_driver_btn.config(state=tk.NORMAL)
                self.stop_driver_btn.config(state=tk.DISABLED)
                self.start_subscriber_btn.config(state=tk.DISABLED)
                if self.is_ros_running:
                    self.stop_ros_subscriber()
                self.log("Livox Driver đã dừng")
            else:
                # Process vẫn chạy, kiểm tra lại sau
                self.after(2000, self.check_livox_driver_process)
    
    def stop_livox_driver(self):
        """Stop Livox driver"""
        if self.livox_driver_process:
            try:
                self.log("Đang dừng Livox Driver...")
                self.livox_driver_process.terminate()
                self.livox_driver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    self.livox_driver_process.kill()
                except:
                    pass
            except Exception as e:
                self.log(f"Lỗi khi dừng driver: {e}")
            finally:
                self.livox_driver_process = None
                self.status_label.config(
                    text="Trạng thái: Livox Driver đã dừng",
                    foreground="red"
                )
                self.start_driver_btn.config(state=tk.NORMAL)
                self.stop_driver_btn.config(state=tk.DISABLED)
                self.start_subscriber_btn.config(state=tk.DISABLED)
                if self.is_ros_running:
                    self.stop_ros_subscriber()
    
    def start_ros_subscriber(self):
        """Bắt đầu ROS subscriber cho livox topics"""
        if self.is_ros_running:
            return
        
        try:
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                self.log("Khởi tạo ROS2...")
                rclpy.init()
            
            # Kiểm tra xem có CustomMsg không (cần source setup.sh)
            if CustomMsg is None:
                self.log("⚠️  Cảnh báo: Không thể import CustomMsg")
                self.log("   Vui lòng source ws/install/setup.sh trước khi chạy GUI:")
                self.log("   source ws/install/setup.sh")
                self.log("   Sau đó chạy lại GUI")
                messagebox.showwarning(
                    "Cảnh báo",
                    "Không thể import CustomMsg.\n"
                    "Vui lòng source ws/install/setup.sh trước khi chạy GUI:\n\n"
                    "source ws/install/setup.sh\n\n"
                    "Sau đó chạy lại GUI."
                )
                return
            
            # Tạo node subscriber
            self.log("Tạo ROS2 node cho Livox...")
            self.ros_node = LivoxSubscriber(
                self.on_lidar_received,
                self.on_imu_received
            )
            
            # Tạo executor và add node
            self.log("Tạo executor...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topics có tồn tại không
            self.log("Kiểm tra topics...")
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            topics_found = []
            if '/livox/lidar' in result.stdout:
                topics_found.append('/livox/lidar')
                self.log("✓ Topic /livox/lidar đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic /livox/lidar chưa tồn tại")
            
            if '/livox/imu' in result.stdout:
                topics_found.append('/livox/imu')
                self.log("✓ Topic /livox/imu đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic /livox/imu chưa tồn tại")
            
            if not topics_found:
                self.log("⚠️  Không tìm thấy topics. Đảm bảo Livox Driver đang chạy.")
            
            # Chạy ROS trong thread riêng
            self.log("Khởi động ROS thread...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_ros_running = True
            self.lidar_count = 0
            self.imu_count = 0
            self.status_label.config(
                text="Trạng thái: Đang subscribe /livox/lidar và /livox/imu...",
                foreground="green"
            )
            self.start_subscriber_btn.config(state=tk.DISABLED)
            self.stop_subscriber_btn.config(state=tk.NORMAL)
            
            self.log("✓ ROS subscriber đã khởi động")
            
        except Exception as e:
            error_msg = f"Không thể start subscriber: {e}"
            self.log(f"✗ Lỗi: {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Lỗi", error_msg)
    
    def ros_spin(self):
        """Spin ROS node trong thread riêng"""
        try:
            self.log("ROS spin thread đã bắt đầu")
            while rclpy.ok() and self.is_ros_running:
                # Dùng executor để spin
                if self.ros_executor is not None:
                    self.ros_executor.spin_once(timeout_sec=0.1)
                else:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            error_msg = f"Lỗi trong ROS spin: {e}"
            self.log(f"✗ {error_msg}")
            import traceback
            traceback.print_exc()
            # Cập nhật UI để báo lỗi
            if self.is_ros_running:
                self.after(0, lambda: self.status_label.config(
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
    
    def on_lidar_received(self, msg):
        """Callback khi nhận được lidar data"""
        self.lidar_count += 1
        
        # Xử lý cả CustomMsg và PointCloud2
        if CustomMsg and isinstance(msg, CustomMsg):
            # CustomMsg format
            info_text = (
                f"Đã nhận {self.lidar_count} point clouds | "
                f"Point num: {msg.point_num} | "
                f"Type: CustomMsg | "
                f"Lidar ID: {msg.lidar_id}"
            )
        else:
            # PointCloud2 format
            info_text = (
                f"Đã nhận {self.lidar_count} point clouds | "
                f"Width: {msg.width} | Height: {msg.height} | "
                f"Point step: {msg.point_step} bytes | "
                f"Type: PointCloud2"
            )
        
        # Cập nhật UI trong main thread
        self.after(0, lambda: self.lidar_info_label.config(text=info_text))
        
        # Cập nhật status mỗi 30 message
        if self.lidar_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.lidar_count} lidar, {self.imu_count} IMU messages",
                foreground="green"
            ))
    
    def on_imu_received(self, msg):
        """Callback khi nhận được IMU data"""
        self.imu_count += 1
        
        # Cập nhật UI trong main thread
        angular_vel = msg.angular_velocity
        linear_accel = msg.linear_acceleration
        
        self.after(0, lambda: self.imu_info_label.config(
            text=f"Đã nhận {self.imu_count} IMU messages | "
                 f"Angular: [{angular_vel.x:.3f}, {angular_vel.y:.3f}, {angular_vel.z:.3f}] | "
                 f"Linear: [{linear_accel.x:.3f}, {linear_accel.y:.3f}, {linear_accel.z:.3f}]"
        ))
        
        # Cập nhật status mỗi 30 message
        if self.imu_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.lidar_count} lidar, {self.imu_count} IMU messages",
                foreground="green"
            ))
    
    def stop_ros_subscriber(self):
        """Dừng ROS subscriber"""
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
        
        # Cập nhật UI
        self.status_label.config(
            text="Trạng thái: Subscriber đã dừng",
            foreground="red"
        )
        self.start_subscriber_btn.config(state=tk.NORMAL)
        self.stop_subscriber_btn.config(state=tk.DISABLED)
        
        self.lidar_info_label.config(text="Chưa nhận dữ liệu")
        self.imu_info_label.config(text="Chưa nhận dữ liệu")
        self.log("ROS subscriber đã dừng")


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
