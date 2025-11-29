#!/usr/bin/env python3
"""
Theta Tab Module
Chứa ThetaTab với Notebook sub-tabs cho 5 camera models
"""

import os
# Fix Qt plugin issue với OpenCV
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''
if 'QT_PLUGIN_PATH' in os.environ:
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
os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '0'
import cv2
try:
    cv2.setNumThreads(1)
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


class SingleImageSubscriber(Node):
    """ROS2 Node để subscribe một topic image"""
    
    def __init__(self, topic_name, callback, node_name='image_subscriber'):
        super().__init__(node_name)
        
        # Đảm bảo topic name có / prefix
        if not topic_name.startswith('/'):
            topic_name = '/' + topic_name
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.callback = callback
        self.topic_name = topic_name
        self.get_logger().info(f'Đã subscribe topic {topic_name}')
    
    def image_callback(self, msg):
        """Callback khi nhận được ảnh"""
        try:
            # Debug: log mỗi 30 frame
            if not hasattr(self, '_callback_count'):
                self._callback_count = 0
            self._callback_count += 1
            if self._callback_count % 30 == 0:
                self.get_logger().info(f'Đã nhận {self._callback_count} frames từ {self.topic_name}')
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.callback(cv_image)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh từ {self.topic_name}: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class CameraModelTab(ttk.Frame):
    """Tab cho một camera model cụ thể"""
    
    def __init__(self, parent, model_name, topic_name, camera_info_topic, node_name):
        super().__init__(parent)
        self.model_name = model_name
        self.topic_name = topic_name
        self.camera_info_topic = camera_info_topic
        self.node_name = node_name
        
        # State
        self.converter_process = None
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_running = False
        self.frame_count = 0
        self.current_image = None
        
        # UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo UI widgets"""
        # Control frame
        control_frame = ttk.Frame(self, padding="5")
        control_frame.pack(fill=tk.X)
        
        self.launch_btn = ttk.Button(
            control_frame,
            text=f"Launch {self.model_name} Converter",
            command=self.launch_converter
        )
        self.launch_btn.pack(side=tk.LEFT, padx=5)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_subscriber,
            state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="Stop",
            command=self.stop_all,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Image canvas
        self.canvas = tk.Canvas(
            self,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Status label
        self.status_label = ttk.Label(
            self,
            text=f"{self.model_name}: Chưa có ảnh",
            font=("Arial", 9)
        )
        self.status_label.pack(fill=tk.X, padx=5, pady=5)
    
    def launch_converter(self):
        """Launch converter node cho camera model này"""
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
            
            # TODO: Update khi có universal_camera_converter_node
            # Hiện tại dùng perspective_converter_node làm placeholder
            cmd = f"source {setup_script} && ros2 run theta_driver {self.node_name}"
            
            self.converter_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.status_label.config(
                text=f"{self.model_name}: Converter đang chạy",
                foreground="orange"
            )
            self.launch_btn.config(state=tk.DISABLED)
            self.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_converter_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch {self.model_name} converter: {e}")
    
    def check_converter_process(self):
        """Kiểm tra xem converter process còn chạy không"""
        if self.converter_process:
            if self.converter_process.poll() is not None:
                # Process đã dừng
                self.status_label.config(
                    text=f"{self.model_name}: Converter đã dừng",
                    foreground="orange"
                )
                self.launch_btn.config(state=tk.NORMAL)
                self.start_btn.config(state=tk.DISABLED)
                if self.is_running:
                    self.stop_all()
    
    def start_subscriber(self):
        """Start ROS subscriber cho topic này"""
        if self.is_running:
            return
        
        try:
            if not rclpy.ok():
                print(f"Khởi tạo ROS2 cho {self.model_name}...")
                rclpy.init()
            
            # Đảm bảo topic name có / prefix
            topic_name = self.topic_name if self.topic_name.startswith('/') else f'/{self.topic_name}'
            
            print(f"Tạo ROS2 node subscriber cho {self.model_name} - topic: {topic_name}...")
            self.ros_node = SingleImageSubscriber(
                topic_name,
                self.on_image_received,
                f'{self.model_name.lower()}_subscriber'
            )
            
            print(f"Tạo executor cho {self.model_name}...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topic có tồn tại không
            print(f"Kiểm tra topic {topic_name}...")
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if topic_name in result.stdout:
                print(f"✓ Topic {topic_name} đã tồn tại")
            else:
                print(f"⚠️  Cảnh báo: Topic {topic_name} chưa tồn tại")
                print("Các topics có sẵn:")
                print(result.stdout)
            
            print(f"Khởi động ROS thread cho {self.model_name}...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_running = True
            self.frame_count = 0
            self.status_label.config(
                text=f"{self.model_name}: Đang subscribe {topic_name}...",
                foreground="green"
            )
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            print(f"✓ ROS subscriber đã khởi động cho {self.model_name}")
            
        except Exception as e:
            error_msg = f"Không thể start subscriber: {e}"
            print(f"✗ Lỗi: {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Lỗi", error_msg)
    
    def ros_spin(self):
        """Spin ROS node trong thread riêng"""
        try:
            print(f"ROS spin thread đã bắt đầu cho {self.model_name}")
            while rclpy.ok() and self.is_running:
                if self.ros_executor is not None:
                    self.ros_executor.spin_once(timeout_sec=0.1)
                else:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            error_msg = f"Lỗi trong ROS spin: {e}"
            print(f"✗ {error_msg} ({self.model_name})")
            import traceback
            traceback.print_exc()
            if self.is_running:
                self.after(0, lambda: self.status_label.config(
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
    
    def on_image_received(self, cv_image):
        """Callback khi nhận được ảnh từ ROS"""
        try:
            self.current_image = cv_image
            self.frame_count += 1
            
            # Debug: log mỗi 30 frame
            if self.frame_count % 30 == 0:
                print(f"✓ {self.model_name}: Đã nhận {self.frame_count} frames")
            
            # Cập nhật UI trong main thread
            self.after(0, self.update_image_display, cv_image)
            
            # Cập nhật status mỗi 30 frame
            if self.frame_count % 30 == 0:
                self.after(0, lambda: self.status_label.config(
                    text=f"{self.model_name}: {self.frame_count} frames | {self.topic_name}",
                    foreground="green"
                ))
        except Exception as e:
            print(f"✗ Lỗi trong on_image_received ({self.model_name}): {e}")
            import traceback
            traceback.print_exc()
    
    def update_image_display(self, cv_image):
        """Cập nhật hiển thị ảnh trên canvas"""
        try:
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                self.after(100, lambda: self.update_image_display(cv_image))
                return
            
            img_height, img_width = cv_image.shape[:2]
            
            scale = min(canvas_width / img_width, canvas_height / img_height)
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            resized = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            pil_image = PILImage.fromarray(resized)
            photo = ImageTk.PhotoImage(image=pil_image)
            
            self.canvas.delete("all")
            self.canvas.create_image(
                canvas_width // 2,
                canvas_height // 2,
                image=photo,
                anchor=tk.CENTER
            )
            
            self.canvas.image = photo
            
            self.status_label.config(
                text=f"{self.model_name}: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f} | Frames: {self.frame_count}",
                foreground="green"
            )
            
        except Exception as e:
            print(f"Lỗi cập nhật hiển thị {self.model_name}: {e}")
    
    def stop_all(self):
        """Stop converter và subscriber"""
        self.is_running = False
        
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
        
        if self.converter_process:
            try:
                self.converter_process.terminate()
                self.converter_process.wait(timeout=5)
            except:
                try:
                    self.converter_process.kill()
                except:
                    pass
            self.converter_process = None
        
        self.status_label.config(
            text=f"{self.model_name}: Đã dừng",
            foreground="red"
        )
        self.launch_btn.config(state=tk.NORMAL)
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        
        self.canvas.delete("all")
        self.frame_count = 0


class EquirectangularTab(ttk.Frame):
    """Tab cho equirectangular (original) - chỉ subscribe, không có converter"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # State
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_running = False
        self.frame_count = 0
        self.current_image = None
        
        # UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo UI widgets"""
        # Control frame
        control_frame = ttk.Frame(self, padding="5")
        control_frame.pack(fill=tk.X)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_subscriber,
            state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="Stop",
            command=self.stop_all,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Image canvas
        self.canvas = tk.Canvas(
            self,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Status label
        self.status_label = ttk.Label(
            self,
            text="Equirectangular: Chưa có ảnh",
            font=("Arial", 9)
        )
        self.status_label.pack(fill=tk.X, padx=5, pady=5)
    
    def start_subscriber(self):
        """Start ROS subscriber"""
        if self.is_running:
            return
        
        try:
            if not rclpy.ok():
                print("Khởi tạo ROS2...")
                rclpy.init()
            
            print("Tạo ROS2 node subscriber cho /image_raw...")
            self.ros_node = SingleImageSubscriber(
                '/image_raw',  # Đảm bảo có / prefix
                self.on_image_received,
                'equirectangular_subscriber'
            )
            
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
            
            print("Khởi động ROS thread...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_running = True
            self.frame_count = 0
            self.status_label.config(
                text="Equirectangular: Đang subscribe /image_raw...",
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
            print("ROS spin thread đã bắt đầu cho Equirectangular")
            while rclpy.ok() and self.is_running:
                if self.ros_executor is not None:
                    self.ros_executor.spin_once(timeout_sec=0.1)
                else:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            error_msg = f"Lỗi trong ROS spin: {e}"
            print(f"✗ {error_msg}")
            import traceback
            traceback.print_exc()
            if self.is_running:
                self.after(0, lambda: self.status_label.config(
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
    
    def on_image_received(self, cv_image):
        """Callback khi nhận được ảnh"""
        try:
            self.current_image = cv_image
            self.frame_count += 1
            
            # Debug: log mỗi 30 frame
            if self.frame_count % 30 == 0:
                print(f"✓ Equirectangular: Đã nhận {self.frame_count} frames")
            
            # Cập nhật UI trong main thread
            self.after(0, self.update_image_display, cv_image)
            
            # Cập nhật status mỗi 30 frame
            if self.frame_count % 30 == 0:
                self.after(0, lambda: self.status_label.config(
                    text=f"Equirectangular: {self.frame_count} frames | /image_raw",
                    foreground="green"
                ))
        except Exception as e:
            print(f"✗ Lỗi trong on_image_received: {e}")
            import traceback
            traceback.print_exc()
    
    def update_image_display(self, cv_image):
        """Cập nhật hiển thị ảnh"""
        try:
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            
            if canvas_width <= 1 or canvas_height <= 1:
                self.after(100, lambda: self.update_image_display(cv_image))
                return
            
            img_height, img_width = cv_image.shape[:2]
            
            scale = min(canvas_width / img_width, canvas_height / img_height)
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            
            resized = cv2.resize(cv_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
            
            pil_image = PILImage.fromarray(resized)
            photo = ImageTk.PhotoImage(image=pil_image)
            
            self.canvas.delete("all")
            self.canvas.create_image(
                canvas_width // 2,
                canvas_height // 2,
                image=photo,
                anchor=tk.CENTER
            )
            
            self.canvas.image = photo
            
            self.status_label.config(
                text=f"Equirectangular: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f} | Frames: {self.frame_count}",
                foreground="green"
            )
            
        except Exception as e:
            print(f"Lỗi cập nhật hiển thị equirectangular: {e}")
    
    def stop_all(self):
        """Stop subscriber"""
        self.is_running = False
        
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
        
        self.status_label.config(
            text="Equirectangular: Đã dừng",
            foreground="red"
        )
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        
        self.canvas.delete("all")
        self.frame_count = 0


class ThetaTab(ttk.Frame):
    """Main tab với Notebook sub-tabs cho các camera models"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # State
        self.theta_driver_process = None
        self.camera_tabs = {}  # {model_name: CameraModelTab}
        
        # UI
        self.create_control_panel()
        self.create_notebook()
    
    def create_control_panel(self):
        """Tạo control panel ở top"""
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Launch Theta Driver
        self.launch_theta_btn = ttk.Button(
            control_frame,
            text="Launch Theta Driver",
            command=self.launch_theta_driver
        )
        self.launch_theta_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Chưa kết nối",
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
    
    def create_notebook(self):
        """Tạo notebook với sub-tabs"""
        self.notebook = ttk.Notebook(self)
        
        # Tab Equirectangular (original)
        self.tab_equirect = EquirectangularTab(self.notebook)
        self.notebook.add(self.tab_equirect, text="Equirectangular")
        
        # Tab Pinhole
        # Note: perspective_converter_node publishes to /image_perspective by default
        self.tab_pinhole = CameraModelTab(
            self.notebook,
            "Pinhole",
            "image_perspective",  # Changed from image_pinhole to match perspective_converter_node output
            "camera_info",
            "perspective_converter_node"
        )
        self.notebook.add(self.tab_pinhole, text="Pinhole")
        self.camera_tabs["Pinhole"] = self.tab_pinhole
        
        # Tab EquidistantCamera
        self.tab_equidistant = CameraModelTab(
            self.notebook,
            "EquidistantCamera",
            "image_equidistant",
            "camera_info_equidistant",
            "equidistant_converter_node"  # TODO: Update when implemented
        )
        self.notebook.add(self.tab_equidistant, text="Equidistant")
        self.camera_tabs["EquidistantCamera"] = self.tab_equidistant
        
        # Tab PolynomialCamera
        self.tab_polynomial = CameraModelTab(
            self.notebook,
            "PolynomialCamera",
            "image_polynomial",
            "camera_info_polynomial",
            "polynomial_converter_node"  # TODO: Update when implemented
        )
        self.notebook.add(self.tab_polynomial, text="Polynomial")
        self.camera_tabs["PolynomialCamera"] = self.tab_polynomial
        
        # Tab ATAN
        self.tab_atan = CameraModelTab(
            self.notebook,
            "ATAN",
            "image_atan",
            "camera_info_atan",
            "atan_converter_node"  # TODO: Update when implemented
        )
        self.notebook.add(self.tab_atan, text="ATAN")
        self.camera_tabs["ATAN"] = self.tab_atan
        
        # Tab Ocam
        self.tab_ocam = CameraModelTab(
            self.notebook,
            "Ocam",
            "image_ocam",
            "camera_info_ocam",
            "ocam_converter_node"  # TODO: Update when implemented
        )
        self.notebook.add(self.tab_ocam, text="Ocam")
        self.camera_tabs["Ocam"] = self.tab_ocam
        
        self.notebook.pack(fill=tk.BOTH, expand=True)
    
    def launch_theta_driver(self):
        """Launch theta_driver node"""
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
            self.launch_theta_btn.config(state=tk.DISABLED)
            self.tab_equirect.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_theta_driver_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch theta_driver: {e}")
    
    def check_theta_driver_process(self):
        """Kiểm tra xem theta_driver process còn chạy không"""
        if self.theta_driver_process:
            if self.theta_driver_process.poll() is not None:
                self.status_label.config(
                    text="Trạng thái: Theta Driver đã dừng",
                    foreground="red"
                )
                self.launch_theta_btn.config(state=tk.NORMAL)
                self.stop_all()
    
    def stop_all(self):
        """Stop tất cả"""
        # Stop equirectangular tab
        if self.tab_equirect.is_running:
            self.tab_equirect.stop_all()
        
        # Stop all camera model tabs
        for tab in self.camera_tabs.values():
            if tab.is_running or tab.converter_process is not None:
                tab.stop_all()
        
        # Stop theta driver
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
        
        self.status_label.config(
            text="Trạng thái: Đã dừng",
            foreground="red"
        )
        self.launch_theta_btn.config(state=tk.NORMAL)
