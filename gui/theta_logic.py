
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
import numpy as np
from languages.translate_engine import translator


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
        """Callback when receiving image"""
        try:
            # Debug: log every 30 frames
            if not hasattr(self, '_callback_count'):
                self._callback_count = 0
            self._callback_count += 1
            if self._callback_count % 30 == 0:
                self.get_logger().info(f'Received {self._callback_count} frames from {self.topic_name}')
                print(f'[SingleImageSubscriber] Received {self._callback_count} frames from {self.topic_name}')
            
            # Debug: log encoding
            if self._callback_count == 1:
                print(f'[SingleImageSubscriber] Encoding: {msg.encoding}, Size: {msg.width}x{msg.height}')
            
            # Handle JPEG compressed images
            if msg.encoding.lower() in ['jpeg', 'jpg']:
                # Decode JPEG data directly from compressed format
                jpeg_data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)
                if cv_image is None:
                    raise ValueError("Failed to decode JPEG image")
                # Convert BGR to RGB
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            else:
                # Use cv_bridge for standard encodings
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Debug: log image shape
            if self._callback_count == 1:
                print(f'[SingleImageSubscriber] CV Image shape: {cv_image.shape}')
            
            self.callback(cv_image)
        except Exception as e:
            error_msg = f'Error processing image from {self.topic_name}: {e}'
            self.get_logger().error(error_msg)
            print(f'[SingleImageSubscriber] ERROR: {error_msg}')
            import traceback
            traceback.print_exc()
            self.get_logger().error(traceback.format_exc())

class ThetaDriver(ttk.Frame):
    def __init__(self, log_callback, update_ui_theta_connected=None, canvas=None):
        super().__init__()
        self.canvas = canvas
        self.log = log_callback # Hàm callback để ghi log ra giao diện
        self.update_ui_theta_connected = update_ui_theta_connected
        self.record_process = None
        self.output_path = None
        self.camera_info_publisher_process = None
        self.is_camera_connected = False
        self.is_active_theta = False
        self.is_running = False
        self.create_control_panel()
    
    def create_control_panel(self):
        self.use4k_var = tk.BooleanVar(value=True)
        self.image_quality_var = tk.StringVar(value="high")  # Default: high
        self.jpeg_quality_var = tk.StringVar(value="85")  # Default: 85

    def launch_theta_driver(self):
        """Launch theta_driver node với parameters"""
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
            
            # Get parameters from UI
            use4k = self.use4k_var.get()
            image_quality = self.image_quality_var.get()
            jpeg_quality_str = self.jpeg_quality_var.get().strip()
            
            # Validate and convert jpeg_quality
            try:
                jpeg_quality = int(jpeg_quality_str) if jpeg_quality_str else 0
                if jpeg_quality < 0 or jpeg_quality > 100:
                    messagebox.showerror("Lỗi", "JPEG Quality phải trong khoảng 0-100")
                    return
            except ValueError:
                messagebox.showerror("Lỗi", "JPEG Quality phải là số nguyên")
                return
            
            # Build command with parameters
            cmd_parts = [f"source {setup_script}"]
            ros_cmd = "ros2 run theta_driver theta_driver_node"
            
            # Add parameters
            param_args = []
            param_args.append(f"-p use4k:={str(use4k).lower()}")
            param_args.append(f"-p image_quality:=\"{image_quality}\"")
            param_args.append(f"-p jpeg_quality:={jpeg_quality}")
            
            if param_args:
                ros_cmd += " --ros-args " + " ".join(param_args)
            
            cmd_parts.append(ros_cmd)
            cmd = " && ".join(cmd_parts)
            
            self.log(f"Launching theta_driver with command: {cmd}")
            
            # Use subprocess with proper environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.theta_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            # Check if process started successfully
            import time
            time.sleep(0.5)
            
            if self.theta_driver_process.poll() is not None:
                try:
                    stdout, _ = self.theta_driver_process.communicate(timeout=1)
                    error_msg = stdout if stdout else f"Process exited with code {self.theta_driver_process.returncode}"
                except subprocess.TimeoutExpired:
                    error_msg = "Process exited immediately (timeout reading output)"
                
                self.log(f"✗ theta_driver failed to start:")
                self.log(f"  Output: {error_msg}")
                self.is_active_theta = False
                if self.update_ui_theta_connected:
                    self.update_ui_theta_connected(self.is_active_theta)
                messagebox.showerror(
                    "Lỗi Launch",
                    translator.get("message.cannot_launch_theta_driver")
                )
                # self.launch_theta_btn.config(state=tk.NORMAL)
                return
            
            # self.log(f"✓ theta_driver process started (PID: {self.theta_driver_process.pid})")
            
            # Update status based on running processes
            if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                # Trạng thái: Theta Driver + Camera Info Publisher đang chạy
                self.is_active_theta = True
            else:
                self.is_active_theta = False

            
            self.log("✓ theta_driver process start_subscriber")
            self.start_subscriber()
            
            if self.update_ui_theta_connected:
                self.update_ui_theta_connected(self.is_active_theta)

            # self.launch_theta_btn.config(state=tk.DISABLED)
            # self.tab_equirect.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_theta_driver_process())
            
        except Exception as e:
            print(f"✗ theta_driver failed to start: {e}")
            messagebox.showerror("Lỗi", translator.get("message.cannot_launch_theta_driver"))
    
    def check_theta_usb_connection(self, check_theta_usb_callback=None):
        """Check if Theta X camera is connected via USB"""
        def check_usb():
            try:
                # Check if Theta X is connected via USB using lsusb
                result = subprocess.run(['lsusb'], 
                                      capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0 and 'theta' in result.stdout.lower():
                    self.is_camera_connected = True
                    self.log("✅ Theta X Connected")
                    self.log("Theta X camera detected via USB")
                else:
                    self.is_camera_connected = False
                    self.log("❌ Theta X Not Found")
                    self.log("Theta X camera not detected")
                    
            except Exception as e:
                self.is_camera_connected = False
                self.log("❌ Error Checking")
                self.log(f"Error checking Theta USB connection: {str(e)}")

            if check_theta_usb_callback:
                check_theta_usb_callback(self.is_camera_connected)
                
        threading.Thread(target=check_usb, daemon=True).start()
    
    def launch_camera_info_publisher(self):
        """Launch camera_info_publisher node cho calibration"""
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
            
            # Build command
            cmd_parts = [f"source {setup_script}"]
            ros_cmd = "ros2 run theta_driver camera_info_publisher_node"
            
            # Add parameters (default values suitable for calibration)
            param_args = []
            param_args.append("-p image_topic:=\"/image_raw\"")
            param_args.append("-p camera_info_topic:=\"/camera_info\"")
            param_args.append("-p camera_frame:=\"camera_link\"")
            param_args.append("-p use_calibration_params:=false")  # Use default for calibration
            
            if param_args:
                ros_cmd += " --ros-args " + " ".join(param_args)
            
            cmd_parts.append(ros_cmd)
            cmd = " && ".join(cmd_parts)
            
            self.log(f"Launching camera_info_publisher with command: {cmd}")
            
            # Use subprocess with proper environment
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
            
            # Check if process started successfully
            import time
            time.sleep(0.5)
            
            if self.camera_info_publisher_process.poll() is not None:
                try:
                    stdout, _ = self.camera_info_publisher_process.communicate(timeout=1)
                    error_msg = stdout if stdout else f"Process exited with code {self.camera_info_publisher_process.returncode}"
                except subprocess.TimeoutExpired:
                    error_msg = "Process exited immediately (timeout reading output)"
                
                # self.log(f"✗ camera_info_publisher failed to start:")
                self.log(f"  Output: {error_msg}")
                
                messagebox.showerror(
                    "Lỗi Launch",
                    translator.get("message.cannot_launch_camera_info_publisher")
                )
                # self.launch_camera_info_btn.config(state=tk.NORMAL)
                return
            
            self.log(f"✓ camera_info_publisher process started (PID: {self.camera_info_publisher_process.pid})")
            
            # Update status based on running processes
            if self.theta_driver_process and self.theta_driver_process.poll() is None:
                self.is_active_theta = True
                # self.status_label.config(
                #     text="Trạng thái: Theta Driver + Camera Info Publisher đang chạy",
                #     foreground="green"
                # )
            else:
                self.is_active_theta = False
            if self.update_ui_theta_connected:
                self.update_ui_theta_connected(self.is_active_theta)
                # self.status_label.config(
                #     text="Trạng thái: Camera Info Publisher đang chạy",
                #     foreground="green"
                # )
            # self.launch_camera_info_btn.config(state=tk.DISABLED)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_camera_info_publisher_process())
            
        except Exception as e:
            self.log(f"✗ camera_info_publisher failed to start: {e}")
            messagebox.showerror("Lỗi", translator.get("message.cannot_launch_camera_info_publisher"))
    
    def check_camera_info_publisher_process(self):
        """Kiểm tra xem camera_info_publisher process còn chạy không"""
        if self.camera_info_publisher_process:
            if self.camera_info_publisher_process.poll() is not None:
                # Update status based on other running processes
                if self.theta_driver_process and self.theta_driver_process.poll() is None:
                    self.is_active_theta = True
                    self.log("Camera Info Publisher is still running")
                    # self.status_label.config(
                    #     text="Trạng thái: Theta Driver đang chạy",
                    #     foreground="orange"
                    # )
                else:
                    self.is_active_theta = False
                    
                    self.log("Camera Info Publisher is not running")

                if self.update_ui_theta_connected:
                    self.update_ui_theta_connected(self.is_active_theta)

                    # self.status_label.config(
                    #     text="Trạng thái: Camera Info Publisher đã dừng",
                    #     foreground="orange"
                    # )

    def check_theta_driver_process(self):
        """Kiểm tra xem theta_driver process còn chạy không"""
        if self.theta_driver_process:
            if self.theta_driver_process.poll() is not None:
                # Update status based on other running processes
                if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                    self.is_active_theta = True

                    # self.status_label.config(
                    #     text="Trạng thái: Camera Info Publisher đang chạy",
                    #     foreground="green"
                    # )
                else:
                    self.is_active_theta = False
                    # self.status_label.config(
                    #     text="Trạng thái: Theta Driver đã dừng",
                    #     foreground="red"
                    # )
                if self.update_ui_theta_connected:
                    self.update_ui_theta_connected(self.is_active_theta)
                # self.launch_theta_btn.config(state=tk.NORMAL)
                self.stop_all()

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
            # if self.is_running:
            #     self.after(0, lambda: self.status_label.config(
            #         text=f"Lỗi: {str(e)[:50]}",
            #         foreground="red"
            #     ))
    
    def start_subscriber(self):
        """Start ROS subscriber"""
        if self.is_running:
            return
        self.log("Bắt đầu ROS2 subscriber cho /image_raw...")
        try:
            if not rclpy.ok():
                self.log("Khởi tạo ROS2...")
                rclpy.init()
            
            self.log("Tạo ROS2 node subscriber cho /image_raw...")
            self.ros_node = SingleImageSubscriber(
                '/image_raw',  # Đảm bảo có / prefix
                self.on_image_received,
                'equirectangular_subscriber'
            )
            
            self.log("Tạo executor...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topic có tồn tại không
            self.log("Kiểm tra topic /image_raw...")
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if '/image_raw' in result.stdout:
                self.log("✓ Topic /image_raw đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic /image_raw chưa tồn tại")
                self.log("Các topics có sẵn:")
                self.log(result.stdout)
            
            self.log("Khởi động ROS thread...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_running = True
            self.frame_count = 0
            # self.status_label.config(
            #     text="Equirectangular: Đang subscribe /image_raw...",
            #     foreground="green"
            # )
            # self.start_btn.config(state=tk.DISABLED)
            # self.stop_btn.config(state=tk.NORMAL)
            
            self.log("✓ ROS subscriber đã khởi động")
            
        except Exception as e:
            error_msg = f"Không thể start subscriber: {e}"
            self.log(f"✗ Lỗi: {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Lỗi", error_msg)
            
    def on_image_received(self, cv_image):
        """Callback khi nhận được ảnh"""
        try:
            self.current_image = cv_image
            self.frame_count += 1
            
            # Debug: log mỗi 30 frame
            # if self.frame_count % 30 == 0:
            #     self.log(f"✓ Equirectangular: Đã nhận {self.frame_count} frames")
            
            # Cập nhật UI trong main thread
            self.after(0, self.update_image_display, cv_image)
            
            # Cập nhật status mỗi 30 frame
            # if self.frame_count % 30 == 0:
            #     self.after(0, lambda: self.status_label.config(
            #         text=f"Equirectangular: {self.frame_count} frames | /image_raw",
            #         foreground="green"
            #     ))
        except Exception as e:
            self.log(f"✗ Lỗi trong on_image_received: {e}")
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
            
            # self.status_label.config(
            #     text=f"Equirectangular: {img_width}x{img_height} | "
            #          f"Hiển thị: {new_width}x{new_height} | "
            #          f"Scale: {scale:.2f} | Frames: {self.frame_count}",
            #     foreground="green"
            # )
            
        except Exception as e:
            self.log(f"Lỗi cập nhật hiển thị equirectangular: {e}")
    
    def stop_all(self):
        """Stop tất cả"""
        # Stop equirectangular tab
        # if self.tab_equirect.is_running:
        #     self.tab_equirect.stop_all()
        
        # Stop camera_info_publisher
        if self.camera_info_publisher_process:
            try:
                self.camera_info_publisher_process.terminate()
                self.camera_info_publisher_process.wait(timeout=5)
            except:
                try:
                    self.camera_info_publisher_process.kill()
                except:
                    pass
            self.camera_info_publisher_process = None
            # self.is_active_theta = False
        
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
            self.is_active_theta = False
            if self.update_ui_theta_connected:
                self.update_ui_theta_connected(self.is_active_theta)
        
        # self.status_label.config(
        #     text="Trạng thái: Đã dừng",
        #     foreground="red"
        # )
        # self.launch_theta_btn.config(state=tk.NORMAL)
        # self.launch_camera_info_btn.config(state=tk.NORMAL)