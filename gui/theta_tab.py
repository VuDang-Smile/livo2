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
                print(f'[SingleImageSubscriber] Đã nhận {self._callback_count} frames từ {self.topic_name}')
            
            # Debug: log encoding
            if self._callback_count == 1:
                print(f'[SingleImageSubscriber] Encoding: {msg.encoding}, Size: {msg.width}x{msg.height}')
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Debug: log image shape
            if self._callback_count == 1:
                print(f'[SingleImageSubscriber] CV Image shape: {cv_image.shape}')
            
            self.callback(cv_image)
        except Exception as e:
            error_msg = f'Lỗi xử lý ảnh từ {self.topic_name}: {e}'
            self.get_logger().error(error_msg)
            print(f'[SingleImageSubscriber] ERROR: {error_msg}')
            import traceback
            traceback.print_exc()
            self.get_logger().error(traceback.format_exc())


class CameraModelTab(ttk.Frame):
    """Tab cho một camera model cụ thể"""
    
    def __init__(self, parent, model_name, topic_name, camera_info_topic, node_name, parameters=None):
        super().__init__(parent)
        self.model_name = model_name
        self.topic_name = topic_name
        self.camera_info_topic = camera_info_topic
        self.node_name = node_name
        
        # Parameters (default values)
        self.parameters = parameters if parameters else {}
        
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
        
        # Parameters frame (nếu có parameters)
        self.param_entries = {}  # Initialize empty dict
        self.node_name_for_params = None  # Will be set when converter is launched
        if self.parameters:
            self.create_parameters_frame()
        
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
    
    def create_parameters_frame(self):
        """Tạo frame để edit parameters"""
        # Collapsible parameters frame
        params_frame = ttk.LabelFrame(self, text="Parameters", padding="5")
        params_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Store entry widgets
        self.param_entries = {}
        
        row = 0
        col = 0
        max_cols = 3
        
        for param_name, param_value in self.parameters.items():
            # Create label and entry
            label = ttk.Label(params_frame, text=f"{param_name}:", width=15)
            label.grid(row=row, column=col*2, padx=2, pady=2, sticky=tk.W)
            
            entry = ttk.Entry(params_frame, width=12)
            entry.insert(0, str(param_value))
            entry.grid(row=row, column=col*2+1, padx=2, pady=2, sticky=tk.W)
            
            self.param_entries[param_name] = entry
            
            col += 1
            if col >= max_cols:
                col = 0
                row += 1
        
        # Add one more row if needed
        if col > 0:
            row += 1
        
        # Add Update Parameters button
        update_btn = ttk.Button(
            params_frame,
            text="Update Parameters (Realtime)",
            command=self.update_parameters_realtime,
            state=tk.DISABLED
        )
        update_btn.grid(row=row, column=0, columnspan=max_cols*2, padx=5, pady=5, sticky=tk.EW)
        self.update_params_btn = update_btn
    
    def get_parameters(self):
        """Lấy parameters từ UI entries"""
        params = {}
        for param_name, entry in self.param_entries.items():
            value = entry.get().strip()
            # Try to convert to appropriate type
            original_value = self.parameters[param_name]
            
            # Special handling for array parameters (pol_coeffs, invpol_coeffs)
            if param_name in ["pol_coeffs", "invpol_coeffs"]:
                # Keep as comma-separated string, converter will parse it
                params[param_name] = value
            elif isinstance(original_value, int):
                try:
                    params[param_name] = int(value)
                except ValueError:
                    params[param_name] = original_value
            elif isinstance(original_value, float):
                try:
                    params[param_name] = float(value)
                except ValueError:
                    params[param_name] = original_value
            else:
                params[param_name] = value
        return params
    
    def update_parameters_realtime(self):
        """Update parameters realtime khi converter đang chạy"""
        if not self.converter_process or self.converter_process.poll() is not None:
            messagebox.showwarning("Cảnh báo", "Converter chưa được launch hoặc đã dừng")
            return
        
        try:
            params = self.get_parameters()
            
            # Get node name - try to find from running nodes
            # ROS2 node name format: usually "equidistant_converter" or similar
            node_name = self.node_name_for_params or "equidistant_converter"
            
            # Try to find actual node name from running nodes
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            # Find matching node
            actual_node_name = None
            for line in result.stdout.split('\n'):
                if node_name in line.lower() or self.node_name.replace('_node', '') in line.lower():
                    actual_node_name = line.strip()
                    break
            
            if not actual_node_name:
                # Try common node name patterns
                possible_names = [
                    node_name,
                    f"/{node_name}",
                    f"/equidistant_converter",
                    f"/{self.node_name.replace('_node', '')}"
                ]
                for name in possible_names:
                    test_result = subprocess.run(
                        ['ros2', 'param', 'list', name],
                        capture_output=True,
                        text=True,
                        timeout=1
                    )
                    if test_result.returncode == 0:
                        actual_node_name = name
                        break
            
            if not actual_node_name:
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy node {node_name}.\n"
                    "Vui lòng đảm bảo converter đang chạy."
                )
                return
            
            # Update each parameter
            success_count = 0
            error_count = 0
            
            for param_name, param_value in params.items():
                try:
                    # Use ros2 param set command
                    cmd = ['ros2', 'param', 'set', actual_node_name, param_name, str(param_value)]
                    result = subprocess.run(
                        cmd,
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    
                    if result.returncode == 0:
                        success_count += 1
                        print(f"✓ Updated {param_name} = {param_value}")
                    else:
                        error_count += 1
                        print(f"✗ Failed to update {param_name}: {result.stderr}")
                        
                except Exception as e:
                    error_count += 1
                    print(f"✗ Error updating {param_name}: {e}")
            
            # Show result
            if success_count > 0:
                self.status_label.config(
                    text=f"{self.model_name}: Updated {success_count} parameters",
                    foreground="green"
                )
                if error_count > 0:
                    messagebox.showwarning(
                        "Một phần thành công",
                        f"Đã update {success_count} parameters.\n"
                        f"{error_count} parameters failed."
                    )
            else:
                messagebox.showerror(
                    "Lỗi",
                    f"Không thể update parameters.\n"
                    f"Node: {actual_node_name}\n"
                    f"Lỗi: {result.stderr if 'result' in locals() else 'Unknown'}"
                )
                
        except Exception as e:
            error_msg = f"Lỗi khi update parameters: {e}"
            print(f"✗ {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror("Lỗi", error_msg)
    
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
            
            # Build command with parameters
            cmd_parts = [f"source {setup_script}"]
            
            # Build ros2 run command with parameters
            ros_cmd = f"ros2 run theta_driver {self.node_name}"
            
            # Add parameters if available
            if self.param_entries:
                params = self.get_parameters()
                param_args = []
                for param_name, param_value in params.items():
                    # Skip empty string parameters (ROS2 can't parse them)
                    if isinstance(param_value, str) and not param_value.strip():
                        continue
                    
                    # ROS2 parameter format: --ros-args -p param_name:=value
                    if isinstance(param_value, str):
                        # Quote string values, escape quotes inside
                        escaped_value = param_value.replace('"', '\\"')
                        param_args.append(f"-p {param_name}:=\"{escaped_value}\"")
                    else:
                        param_args.append(f"-p {param_name}:={param_value}")
                
                if param_args:
                    ros_cmd += " --ros-args " + " ".join(param_args)
            
            cmd_parts.append(ros_cmd)
            cmd = " && ".join(cmd_parts)
            
            print(f"Launching {self.model_name} converter with command: {cmd}")
            
            # Use subprocess with proper environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            # Launch process - redirect stderr to stdout for easier debugging
            self.converter_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # Merge stderr into stdout
                universal_newlines=True,
                bufsize=1,
                env=env
            )
            
            # Check if process started successfully
            import time
            time.sleep(0.5)  # Wait a bit for process to start
            
            if self.converter_process.poll() is not None:
                # Process exited immediately - there's an error
                try:
                    stdout, _ = self.converter_process.communicate(timeout=1)
                    error_msg = stdout if stdout else f"Process exited with code {self.converter_process.returncode}"
                except subprocess.TimeoutExpired:
                    error_msg = "Process exited immediately (timeout reading output)"
                
                print(f"✗ {self.model_name} converter failed to start:")
                print(f"  Output: {error_msg}")
                
                messagebox.showerror(
                    "Lỗi Launch",
                    f"Không thể launch {self.model_name} converter:\n\n{error_msg[:500]}"
                )
                self.launch_btn.config(state=tk.NORMAL)
                return
            
            print(f"✓ {self.model_name} converter process started (PID: {self.converter_process.pid})")
            
            self.status_label.config(
                text=f"{self.model_name}: Converter đang chạy",
                foreground="orange"
            )
            self.launch_btn.config(state=tk.DISABLED)
            self.start_btn.config(state=tk.NORMAL)
            
            # Enable update parameters button
            if hasattr(self, 'update_params_btn'):
                self.update_params_btn.config(state=tk.NORMAL)
            
            # Store node name for parameter updates
            # ROS2 node name is usually the executable name
            self.node_name_for_params = self.node_name.replace('_node', '')
            
            print(f"✓ {self.model_name} converter launched successfully (PID: {self.converter_process.pid})")
            
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
            # Debug: log first frame
            if self.frame_count == 0:
                print(f"[{self.model_name}] on_image_received: First frame received, shape: {cv_image.shape}")
            
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
        
        # Disable update parameters button
        if hasattr(self, 'update_params_btn'):
            self.update_params_btn.config(state=tk.DISABLED)
        
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
        self.camera_info_publisher_process = None
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
        
        # Launch Camera Info Publisher
        self.launch_camera_info_btn = ttk.Button(
            control_frame,
            text="Launch Camera Info Publisher",
            command=self.launch_camera_info_publisher
        )
        self.launch_camera_info_btn.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(
            control_frame,
            text="Trạng thái: Chưa kết nối",
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Parameters frame for theta_driver
        params_frame = ttk.LabelFrame(control_frame, text="Theta Driver Parameters", padding="5")
        params_frame.pack(side=tk.LEFT, padx=10)
        
        # use4k parameter
        ttk.Label(params_frame, text="Use 4K:").grid(row=0, column=0, padx=2, pady=2, sticky=tk.W)
        self.use4k_var = tk.BooleanVar(value=True)  # Default: True
        use4k_check = ttk.Checkbutton(params_frame, variable=self.use4k_var)
        use4k_check.grid(row=0, column=1, padx=2, pady=2, sticky=tk.W)
        
        # image_quality parameter
        ttk.Label(params_frame, text="Image Quality:").grid(row=0, column=2, padx=2, pady=2, sticky=tk.W)
        self.image_quality_var = tk.StringVar(value="high")  # Default: high
        quality_combo = ttk.Combobox(params_frame, textvariable=self.image_quality_var, 
                                     values=["raw", "high", "medium", "low", "tiny"], 
                                     width=10, state="readonly")
        quality_combo.grid(row=0, column=3, padx=2, pady=2, sticky=tk.W)
        
        # jpeg_quality parameter
        ttk.Label(params_frame, text="JPEG Quality:").grid(row=0, column=4, padx=2, pady=2, sticky=tk.W)
        self.jpeg_quality_var = tk.StringVar(value="85")  # Default: 85
        jpeg_quality_entry = ttk.Entry(params_frame, textvariable=self.jpeg_quality_var, width=8)
        jpeg_quality_entry.grid(row=0, column=5, padx=2, pady=2, sticky=tk.W)
    
    def create_notebook(self):
        """Tạo notebook với sub-tabs"""
        self.notebook = ttk.Notebook(self)
        
        # Tab Equirectangular (original)
        self.tab_equirect = EquirectangularTab(self.notebook)
        self.notebook.add(self.tab_equirect, text="Equirectangular")
        
        self.notebook.pack(fill=tk.BOTH, expand=True)
    
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
            
            print(f"Launching theta_driver with command: {cmd}")
            
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
                
                print(f"✗ theta_driver failed to start:")
                print(f"  Output: {error_msg}")
                
                messagebox.showerror(
                    "Lỗi Launch",
                    f"Không thể launch theta_driver:\n\n{error_msg[:500]}"
                )
                self.launch_theta_btn.config(state=tk.NORMAL)
                return
            
            print(f"✓ theta_driver process started (PID: {self.theta_driver_process.pid})")
            
            # Update status based on running processes
            if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                self.status_label.config(
                    text="Trạng thái: Theta Driver + Camera Info Publisher đang chạy",
                    foreground="green"
                )
            else:
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
            
            print(f"Launching camera_info_publisher with command: {cmd}")
            
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
                
                print(f"✗ camera_info_publisher failed to start:")
                print(f"  Output: {error_msg}")
                
                messagebox.showerror(
                    "Lỗi Launch",
                    f"Không thể launch camera_info_publisher:\n\n{error_msg[:500]}"
                )
                self.launch_camera_info_btn.config(state=tk.NORMAL)
                return
            
            print(f"✓ camera_info_publisher process started (PID: {self.camera_info_publisher_process.pid})")
            
            # Update status based on running processes
            if self.theta_driver_process and self.theta_driver_process.poll() is None:
                self.status_label.config(
                    text="Trạng thái: Theta Driver + Camera Info Publisher đang chạy",
                    foreground="green"
                )
            else:
                self.status_label.config(
                    text="Trạng thái: Camera Info Publisher đang chạy",
                    foreground="green"
                )
            self.launch_camera_info_btn.config(state=tk.DISABLED)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_camera_info_publisher_process)
            
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể launch camera_info_publisher: {e}")
    
    def check_camera_info_publisher_process(self):
        """Kiểm tra xem camera_info_publisher process còn chạy không"""
        if self.camera_info_publisher_process:
            if self.camera_info_publisher_process.poll() is not None:
                # Update status based on other running processes
                if self.theta_driver_process and self.theta_driver_process.poll() is None:
                    self.status_label.config(
                        text="Trạng thái: Theta Driver đang chạy",
                        foreground="orange"
                    )
                else:
                    self.status_label.config(
                        text="Trạng thái: Camera Info Publisher đã dừng",
                        foreground="orange"
                    )
                self.launch_camera_info_btn.config(state=tk.NORMAL)
    
    def check_theta_driver_process(self):
        """Kiểm tra xem theta_driver process còn chạy không"""
        if self.theta_driver_process:
            if self.theta_driver_process.poll() is not None:
                # Update status based on other running processes
                if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                    self.status_label.config(
                        text="Trạng thái: Camera Info Publisher đang chạy",
                        foreground="green"
                    )
                else:
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
        self.launch_camera_info_btn.config(state=tk.NORMAL)
