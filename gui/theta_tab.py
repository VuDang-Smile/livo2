#!/usr/bin/env python3
"""
Theta Tab Module
Chứa ThetaTab với Notebook sub-tabs cho các camera models
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
import numpy as np

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
                print(f'[SingleImageSubscriber] ✅ Callback được gọi lần đầu từ {self.topic_name}')
            self._callback_count += 1
            
            # Log ngay frame đầu tiên để confirm callback được gọi
            if self._callback_count == 1:
                print(f'[SingleImageSubscriber] ✅ Đã nhận frame đầu tiên từ {self.topic_name}')
                print(f'[SingleImageSubscriber]   Encoding: {msg.encoding}, Size: {msg.width}x{msg.height}, Data size: {len(msg.data)} bytes')
            
            if self._callback_count % 30 == 0:
                self.get_logger().info(f'Đã nhận {self._callback_count} frames từ {self.topic_name}')
                print(f'[SingleImageSubscriber] Đã nhận {self._callback_count} frames từ {self.topic_name}')
            
            # Debug: log encoding (bao gồm JPEG)
            if self._callback_count == 1:
                encoding_info = f'Encoding: {msg.encoding}, Size: {msg.width}x{msg.height}'
                encoding_lower = msg.encoding.lower()
                if 'jpeg' in encoding_lower:
                    encoding_info += ' (JPEG compressed)'
                print(f'[SingleImageSubscriber] {encoding_info}')
            
            # Xử lý JPEG encoding đặc biệt vì cv_bridge không hỗ trợ
            # Hỗ trợ các encoding: "jpeg", "JPEG", "jpeg2000", etc.
            encoding_lower = msg.encoding.lower()
            if 'jpeg' in encoding_lower:
                # Decode JPEG data thủ công bằng OpenCV
                # msg.data có thể là bytes, list, tuple, hoặc array.array
                # Chuyển đổi sang bytes trước
                if isinstance(msg.data, bytes):
                    jpeg_bytes = msg.data
                elif isinstance(msg.data, (list, tuple)):
                    jpeg_bytes = bytes(msg.data)
                elif hasattr(msg.data, 'tobytes'):
                    # Xử lý array.array hoặc numpy array
                    jpeg_bytes = msg.data.tobytes()
                else:
                    # Thử convert sang bytes
                    try:
                        jpeg_bytes = bytes(msg.data)
                    except Exception as e:
                        raise RuntimeError(f"Không thể convert data sang bytes. Type: {type(msg.data)}, Error: {e}")
                
                # Kiểm tra data không rỗng
                if not jpeg_bytes or len(jpeg_bytes) == 0:
                    raise RuntimeError("JPEG data rỗng")
                
                # Debug: log data size
                if self._callback_count == 1:
                    print(f'[SingleImageSubscriber] JPEG data size: {len(jpeg_bytes)} bytes')
                
                # Chuyển đổi data từ bytes sang numpy array
                jpeg_data = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                
                # Decode JPEG
                cv_image_bgr = cv2.imdecode(jpeg_data, cv2.IMREAD_COLOR)
                
                if cv_image_bgr is None:
                    # Debug: log thêm thông tin khi decode fail
                    error_detail = f"Data size: {len(jpeg_bytes)}, First 10 bytes: {jpeg_bytes[:10] if len(jpeg_bytes) >= 10 else jpeg_bytes}"
                    raise RuntimeError(f"Không thể decode JPEG data. {error_detail}")
                
                # Chuyển từ BGR sang RGB (OpenCV decode trả về BGR)
                cv_image = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
                
                # Debug: log image shape
                if self._callback_count == 1:
                    print(f'[SingleImageSubscriber] JPEG decoded, CV Image shape: {cv_image.shape}')
            else:
                # Sử dụng cv_bridge cho các encoding khác (rgb8, bgr8, etc.)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                
                # Debug: log image shape
                if self._callback_count == 1:
                    print(f'[SingleImageSubscriber] CV Image shape: {cv_image.shape}')
            
            # Truyền thêm encoding info nếu callback hỗ trợ
            if callable(self.callback):
                # Kiểm tra xem callback có nhận encoding không bằng cách xem số lượng parameters
                import inspect
                try:
                    sig = inspect.signature(self.callback)
                    param_count = len(sig.parameters)
                    # Nếu callback có 2 parameters trở lên, truyền encoding
                    if param_count >= 2:
                        self.callback(cv_image, msg.encoding)
                    else:
                        self.callback(cv_image)
                except (ValueError, TypeError):
                    # Nếu không thể inspect, thử gọi với encoding trước
                    try:
                        self.callback(cv_image, msg.encoding)
                    except TypeError:
                        # Nếu callback không nhận encoding, gọi không có encoding
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
            # ROS2 node name format: usually "perspective_converter" or similar
            node_name = self.node_name_for_params or self.node_name.replace('_node', '')
            
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
    """Tab cho equirectangular (original) - chỉ subscribe, không có converter
    Hỗ trợ cả RGB8 và JPEG encoding"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # State
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_running = False
        self.frame_count = 0
        self.current_image = None
        self.current_encoding = None  # Lưu encoding hiện tại
        self.topic_var = None  # Sẽ được tạo trong create_widgets
        
        # UI
        self.create_widgets()
    
    def create_widgets(self):
        """Tạo UI widgets"""
        # Control frame
        control_frame = ttk.Frame(self, padding="5")
        control_frame.pack(fill=tk.X)
        
        # Topic selection frame
        topic_frame = ttk.Frame(control_frame)
        topic_frame.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(topic_frame, text="Topic:").pack(side=tk.LEFT, padx=2)
        self.topic_var = tk.StringVar(value="/image_raw")
        topic_entry = ttk.Entry(topic_frame, textvariable=self.topic_var, width=20)
        topic_entry.pack(side=tk.LEFT, padx=2)
        
        refresh_topics_btn = ttk.Button(
            topic_frame,
            text="🔄 Refresh Topics",
            command=self.refresh_topics_list,
            width=15
        )
        refresh_topics_btn.pack(side=tk.LEFT, padx=2)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_subscriber,
            state=tk.NORMAL  # Enable ngay từ đầu để có thể subscribe từ bag hoặc driver khác
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
            text="Equirectangular: Chưa có ảnh (Có thể subscribe từ bag hoặc theta driver)\n💡 Tip: Click 'Refresh Topics' để xem các topics có sẵn",
            font=("Arial", 9)
        )
        self.status_label.pack(fill=tk.X, padx=5, pady=5)
    
    def refresh_topics_list(self):
        """Refresh danh sách topics và hiển thị các image topics"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                all_topics = [line.strip() for line in result.stdout.split('\n') if line.strip()]
                image_topics = [t for t in all_topics if 'image' in t.lower() or 'camera' in t.lower()]
                
                if image_topics:
                    print("📡 Các image topics có sẵn:")
                    for topic in image_topics:
                        print(f"   - {topic}")
                    
                    # Hiển thị trong messagebox
                    topics_text = "\n".join(image_topics)
                    messagebox.showinfo(
                        "Image Topics",
                        f"Các image topics có sẵn:\n\n{topics_text}\n\n"
                        f"Bạn có thể copy topic name và paste vào ô Topic ở trên."
                    )
                else:
                    print("⚠️  Không tìm thấy image topics")
                    messagebox.showinfo(
                        "Không có Image Topics",
                        "Không tìm thấy image topics nào.\n\n"
                        "Các topics có sẵn:\n" + "\n".join(all_topics[:10])
                    )
            else:
                messagebox.showerror("Lỗi", f"Không thể lấy danh sách topics: {result.stderr}")
        except Exception as e:
            messagebox.showerror("Lỗi", f"Lỗi khi refresh topics: {e}")
    
    def start_subscriber(self):
        """Start ROS subscriber"""
        if self.is_running:
            return
        
        try:
            if not rclpy.ok():
                print("Khởi tạo ROS2...")
                rclpy.init()
            
            # Lấy topic name từ UI
            topic_name = self.topic_var.get().strip()
            if not topic_name:
                messagebox.showerror("Lỗi", "Vui lòng nhập topic name")
                return
            
            # Đảm bảo topic name có / prefix
            if not topic_name.startswith('/'):
                topic_name = '/' + topic_name
            
            print(f"Tạo ROS2 node subscriber cho {topic_name}...")
            self.ros_node = SingleImageSubscriber(
                topic_name,
                self.on_image_received,
                'equirectangular_subscriber'
            )
            
            print("Tạo executor...")
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topic có tồn tại không (không bắt buộc, có thể subscribe từ bag sau)
            print(f"Kiểm tra topic {topic_name}...")
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            # Kiểm tra ROS2 domain ID
            ros_domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
            print(f"ROS_DOMAIN_ID hiện tại: {ros_domain_id}")
            
            if topic_name in result.stdout:
                print(f"✓ Topic {topic_name} đã tồn tại")
                # Kiểm tra topic type và hz
                try:
                    type_result = subprocess.run(
                        ['ros2', 'topic', 'type', topic_name],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    if type_result.returncode == 0:
                        print(f"✓ Topic type: {type_result.stdout.strip()}")
                    
                    # Kiểm tra hz (chỉ 1 lần, không chờ lâu)
                    hz_result = subprocess.run(
                        ['ros2', 'topic', 'hz', topic_name, '--window', '5'],
                        capture_output=True,
                        text=True,
                        timeout=6
                    )
                    if hz_result.returncode == 0 and 'average rate' in hz_result.stdout:
                        print(f"✓ Topic hz: {hz_result.stdout.strip()}")
                except Exception as e:
                    print(f"⚠️  Không thể kiểm tra topic info: {e}")
            else:
                print(f"⚠️  Cảnh báo: Topic {topic_name} chưa tồn tại")
                print("   Bạn có thể:")
                print("   - Play bag file: ros2 bag play <bag_file>")
                print("   - Launch theta driver từ GUI")
                print("   - Hoặc subscribe sẽ đợi topic xuất hiện")
                print("   - Hoặc click 'Refresh Topics' để xem các topics có sẵn")
                print("Các topics có sẵn:")
                print(result.stdout)
                
                # Gợi ý các topic image có thể có
                image_topics = [line.strip() for line in result.stdout.split('\n') 
                               if 'image' in line.lower() or 'camera' in line.lower()]
                if image_topics:
                    print("\n💡 Các topic image có thể có:")
                    for topic in image_topics:
                        print(f"   - {topic}")
            
            print("Khởi động ROS thread...")
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_running = True
            self.frame_count = 0
            
            # Cập nhật status dựa trên topic có tồn tại không
            topic_exists = topic_name in result.stdout if 'result' in locals() else False
            if topic_exists:
                status_text = f"Equirectangular: Đang subscribe {topic_name} (topic đã có sẵn)"
            else:
                status_text = f"Equirectangular: Đang subscribe {topic_name} (đợi topic từ bag/driver)..."
            
            self.status_label.config(
                text=status_text,
                foreground="green"
            )
            self.start_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            print("✓ ROS subscriber đã khởi động")
            
            # Kiểm tra sau 3 giây xem có nhận được messages không
            self.after(3000, self.check_subscriber_status)
            
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
            spin_count = 0
            while rclpy.ok() and self.is_running:
                if self.ros_executor is not None:
                    self.ros_executor.spin_once(timeout_sec=0.1)
                else:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                
                # Debug: log mỗi 100 spins để confirm thread đang chạy
                spin_count += 1
                if spin_count % 100 == 0:
                    print(f"[Equirectangular] ROS spin đang chạy ({spin_count} spins)")
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
    
    def check_subscriber_status(self):
        """Kiểm tra xem subscriber có nhận được messages không"""
        if not self.is_running:
            return
        
        if self.frame_count == 0:
            # Chưa nhận được frame nào
            print("⚠️  Chưa nhận được frame nào sau 3 giây")
            print("   Kiểm tra:")
            print("   1. Bag file có đang play không?")
            print("   2. Topic name có đúng không? (kiểm tra: ros2 topic list)")
            print("   3. ROS_DOMAIN_ID có khớp không? (bag và GUI phải cùng domain)")
            
            # Kiểm tra topic lại
            try:
                result = subprocess.run(
                    ['ros2', 'topic', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                topic_name = self.topic_var.get().strip()
                if not topic_name.startswith('/'):
                    topic_name = '/' + topic_name
                
                if topic_name in result.stdout:
                    print(f"   ✓ Topic {topic_name} đang tồn tại")
                    # Kiểm tra hz
                    try:
                        hz_result = subprocess.run(
                            ['ros2', 'topic', 'hz', topic_name, '--window', '3'],
                            capture_output=True,
                            text=True,
                            timeout=4
                        )
                        if hz_result.returncode == 0:
                            print(f"   Topic hz: {hz_result.stdout.strip()}")
                    except:
                        pass
                else:
                    print(f"   ✗ Topic {topic_name} không tồn tại")
                    print("   Các topics có sẵn:")
                    for line in result.stdout.split('\n'):
                        if line.strip() and ('image' in line.lower() or 'camera' in line.lower()):
                            print(f"     - {line.strip()}")
            except Exception as e:
                print(f"   Lỗi khi kiểm tra topic: {e}")
            
            # Cập nhật status
            self.status_label.config(
                text="Equirectangular: ⚠️ Chưa nhận được frames (kiểm tra topic name và ROS_DOMAIN_ID)",
                foreground="orange"
            )
            
            # Kiểm tra lại sau 5 giây nữa
            self.after(5000, self.check_subscriber_status)
        else:
            print(f"✓ Subscriber đang hoạt động: đã nhận {self.frame_count} frames")
    
    def on_image_received(self, cv_image, encoding=None):
        """Callback khi nhận được ảnh (hỗ trợ JPEG và RGB8)"""
        try:
            self.current_image = cv_image
            self.current_encoding = encoding or "rgb8"  # Mặc định là rgb8
            self.frame_count += 1
            
            # Debug: log mỗi 30 frame
            if self.frame_count % 30 == 0:
                encoding_info = f" ({encoding})" if encoding and 'jpeg' in encoding.lower() else ""
                print(f"✓ Equirectangular: Đã nhận {self.frame_count} frames{encoding_info}")
            
            # Cập nhật UI trong main thread
            self.after(0, self.update_image_display, cv_image)
            
            # Cập nhật status mỗi 30 frame với encoding info
            if self.frame_count % 30 == 0:
                encoding_display = f" | {encoding}" if encoding and 'jpeg' in encoding.lower() else ""
                topic_name = self.topic_var.get().strip() if self.topic_var else "/image_raw"
                if not topic_name.startswith('/'):
                    topic_name = '/' + topic_name
                self.after(0, lambda: self.status_label.config(
                    text=f"Equirectangular: {self.frame_count} frames | {topic_name}{encoding_display}",
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
            
            # Hiển thị encoding info nếu là JPEG
            encoding_info = ""
            if self.current_encoding and 'jpeg' in self.current_encoding.lower():
                encoding_info = f" | {self.current_encoding.upper()}"
            
            self.status_label.config(
                text=f"Equirectangular: {img_width}x{img_height} | "
                     f"Hiển thị: {new_width}x{new_height} | "
                     f"Scale: {scale:.2f} | Frames: {self.frame_count}{encoding_info}",
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
        
        # Default parameters
        self.use4k = tk.BooleanVar(value=True)
        self.image_quality = tk.StringVar(value="high")
        self.jpeg_compress_value = tk.IntVar(value=85)
        self.fps_limit = tk.DoubleVar(value=0.0)
        
        # UI
        self.create_control_panel()
        self.create_notebook()
    
    def create_control_panel(self):
        """Tạo control panel ở top"""
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Parameters frame
        params_frame = ttk.LabelFrame(control_frame, text="Theta Driver Parameters", padding="5")
        params_frame.pack(side=tk.LEFT, padx=5)
        
        # Use4K checkbox
        use4k_check = ttk.Checkbutton(
            params_frame,
            text="Use 4K",
            variable=self.use4k
        )
        use4k_check.grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        
        # Image quality dropdown
        ttk.Label(params_frame, text="Image Quality:").grid(row=0, column=1, padx=5, pady=2, sticky=tk.W)
        quality_combo = ttk.Combobox(
            params_frame,
            textvariable=self.image_quality,
            values=["raw", "high", "medium", "low", "tiny"],
            state="readonly",
            width=10
        )
        quality_combo.grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        
        # JPEG compress value
        ttk.Label(params_frame, text="JPEG Quality:").grid(row=0, column=3, padx=5, pady=2, sticky=tk.W)
        jpeg_spin = ttk.Spinbox(
            params_frame,
            from_=0,
            to=100,
            textvariable=self.jpeg_compress_value,
            width=8
        )
        jpeg_spin.grid(row=0, column=4, padx=5, pady=2, sticky=tk.W)
        ttk.Label(params_frame, text="(0=disabled)").grid(row=0, column=5, padx=2, pady=2, sticky=tk.W)
        
        # FPS limit
        ttk.Label(params_frame, text="FPS Limit:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        fps_spin = ttk.Spinbox(
            params_frame,
            from_=0.0,
            to=60.0,
            textvariable=self.fps_limit,
            width=8,
            increment=1.0
        )
        fps_spin.grid(row=1, column=1, padx=5, pady=2, sticky=tk.W)
        ttk.Label(params_frame, text="(0=no limit)").grid(row=1, column=2, padx=2, pady=2, sticky=tk.W)
        
        # Launch Theta Driver button
        self.launch_theta_btn = ttk.Button(
            control_frame,
            text="Launch Theta Driver",
            command=self.launch_theta_driver
        )
        self.launch_theta_btn.pack(side=tk.LEFT, padx=10)
        
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
            
            # Build command with parameters
            use4k_val = "true" if self.use4k.get() else "false"
            image_quality_val = self.image_quality.get()
            jpeg_compress_val = self.jpeg_compress_value.get()
            fps_limit_val = self.fps_limit.get()
            
            cmd = (
                f"source {setup_script} && "
                f"ros2 run theta_driver theta_driver_node --ros-args "
                f"-p use4k:={use4k_val} "
                f"-p image_quality:={image_quality_val} "
                f"-p jpeg_compress_value:={jpeg_compress_val} "
                f"-p fps_limit:={fps_limit_val}"
            )
            
            print(f"Launching theta_driver with command: {cmd}")
            
            self.theta_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.status_label.config(
                text=f"Trạng thái: Theta Driver đang chạy (4K:{use4k_val}, Quality:{image_quality_val}, JPEG:{jpeg_compress_val}, FPS:{fps_limit_val})",
                foreground="orange"
            )
            self.launch_theta_btn.config(state=tk.DISABLED)
            # Không cần enable start_btn ở đây vì đã enable từ đầu để có thể subscribe từ bag
            # Cập nhật status của equirectangular tab nếu đang subscribe
            if self.tab_equirect.is_running:
                self.tab_equirect.status_label.config(
                    text="Equirectangular: Đang subscribe /image_raw (topic từ theta driver)",
                    foreground="green"
                )
            
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
