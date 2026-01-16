#!/usr/bin/env python3
"""
Livox Tab Module
Chứa LivoxTab và LivoxSubscriber cho Livox Driver 2
"""

import threading
import subprocess
import os
import queue
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2, Imu
try:
    from livox_ros_driver2.msg import CustomMsg
except ImportError:
    CustomMsg = None

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, scrolledtext
except ImportError as e:
    print(translator.get("log.import_error", "Import error: {error}").replace("{error}", str(e)))
    import sys
    sys.exit(1)

from languages.translate_engine import translator


class LivoxSubscriber(Node):
    """ROS2 Node để subscribe topics /livox/lidar, livox/imu và /livox/points2"""
    
    def __init__(self, callback_lidar, callback_imu, callback_points2=None, use_pointcloud2=True):
        super().__init__('livox_viewer_subscriber')
        
        # Mặc định dùng CustomMsg vì launch file msg_MID360_launch.py dùng xfer_format=1
        # /livox/lidar luôn là CustomMsg format
        if CustomMsg is None:
            self.get_logger().warn(
                translator.get("log.cannot_import_custommsg")
            )
            lidar_msg_type = PointCloud2
        else:
            lidar_msg_type = CustomMsg
            self.get_logger().info(translator.get("log.using_custommsg_format"))
        
        # Subscribe topic lidar với đúng message type
        # Topic name: /livox/lidar (đã được remap từ livox/points trong launch file)
        self.subscription_lidar = self.create_subscription(
            lidar_msg_type,
            '/livox/lidar',
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
        
        # Subscribe topic /livox/points2 (PointCloud2 từ converter)
        if callback_points2 is not None:
            self.subscription_points2 = self.create_subscription(
                PointCloud2,
                '/livox/points2',
                self.points2_callback,
                10
            )
            self.get_logger().info(translator.get("log.subscribed_points2"))
        else:
            self.subscription_points2 = None
        
        self.callback_lidar = callback_lidar
        self.callback_imu = callback_imu
        self.callback_points2 = callback_points2
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
            self.get_logger().error(translator.get("log.error_processing_lidar").replace("{error}", str(e)))
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
            self.get_logger().error(translator.get("log.error_processing_imu").replace("{error}", str(e)))
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def points2_callback(self, msg):
        """Callback khi nhận được PointCloud2 từ converter"""
        try:
            if not hasattr(self, '_points2_count'):
                self._points2_count = 0
            self._points2_count += 1
            # Không log để tránh spam log
            
            if self.callback_points2:
                self.callback_points2(msg)
        except Exception as e:
            self.get_logger().error(translator.get("log.error_processing_points2").replace("{error}", str(e)))
            import traceback
            self.get_logger().error(traceback.format_exc())


class LivoxTab(ttk.Frame):
    """Tab cho Livox Driver 2"""
    def __init__(self, log_callback, update_label_livo_connected=None):
        super().__init__()

        self.log = log_callback
        self.update_label_livo_connected = update_label_livo_connected
        # Biến trạng thái
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.livox_driver_process = None
        self.converter_process = None
        self.is_ros_running = False
        self.lidar_count = 0
        self.imu_count = 0
        self.points2_count = 0
        # Thread safety locks
        self._ros_lock = threading.Lock()
        self._process_lock = threading.Lock()  # Lock để bảo vệ truy cập process objects
        
        # Queue để giao tiếp giữa threads và main thread (thread-safe)
        self.ui_queue = queue.Queue()
        
        # Tạo UI
        self.create_widgets()
        
        # Start queue processor để xử lý UI updates từ threads
        self._process_ui_queue()
    
    def create_widgets(self):
        """Tạo các widget cho tab Livox"""
        
        # Frame điều khiển
        control_frame = ttk.Frame(self, padding="10")
        control_frame.pack(fill=tk.X)
        
        # Nút Start Livox Driver
        self.start_driver_btn = ttk.Button(
            control_frame,
            text=translator.get("button.start_livox_mid360"),
            command=self.start_livox_driver
        )
        self.start_driver_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Livox Driver
        self.stop_driver_btn = ttk.Button(
            control_frame,
            text=translator.get("button.stop_livox_driver"),
            command=self.stop_livox_driver,
            state=tk.DISABLED
        )
        self.stop_driver_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start ROS Subscriber
        self.start_subscriber_btn = ttk.Button(
            control_frame,
            text=translator.get("button.start_subscriber"),
            command=self.start_ros_subscriber,
            state=tk.DISABLED
        )
        self.start_subscriber_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Subscriber
        self.stop_subscriber_btn = ttk.Button(
            control_frame,
            text=translator.get("button.stop_subscriber"),
            command=self.stop_ros_subscriber,
            state=tk.DISABLED
        )
        self.stop_subscriber_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Start Converter (độc lập, không phụ thuộc vào driver)
        self.start_converter_btn = ttk.Button(
            control_frame,
            text=translator.get("button.start_converter"),
            command=self.start_converter,
            state=tk.NORMAL  # Enable ngay từ đầu
        )
        self.start_converter_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Converter
        self.stop_converter_btn = ttk.Button(
            control_frame,
            text=translator.get("button.stop_converter"),
            command=self.stop_converter,
            state=tk.DISABLED
        )
        self.stop_converter_btn.pack(side=tk.LEFT, padx=5)
        
        # Label trạng thái
        self.status_label = ttk.Label(
            control_frame,
            text=translator.get("label.status_not_connected"),
            foreground="red"
        )
        self.status_label.pack(side=tk.LEFT, padx=20)
        
        # Frame thông tin topics
        info_frame = ttk.LabelFrame(self, text=translator.get("label.topic_information"), padding="10")
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Frame cho Lidar info
        lidar_frame = ttk.LabelFrame(info_frame, text=translator.get("label.lidar_topic_full"), padding="5")
        lidar_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.lidar_info_label = ttk.Label(
            lidar_frame,
            text=translator.get("label.no_data_received"),
            font=("Arial", 10)
        )
        self.lidar_info_label.pack(anchor=tk.W, padx=5)
        
        # Frame cho IMU info
        imu_frame = ttk.LabelFrame(info_frame, text=translator.get("label.imu_topic_full"), padding="5")
        imu_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.imu_info_label = ttk.Label(
            imu_frame,
            text=translator.get("label.no_data_received"),
            font=("Arial", 10)
        )
        self.imu_info_label.pack(anchor=tk.W, padx=5)
        
        # Frame cho Points2 info (từ converter)
        points2_frame = ttk.LabelFrame(info_frame, text=translator.get("label.points2_topic"), padding="5")
        points2_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.points2_info_label = ttk.Label(
            points2_frame,
            text=translator.get("label.no_data_received"),
            font=("Arial", 10)
        )
        self.points2_info_label.pack(anchor=tk.W, padx=5)
        
        # Text area để hiển thị log
        log_frame = ttk.LabelFrame(self, text=translator.get("label.log_label"), padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame,
            height=10,
            wrap=tk.WORD,
            state=tk.DISABLED
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def log(self, message):
        """Thêm message vào log - thread-safe qua queue"""
        # Kiểm tra xem đang ở main thread hay không
        if threading.current_thread() is threading.main_thread():
            # Nếu là main thread, gọi trực tiếp
            self._log_impl(message)
        else:
            # Nếu là thread khác, đưa vào queue
            self.ui_queue.put(('log', message))
    
    def _log_impl(self, message):
        """Implementation của log - chỉ được gọi từ main thread"""
        try:
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"{message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        except Exception as e:
            # Nếu có lỗi, in ra console
            print(f"Error in log: {e}")
    
    def _process_ui_queue(self):
        """Xử lý queue để update UI từ threads - chỉ chạy trong main thread"""
        try:
            while True:
                try:
                    action, *args = self.ui_queue.get_nowait()
                    if action == 'log':
                        self._log_impl(args[0])
                    elif action == 'ui_update':
                        callback = args[0]
                        callback()
                except queue.Empty:
                    break
        except Exception as e:
            print(f"Error processing UI queue: {e}")
        
        # Schedule lại sau 50ms để tiếp tục xử lý queue
        self.after(50, self._process_ui_queue)
    
    def _schedule_ui_update(self, callback):
        """Schedule UI update từ thread - thread-safe, luôn dùng queue để đảm bảo an toàn"""
        # Luôn dùng queue để đảm bảo thread-safe, không gọi trực tiếp
        self.ui_queue.put(('ui_update', callback))
    
    def _validate_ros2_setup(self):
        """Kiểm tra ROS2 setup có tồn tại không"""
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        if not Path(ros2_setup).exists():
            # Schedule messagebox từ main thread
            error_msg = translator.get("message.ros2_setup_not_found").replace("{path}", ros2_setup)
            self._schedule_ui_update(lambda: messagebox.showerror(
                translator.get("dialog.error"),
                error_msg
            ))
            return None
        return ros2_setup
    
    def _start_ros2_process(self, workspace_path, launch_file, process_name, process_attr, monitor_func, status_text, package_name=None):
        """Helper function để start ROS2 process"""
        import threading
        thread_id = threading.get_ident()
        self.log(f"[DEBUG] _start_ros2_process called for {process_name} from thread {thread_id}")
        try:
            setup_script = workspace_path / "install" / "setup.sh"
            if not setup_script.exists():
                # Schedule messagebox từ main thread
                error_msg = translator.get("message.workspace_setup_not_found").replace("{path}", str(setup_script))
                self._schedule_ui_update(lambda: messagebox.showerror(
                    translator.get("dialog.error"),
                    error_msg
                ))
                return False
            
            launch_path = workspace_path / launch_file
            if not launch_path.exists():
                # Schedule messagebox từ main thread
                error_msg = translator.get("message.launch_file_not_found").replace("{path}", str(launch_path))
                self._schedule_ui_update(lambda: messagebox.showerror(
                    translator.get("dialog.error"),
                    error_msg
                ))
                return False
            
            ros2_setup = self._validate_ros2_setup()
            if not ros2_setup:
                return False
            
            # Xác định package name từ launch file path
            if package_name is None:
                # Tìm package name từ path: src/package_name/launch/...
                parts = launch_file.parts
                if len(parts) >= 2 and parts[0] == "src":
                    package_name = parts[1]
                else:
                    # Fallback: dùng tên thư mục chứa launch file
                    package_name = launch_path.parent.parent.name
            
            launch_filename = launch_path.name
            cmd = f"source {ros2_setup} && source {setup_script} && ros2 launch {package_name} {launch_filename}"
            
            self.log(translator.get("log.starting_process").replace("{process_name}", process_name))
            self.log(translator.get("log.command").replace("{cmd}", cmd))
            
            # Use subprocess with proper environment
            env = os.environ.copy()
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            # Sử dụng preexec_fn để tạo process group mới, tránh conflict với ROS2 context
            # Điều này giúp tránh segmentation fault khi launch ROS2 nodes
            try:
                preexec_fn = os.setsid if hasattr(os, 'setsid') else None
            except:
                preexec_fn = None
            
            self.log(f"[DEBUG] Creating subprocess for {process_name} (thread {thread_id})")
            process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                env=env,
                preexec_fn=preexec_fn
            )
            
            self.log(f"[DEBUG] Process created with PID: {process.pid} (thread {thread_id})")
            setattr(self, process_attr, process)
            
            # Kiểm tra process đã start thành công chưa (chờ một chút)
            import time
            time.sleep(0.5)
            
            # Kiểm tra process status với lock để tránh race condition với monitor thread
            with self._process_lock:
                process_status = process.poll()
            
            if process_status is not None:
                # Process đã dừng ngay sau khi start
                try:
                    stdout, _ = process.communicate(timeout=1)
                    error_msg = stdout if stdout else f"Process exited with code {process.returncode}"
                except subprocess.TimeoutExpired:
                    error_msg = "Process exited immediately (timeout reading output)"
                
                self.log(f"✗ {process_name} failed to start:")
                self.log(f"  Output: {error_msg}")
                # Schedule messagebox từ main thread
                dialog_msg = translator.get("log.process_cannot_start", "{process_name} cannot start: {error}").replace("{process_name}", process_name).replace("{error}", error_msg[:200])
                self._schedule_ui_update(lambda: messagebox.showerror(
                    translator.get("dialog.error"),
                    dialog_msg
                ))
                return False
            
            # Start monitor thread
            monitor_thread = threading.Thread(target=monitor_func, daemon=True, name=f"{process_name}Monitor")
            self.log(f"[DEBUG] Starting monitor thread for {process_name} (caller thread {thread_id})")
            monitor_thread.start()
            self.log(f"[DEBUG] Monitor thread started with ID: {monitor_thread.ident}")
            
            self._schedule_ui_update(lambda: self.status_label.config(text=translator.get("label.status") + ": " + status_text, foreground="orange"))
            self.log(f"[DEBUG] _start_ros2_process completed successfully for {process_name}")
            return True
            
        except Exception as e:

            error_msg = translator.get("log.error_start_process").replace("{process_name}", process_name).replace("{error}", str(e))
            print(error_msg)
            self.log(translator.get("log.error_prefix", "Error: {message}").replace("{message}", error_msg))
            # Schedule messagebox từ main thread
            self._schedule_ui_update(lambda: messagebox.showerror(translator.get("dialog.error"), error_msg))
            return False
    
    def start_livox_driver(self):
        """Start Livox MID 360 driver"""
        import threading
        thread_id = threading.get_ident()
        self.log(f"[DEBUG] start_livox_driver called from thread {thread_id}")
        self.log(translator.get("log.starting_livox_driver"))
        # Debug print - có thể bỏ comment nếu cần debug
        # print(f"[DEBUG] Starting Livox MID 360 driver... (thread {thread_id})")
        try:
            workspace_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
            launch_file = Path("src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py")
            # print(f"[DEBUG] Starting Livox MID 360 driver line 2 (thread {thread_id})")
            
            if self._start_ros2_process(
                workspace_path,
                launch_file,
                "Livox MID 360 driver",
                "livox_driver_process",
                self.monitor_livox_driver_output,
                "Livox Driver đang chạy"
            ):
                # print(f"[DEBUG] Starting Livox MID 360 driver line 3 (thread {thread_id})")
                self.log(f"[DEBUG] Driver process started, PID: {self.livox_driver_process.pid if self.livox_driver_process else 'None'}")
                
                # UI updates phải được gọi từ main thread để tránh segmentation fault
                # Sử dụng queue để schedule UI updates từ main thread
                self.log(f"[DEBUG] Scheduling UI updates from thread {thread_id}")
                self._schedule_ui_update(lambda: self.start_driver_btn.config(state=tk.DISABLED))
                self._schedule_ui_update(lambda: self.stop_driver_btn.config(state=tk.NORMAL))
                self._schedule_ui_update(lambda: self.start_subscriber_btn.config(state=tk.NORMAL))
                self.log(f"[DEBUG] UI updates scheduled successfully")
                
                # Chờ một chút để driver process ổn định trước khi start converter
                import time
                self.log(f"[DEBUG] Waiting 2.0s for driver to stabilize (thread {thread_id})")
                time.sleep(2.0)  # Tăng delay để đảm bảo driver ổn định hoàn toàn
                
                # Kiểm tra driver process vẫn còn chạy trước khi start converter
                # Sử dụng lock để tránh race condition với monitor thread đang đọc stdout
                self.log(f"[DEBUG] Checking driver process status (thread {thread_id})")
                driver_running = False
                with self._process_lock:
                    if self.livox_driver_process:
                        try:
                            driver_running = (self.livox_driver_process.poll() is None)
                            self.log(f"[DEBUG] Driver process status: {'running' if driver_running else 'stopped'}")
                        except Exception as e:
                            self.log(translator.get("log.error_check_driver_status", "⚠️ Error checking driver process status: {error}").replace("{error}", str(e)))
                            driver_running = False
                
                if driver_running:
                    self.log(f"[DEBUG] Starting converter thread (thread {thread_id})")
                    # Gọi start_converter trong thread riêng với proper error handling
                    def safe_start_converter():
                        try:
                            import time
                            # Thêm delay nhỏ trong thread để đảm bảo mọi thứ ổn định
                            time.sleep(1.0)
                            
                            # Kiểm tra lại driver process trước khi start converter với lock
                            driver_still_running = False
                            with self._process_lock:
                                if self.livox_driver_process:
                                    try:
                                        driver_still_running = (self.livox_driver_process.poll() is None)
                                    except Exception as e:
                                        self.log(translator.get("log.error_check_driver_thread", "⚠️ Error checking driver process in thread: {error}").replace("{error}", str(e)))
                                        driver_still_running = False
                            
                            if not driver_still_running:
                                self.log(translator.get("log.driver_stopped_no_converter", "⚠️ Driver process has stopped, not starting converter"))
                                return
                            
                            # Gọi start_converter với protection
                            try:
                                self.start_converter()
                            except Exception as e:
                                error_msg = translator.get("log.error_start_converter", "Error starting converter: {error}").replace("{error}", str(e))
                                self.log(translator.get("log.error_prefix", "Error: {message}").replace("{message}", error_msg))
                                import traceback
                                self.log(traceback.format_exc())
                        except Exception as e:
                            error_msg = translator.get("log.error_safe_start_converter", "Error in safe_start_converter thread: {error}").replace("{error}", str(e))
                            self.log(translator.get("log.error_prefix", "Error: {message}").replace("{message}", error_msg))
                            import traceback
                            self.log(traceback.format_exc())
                    
                    # Start thread với name để dễ debug
                    converter_thread = threading.Thread(target=safe_start_converter, daemon=True, name="ConverterStarter")
                    converter_thread.start()
                else:
                    self.log(translator.get("log.driver_stopped_no_converter", "⚠️ Driver process has stopped, not starting converter"))
                
                # UI update callback cũng phải được gọi từ main thread
                if self.update_label_livo_connected:
                    self._schedule_ui_update(lambda: self.update_label_livo_connected(True))
            # Converter button đã enable từ đầu, không cần enable lại
        except Exception as e:
            # UI update callback cũng phải được gọi từ main thread
            if self.update_label_livo_connected:
                self._schedule_ui_update(lambda: self.update_label_livo_connected(False))
            print(f"Livox MID 360 driver... error {e}")

        # print("Bắt đầu Livox MID 360 driver...")

        self.log("Ket thuc Livox MID 360 driver...")
    
    def _monitor_process_output(self, process, process_name, stopped_handler):
        """Helper function để monitor output từ process với improved error handling"""
        if not process:
            return
        
        try:
            # Sử dụng timeout để tránh blocking vô hạn
            import select
            import sys
            
            # Đọc stdout với improved error handling
            while True:
                # Kiểm tra process còn chạy không với lock (chỉ check, không block)
                process_stopped = False
                with self._process_lock:
                    try:
                        process_stopped = (process.poll() is not None)
                    except:
                        process_stopped = True
                
                if process_stopped:
                    # Process đã dừng
                    break
                
                # Đọc stdout ngoài lock để tránh deadlock
                try:
                    line = process.stdout.readline()
                    if not line:
                        # EOF reached
                        break
                except Exception as e:
                    # Lỗi khi đọc stdout, có thể process đã đóng stdout
                    self.log(translator.get("log.error_reading_stdout", "⚠️ Error reading stdout from {process_name}: {error}").replace("{process_name}", process_name).replace("{error}", str(e)))
                    break
                
                line = line.strip()
                if line:
                    # Log output với prefix
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed', 'cannot', 'unable']):
                        self.log(f"❌ {process_name} ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️  {process_name} WARNING: {line}")
                    else:
                        self.log(f"{process_name}: {line}")
        except Exception as e:
            error_msg = translator.get("log.error_reading_output_from").replace("{process_name}", process_name).replace("{error}", str(e))
            print(error_msg)
            self.log(error_msg)
            import traceback
            self.log(traceback.format_exc())
        
        # Kiểm tra exit code với lock để tránh race condition
        with self._process_lock:
            exit_code = process.poll()
            if exit_code is not None:
                if exit_code != 0:
                    self.log(translator.get("log.process_stopped_exit_code").replace("{process_name}", process_name).replace("{exit_code}", str(exit_code)))
                else:
                    self.log(translator.get("log.process_stopped_normal").replace("{process_name}", process_name))
                self.after(0, stopped_handler)
    
    def monitor_livox_driver_output(self):
        """Monitor output từ livox driver process"""
        self._monitor_process_output(
            self.livox_driver_process,
            "Driver",
            self._handle_driver_stopped
        )
    
    def _handle_driver_stopped(self):
        """Xử lý khi driver dừng - được gọi từ monitor thread, cần schedule UI updates"""
        # UI updates phải được gọi từ main thread
        self.after(0, lambda: self.status_label.config(
            text=translator.get("label.status_driver_stopped"),
            foreground="red"
        ))
        self.after(0, lambda: self.start_driver_btn.config(state=tk.NORMAL))
        self.after(0, lambda: self.stop_driver_btn.config(state=tk.DISABLED))
        self.after(0, lambda: self.start_subscriber_btn.config(state=tk.DISABLED))
        # Converter có thể chạy độc lập, không tự động stop khi driver dừng
        # if self.converter_process:
        #     self.stop_converter()
        if self.is_ros_running:
            self.stop_ros_subscriber()
    
    
    def _stop_process(self, process, process_name, timeout=2):
        """Helper function để stop process với timeout ngắn hơn để không block UI"""
        if not process:
            return
        
        try:
            self.log(translator.get("log.stopping_process").replace("{process_name}", process_name))
            # Terminate process
            with self._process_lock:
                if process.poll() is None:  # Process vẫn đang chạy
                    process.terminate()
            
            # Chờ với timeout ngắn hơn để không block UI quá lâu
            try:
                process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                # Nếu timeout, kill process ngay lập tức
                try:
                    with self._process_lock:
                        if process.poll() is None:
                            process.kill()
                            process.wait(timeout=0.5)  # Chờ ngắn sau khi kill
                except:
                    pass
        except Exception as e:
            self.log(translator.get("log.error_stopping_process").replace("{process_name}", process_name).replace("{error}", str(e)))
    
    def stop_livox_driver(self):
        """Stop Livox driver - chạy trong thread để không block UI"""
        # Update UI ngay lập tức
        self.after(0, lambda: self.status_label.config(
            text=translator.get("label.status_driver_stopped"),
            foreground="red"
        ))
        self.after(0, lambda: self.start_driver_btn.config(state=tk.NORMAL))
        self.after(0, lambda: self.stop_driver_btn.config(state=tk.DISABLED))
        self.after(0, lambda: self.start_subscriber_btn.config(state=tk.DISABLED))
        
        if self.update_label_livo_connected:
            self.after(0, lambda: self.update_label_livo_connected(False))
        
        # Stop operations chạy trong thread riêng để không block UI
        def stop_driver_thread():
            try:
                # Stop ROS subscriber trước nếu đang chạy
                if self.is_ros_running:
                    self.stop_ros_subscriber()
                
                # Stop driver process
                self._stop_process(self.livox_driver_process, "Livox Driver")
                
                # Clear process reference với lock
                with self._process_lock:
                    self.livox_driver_process = None
                
                self.log(translator.get("log.livox_driver_stopped", "✓ Livox driver stopped"))
            except Exception as e:
                self.log(translator.get("log.error_stop_driver", "⚠️ Error stopping driver: {error}").replace("{error}", str(e)))
                import traceback
                self.log(traceback.format_exc())
        
        # Chạy stop trong thread riêng
        threading.Thread(target=stop_driver_thread, daemon=True, name="DriverStop").start()
    
    def start_ros_subscriber(self):
        """Bắt đầu ROS subscriber cho livox topics"""
        # Debug print - có thể bỏ comment nếu cần debug
        # print(translator.get("log.starting_ros_subscriber"))
        self.log(translator.get("log.starting_ros_subscriber"))
        if self.is_ros_running:
            return
        
        try:
            # Khởi tạo ROS2 một cách an toàn với proper error handling
            try:
                if not rclpy.ok():
                    self.log(translator.get("log.initializing_ros2"))
                    rclpy.init()
                else:
                    self.log(translator.get("log.ros2_already_initialized", "ROS2 already initialized"))
            except RuntimeError as e:
                # ROS2 có thể đã được khởi tạo từ nơi khác
                if "already initialized" in str(e).lower() or "context already exists" in str(e).lower():
                    self.log(translator.get("log.ros2_already_initialized_ignore", "ROS2 already initialized (ignoring error)"))
                else:
                    # Lỗi khác, raise lại
                    raise
            except Exception as e:
                error_msg = translator.get("log.error_init_ros2", "Error initializing ROS2: {error}").replace("{error}", str(e))
                self.log(f"✗ {error_msg}")
                raise
            
            # Kiểm tra topic type trước khi subscribe
            self.log(translator.get("log.checking_topic_type"))
            topic_type_result = subprocess.run(
                ['ros2', 'topic', 'type', '/livox/lidar'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            use_pointcloud2 = False  # Mặc định dùng CustomMsg vì /livox/lidar là CustomMsg
            if topic_type_result.returncode == 0:
                topic_type = topic_type_result.stdout.strip()
                self.log(translator.get("log.topic_type").replace("{type}", topic_type))
                if 'CustomMsg' in topic_type:
                    use_pointcloud2 = False
                    self.log(translator.get("log.detected_custommsg"))
                elif 'PointCloud2' in topic_type or 'sensor_msgs' in topic_type:
                    use_pointcloud2 = True
                    self.log(translator.get("log.detected_pointcloud2"))
                else:
                    self.log(translator.get("log.unknown_topic_type").replace("{type}", topic_type))
            else:
                self.log(translator.get("log.cannot_check_topic_type"))
                self.log(translator.get("log.topic_may_not_exist"))
            
            # Tạo node subscriber
            # Luôn subscribe /livox/points2 nếu converter có publish, không cần restart subscriber
            self.log(translator.get("log.creating_ros2_node"))
            self.ros_node = LivoxSubscriber(
                self.on_lidar_received,
                self.on_imu_received,
                callback_points2=self.on_points2_received,
                use_pointcloud2=use_pointcloud2
            )
            
            # Tạo executor và add node
            self.log(translator.get("log.creating_executor"))
            self.ros_executor = SingleThreadedExecutor()
            self.ros_executor.add_node(self.ros_node)
            
            # Kiểm tra topics có tồn tại không
            self.log(translator.get("log.checking_topics"))
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            topics_found = []
            # Kiểm tra cả /livox/lidar và livox/lidar (ROS2 có thể normalize)
            if '/livox/lidar' in result.stdout or 'livox/lidar' in result.stdout:
                topics_found.append('/livox/lidar')
                self.log(translator.get("log.topic_livox_lidar_exists"))
            else:
                self.log(translator.get("log.warning_topic_livox_lidar_not_exists"))
            
            # Kiểm tra cả livox/imu và /livox/imu (ROS2 có thể normalize)
            if 'livox/imu' in result.stdout or '/livox/imu' in result.stdout:
                topics_found.append('livox/imu')
                self.log(translator.get("log.topic_livox_imu_exists"))
            else:
                self.log(translator.get("log.warning_topic_livox_imu_not_exists"))
            
            if not topics_found:
                self.log(translator.get("log.no_topics_found"))
            
            # Chạy ROS trong thread riêng
            self.log(translator.get("log.starting_ros_thread"))
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            
            self.is_ros_running = True
            self.lidar_count = 0
            self.imu_count = 0
            # UI updates phải được gọi từ main thread
            self._schedule_ui_update(lambda: self.status_label.config(
                text=translator.get("log.status_subscribing"),
                foreground="green"
            ))
            self._schedule_ui_update(lambda: self.start_subscriber_btn.config(state=tk.DISABLED))
            self._schedule_ui_update(lambda: self.stop_subscriber_btn.config(state=tk.NORMAL))
            
            self.log(translator.get("log.ros_subscriber_started"))
            
        except Exception as e:
            error_msg = translator.get("log.error_start_subscriber").replace("{error}", str(e))
            print(error_msg)
            self.log(translator.get("log.error_prefix", "Error: {message}").replace("{message}", error_msg))
            import traceback
            traceback.print_exc()
            # Schedule messagebox từ main thread
            self._schedule_ui_update(lambda: messagebox.showerror(translator.get("dialog.error"), error_msg))
        self.log(translator.get("log.ending_ros_subscriber"))
    
    def ros_spin(self):
        """Spin ROS node trong thread riêng với improved thread safety"""
        try:
            self.log(translator.get("log.ros_spin_thread_started"))
            while rclpy.ok() and self.is_ros_running:
                # Sử dụng lock để đảm bảo thread safety khi truy cập executor/node
                with self._ros_lock:
                    # Kiểm tra executor và node trước khi sử dụng
                    if self.ros_executor is not None and self.ros_node is not None:
                        try:
                            self.ros_executor.spin_once(timeout_sec=0.1)
                        except Exception as e:
                            # Executor có thể đã bị shutdown, thoát khỏi vòng lặp
                            self.log(f"⚠️  Executor error trong ros_spin: {e}")
                            break
                    elif self.ros_node is not None:
                        try:
                            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                        except Exception as e:
                            # Node có thể đã bị destroy, thoát khỏi vòng lặp
                            self.log(f"⚠️  Node error trong ros_spin: {e}")
                            break
                    else:
                        # Không có executor hoặc node, thoát
                        break
        except Exception as e:
            error_msg = translator.get("log.error_ros_spin").replace("{error}", str(e))
            self.log(f"✗ {error_msg}")
            import traceback
            traceback.print_exc()
            # Cập nhật UI để báo lỗi
            if self.is_ros_running:
                self.after(0, lambda: self.status_label.config(
                    text=translator.get("log.error_prefix", "Error: {message}").replace("{message}", str(e)[:50]),
                    foreground="red"
                ))
        finally:
            # Đảm bảo flag được reset khi thread kết thúc
            with self._ros_lock:
                self.is_ros_running = False
    
    def on_lidar_received(self, msg):
        """Callback khi nhận được lidar data"""
        self.lidar_count += 1
        
        # Xử lý cả CustomMsg và PointCloud2
        if CustomMsg and isinstance(msg, CustomMsg):
            # CustomMsg format
            info_text = (
                translator.get("log.received_lidar_pointclouds").replace("{count}", str(self.lidar_count)) + " | " +
                translator.get("log.point_num").replace("{num}", str(msg.point_num)) + " | " +
                "Type: CustomMsg | " +
                translator.get("log.lidar_id").replace("{id}", str(msg.lidar_id))
            )
        else:
            # PointCloud2 format
            info_text = (
                translator.get("log.received_lidar_pointclouds").replace("{count}", str(self.lidar_count)) + " | " +
                f"Width: {msg.width} | Height: {msg.height} | " +
                translator.get("log.point_step_bytes").replace("{step}", str(msg.point_step)) + " | " +
                "Type: PointCloud2"
            )
        
        # Cập nhật UI trong main thread
        self.after(0, lambda: self.lidar_info_label.config(text=info_text))
        
        # Cập nhật status mỗi 30 message
        if self.lidar_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=translator.get("log.status_received_messages").replace("{lidar}", str(self.lidar_count)).replace("{imu}", str(self.imu_count)),
                foreground="green"
            ))
    
    def on_imu_received(self, msg):
        """Callback khi nhận được IMU data"""
        self.imu_count += 1
        
        # Cập nhật UI trong main thread
        angular_vel = msg.angular_velocity
        linear_accel = msg.linear_acceleration
        
        angular_text = translator.get("log.angular").replace("{x:.3f}", f"{angular_vel.x:.3f}").replace("{y:.3f}", f"{angular_vel.y:.3f}").replace("{z:.3f}", f"{angular_vel.z:.3f}")
        linear_text = translator.get("log.linear").replace("{x:.3f}", f"{linear_accel.x:.3f}").replace("{y:.3f}", f"{linear_accel.y:.3f}").replace("{z:.3f}", f"{linear_accel.z:.3f}")
        self.after(0, lambda: self.imu_info_label.config(
            text=translator.get("log.received_imu_messages").replace("{count}", str(self.imu_count)) + " | " +
                 angular_text + " | " +
                 linear_text
        ))
        
        # Cập nhật status mỗi 30 message
        if self.imu_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=translator.get("log.status_received_messages").replace("{lidar}", str(self.lidar_count)).replace("{imu}", str(self.imu_count)),
                foreground="green"
            ))
    
    def stop_ros_subscriber(self):
        """Dừng ROS subscriber với proper cleanup - chạy trong thread để không block UI"""
        # Set flag ngay lập tức để spin thread biết dừng
        with self._ros_lock:
            if not self.is_ros_running:
                # Đã dừng rồi, không cần làm gì
                return
            self.is_ros_running = False
        
        # Update UI ngay lập tức để không block
        self.after(0, lambda: self.status_label.config(
            text=translator.get("log.status_subscriber_stopped"),
            foreground="red"
        ))
        self.after(0, lambda: self.start_subscriber_btn.config(state=tk.NORMAL))
        self.after(0, lambda: self.stop_subscriber_btn.config(state=tk.DISABLED))
        
        self.after(0, lambda: self.lidar_info_label.config(text=translator.get("label.no_data_received")))
        self.after(0, lambda: self.imu_info_label.config(text=translator.get("label.no_data_received")))
        self.after(0, lambda: self.points2_info_label.config(text=translator.get("label.no_data_received")))
        
        # Cleanup chạy trong thread riêng để không block UI
        def cleanup_ros():
            import time
            # Đợi một chút để spin thread thoát khỏi vòng lặp
            time.sleep(0.1)  # Giảm từ 0.3 xuống 0.1
            
            # Đợi thread kết thúc nếu có với timeout ngắn hơn
            if self.ros_thread and self.ros_thread.is_alive():
                self.ros_thread.join(timeout=1.0)  # Giảm từ 2.0 xuống 1.0
                if self.ros_thread.is_alive():
                    self.log(translator.get("log.ros_thread_timeout", "⚠️ ROS thread did not end after timeout, continuing cleanup"))
            
            # Sau đó mới shutdown executor và destroy node với lock
            with self._ros_lock:
                if self.ros_executor:
                    try:
                        self.ros_executor.shutdown()
                    except Exception as e:
                        self.log(translator.get("log.error_shutdown_executor").replace("{error}", str(e)))
                    finally:
                        self.ros_executor = None
                
                if self.ros_node:
                    try:
                        self.ros_node.destroy_node()
                    except Exception as e:
                        self.log(translator.get("log.error_destroy_node").replace("{error}", str(e)))
                    finally:
                        self.ros_node = None
            
            self.ros_thread = None
            self.log(translator.get("log.ros_subscriber_stopped"))
        
        # Chạy cleanup trong thread riêng
        threading.Thread(target=cleanup_ros, daemon=True, name="ROSCleanup").start()
    
    def start_converter(self):
        """Start Livox Message Converter node (độc lập, không phụ thuộc vào driver)"""
        # Kiểm tra xem converter đã đang chạy chưa
        if self.converter_process and self.converter_process.poll() is None:
            self.log(translator.get("log.converter_already_running", "⚠️ Converter is already running, not starting again"))
            return
        
        try:
            self.log(translator.get("log.starting_converter"))
            workspace_path = Path(__file__).parent.parent / "ws"
            launch_file = Path("src/livox_msg_converter/launch/livox_msg_converter.launch.py")
            
            # Bỏ qua việc check topic để tránh crash với subprocess.run()
            # Converter sẽ tự động chờ topic xuất hiện
            self.log(translator.get("log.converter_will_wait"))
            
            # Gọi trực tiếp _start_ros2_process() trong thread hiện tại
            # Không tạo nested thread để tránh crash
            if self._start_ros2_process(
                workspace_path,
                launch_file,
                "Livox Message Converter",
                "converter_process",
                self.monitor_converter_output,
                "Converter đang chạy (độc lập)"
            ):
                self.start_converter_btn.config(state=tk.DISABLED)
                self.stop_converter_btn.config(state=tk.NORMAL)
            
        except Exception as e:
            error_msg = translator.get("log.error_start_converter", "Error starting converter: {error}").replace("{error}", str(e))
            self.log(f"✗ {error_msg}")
            import traceback
            self.log(traceback.format_exc())
        finally:
            self.log(translator.get("log.ending_converter"))
    
    def monitor_converter_output(self):
        """Monitor output từ converter process"""
        self._monitor_process_output(
            self.converter_process,
            "Converter",
            self._handle_converter_stopped
        )
    
    def _handle_converter_stopped(self):
        """Xử lý khi converter dừng - được gọi từ monitor thread, cần schedule UI updates"""
        # Cập nhật status dựa trên các process khác đang chạy - phải được gọi từ main thread
        def update_ui():
            # Kiểm tra driver process với lock để tránh race condition
            driver_running = False
            with self._process_lock:
                if self.livox_driver_process:
                    try:
                        driver_running = (self.livox_driver_process.poll() is None)
                    except:
                        driver_running = False
            
            if driver_running:
                self.status_label.config(
                    text=translator.get("log.status_driver_running_converter_stopped"),
                    foreground="orange"
                )
            else:
                self.status_label.config(
                    text=translator.get("log.status_converter_stopped"),
                    foreground="red"
                )
            self.start_converter_btn.config(state=tk.NORMAL)
            self.stop_converter_btn.config(state=tk.DISABLED)
            self.points2_info_label.config(text=translator.get("label.no_data_received"))
            self.points2_count = 0
        
        self.after(0, update_ui)
    
    
    def stop_converter(self):
        """Stop Converter node - chạy trong thread để không block UI"""
        # Update UI ngay lập tức
        def update_ui():
            # Kiểm tra driver process với lock để tránh race condition
            driver_running = False
            with self._process_lock:
                if self.livox_driver_process:
                    try:
                        driver_running = (self.livox_driver_process.poll() is None)
                    except:
                        driver_running = False
            
            if driver_running:
                self.status_label.config(
                    text=translator.get("log.status_driver_running_converter_stopped"),
                    foreground="orange"
                )
            else:
                self.status_label.config(
                    text=translator.get("log.status_converter_stopped"),
                    foreground="red"
                )
            self.start_converter_btn.config(state=tk.NORMAL)
            self.stop_converter_btn.config(state=tk.DISABLED)
            self.points2_info_label.config(text=translator.get("label.no_data_received"))
            self.points2_count = 0
        
        self.after(0, update_ui)
        
        # Stop process chạy trong thread riêng
        def stop_converter_thread():
            try:
                self._stop_process(self.converter_process, "Converter")
                with self._process_lock:
                    self.converter_process = None
                self.log(translator.get("log.converter_stopped", "✓ Converter stopped"))
            except Exception as e:
                self.log(translator.get("log.error_stop_converter", "⚠️ Error stopping converter: {error}").replace("{error}", str(e)))
        
        threading.Thread(target=stop_converter_thread, daemon=True, name="ConverterStop").start()
    
    def on_points2_received(self, msg):
        """Callback khi nhận được PointCloud2 từ /livox/points2"""
        self.points2_count += 1
        
        # Cập nhật UI trong main thread
        info_text = (
            translator.get("log.received_points2_pointclouds").replace("{count}", str(self.points2_count)) + " | " +
            f"Width: {msg.width} | Height: {msg.height} | " +
            translator.get("log.point_step_bytes").replace("{step}", str(msg.point_step)) + " | " +
            translator.get("log.type_pointcloud2_from_converter")
        )
        
        self.after(0, lambda: self.points2_info_label.config(text=info_text))
        
        # Cập nhật status mỗi 30 message
        if self.points2_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=translator.get("log.status_received_all_messages").replace("{lidar}", str(self.lidar_count)).replace("{imu}", str(self.imu_count)).replace("{points2}", str(self.points2_count)),
                foreground="green"
            ))

