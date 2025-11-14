#!/usr/bin/env python3
"""
Livox Tab Module
Chứa LivoxTab và LivoxSubscriber cho Livox Driver 2
"""

import threading
import subprocess
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
    print(f"Lỗi import: {e}")
    import sys
    sys.exit(1)


class LivoxSubscriber(Node):
    """ROS2 Node để subscribe topics /livox/lidar, livox/imu và /livox/points2"""
    
    def __init__(self, callback_lidar, callback_imu, callback_points2=None, use_pointcloud2=True):
        super().__init__('livox_viewer_subscriber')
        
        # Mặc định dùng CustomMsg vì launch file msg_MID360_launch.py dùng xfer_format=1
        # /livox/lidar luôn là CustomMsg format
        if CustomMsg is None:
            self.get_logger().warn(
                "Không thể import CustomMsg, fallback về PointCloud2. "
                "Vui lòng source drive_ws/install/setup.sh trước khi chạy GUI."
            )
            lidar_msg_type = PointCloud2
        else:
            lidar_msg_type = CustomMsg
            self.get_logger().info('Sử dụng CustomMsg format cho /livox/lidar')
        
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
            self.get_logger().info('Đã subscribe topic /livox/points2')
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
            self.get_logger().error(f'Lỗi xử lý points2 data: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class LivoxTab(ttk.Frame):
    """Tab cho Livox Driver 2"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
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
        
        # Nút Start Converter
        self.start_converter_btn = ttk.Button(
            control_frame,
            text="Start Converter",
            command=self.start_converter,
            state=tk.DISABLED
        )
        self.start_converter_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút Stop Converter
        self.stop_converter_btn = ttk.Button(
            control_frame,
            text="Stop Converter",
            command=self.stop_converter,
            state=tk.DISABLED
        )
        self.stop_converter_btn.pack(side=tk.LEFT, padx=5)
        
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
        imu_frame = ttk.LabelFrame(info_frame, text="IMU Topic: livox/imu", padding="5")
        imu_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.imu_info_label = ttk.Label(
            imu_frame,
            text="Chưa nhận dữ liệu",
            font=("Arial", 10)
        )
        self.imu_info_label.pack(anchor=tk.W, padx=5)
        
        # Frame cho Points2 info (từ converter)
        points2_frame = ttk.LabelFrame(info_frame, text="PointCloud2 Topic: /livox/points2", padding="5")
        points2_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.points2_info_label = ttk.Label(
            points2_frame,
            text="Chưa nhận dữ liệu",
            font=("Arial", 10)
        )
        self.points2_info_label.pack(anchor=tk.W, padx=5)
        
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
    
    def _validate_ros2_setup(self):
        """Kiểm tra ROS2 setup có tồn tại không"""
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        if not Path(ros2_setup).exists():
            messagebox.showerror(
                "Lỗi",
                f"Không tìm thấy ROS2 setup tại: {ros2_setup}\n"
                "Vui lòng cài đặt ROS2 Jazzy."
            )
            return None
        return ros2_setup
    
    def _start_ros2_process(self, workspace_path, launch_file, process_name, process_attr, monitor_func, status_text, package_name=None):
        """Helper function để start ROS2 process"""
        try:
            setup_script = workspace_path / "install" / "setup.sh"
            if not setup_script.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy setup.sh tại: {setup_script}\n"
                    "Vui lòng build workspace trước."
                )
                return False
            
            launch_path = workspace_path / launch_file
            if not launch_path.exists():
                messagebox.showerror(
                    "Lỗi",
                    f"Không tìm thấy launch file tại: {launch_path}"
                )
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
            
            self.log(f"Đang khởi động {process_name}...")
            self.log(f"Command: {cmd}")
            
            process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            setattr(self, process_attr, process)
            threading.Thread(target=monitor_func, daemon=True).start()
            
            self.status_label.config(text=f"Trạng thái: {status_text}", foreground="orange")
            return True
            
        except Exception as e:
            error_msg = f"Không thể start {process_name}: {e}"
            self.log(f"Lỗi: {error_msg}")
            messagebox.showerror("Lỗi", error_msg)
            return False
    
    def start_livox_driver(self):
        """Start Livox MID 360 driver"""
        workspace_path = Path(__file__).parent.parent / "drive_ws"
        launch_file = Path("src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py")
        
        if self._start_ros2_process(
            workspace_path,
            launch_file,
            "Livox MID 360 driver",
            "livox_driver_process",
            self.monitor_livox_driver_output,
            "Livox Driver đang chạy"
        ):
            self.start_driver_btn.config(state=tk.DISABLED)
            self.stop_driver_btn.config(state=tk.NORMAL)
            self.start_subscriber_btn.config(state=tk.NORMAL)
            self.start_converter_btn.config(state=tk.NORMAL)
    
    def _monitor_process_output(self, process, process_name, stopped_handler):
        """Helper function để monitor output từ process"""
        if not process:
            return
        
        try:
            for line in iter(process.stdout.readline, ''):
                if not line:
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
            self.log(f"Lỗi khi đọc output từ {process_name}: {e}")
        
        # Kiểm tra exit code
        if process.poll() is not None:
            exit_code = process.poll()
            if exit_code != 0:
                self.log(f"✗ {process_name} đã dừng với exit code: {exit_code}")
            else:
                self.log(f"✓ {process_name} đã dừng bình thường")
            self.after(0, stopped_handler)
    
    def monitor_livox_driver_output(self):
        """Monitor output từ livox driver process"""
        self._monitor_process_output(
            self.livox_driver_process,
            "Driver",
            self._handle_driver_stopped
        )
    
    def _handle_driver_stopped(self):
        """Xử lý khi driver dừng"""
        self.status_label.config(
            text="Trạng thái: Livox Driver đã dừng",
            foreground="red"
        )
        self.start_driver_btn.config(state=tk.NORMAL)
        self.stop_driver_btn.config(state=tk.DISABLED)
        self.start_subscriber_btn.config(state=tk.DISABLED)
        if self.converter_process:
            self.stop_converter()
        if self.is_ros_running:
            self.stop_ros_subscriber()
    
    
    def _stop_process(self, process, process_name, timeout=5):
        """Helper function để stop process"""
        if not process:
            return
        
        try:
            self.log(f"Đang dừng {process_name}...")
            process.terminate()
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except:
                pass
        except Exception as e:
            self.log(f"Lỗi khi dừng {process_name}: {e}")
    
    def stop_livox_driver(self):
        """Stop Livox driver"""
        self._stop_process(self.livox_driver_process, "Livox Driver")
        self.livox_driver_process = None
        
        self.status_label.config(
            text="Trạng thái: Livox Driver đã dừng",
            foreground="red"
        )
        self.start_driver_btn.config(state=tk.NORMAL)
        self.stop_driver_btn.config(state=tk.DISABLED)
        self.start_subscriber_btn.config(state=tk.DISABLED)
        
        if self.converter_process:
            self.stop_converter()
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
            
            # Kiểm tra topic type trước khi subscribe
            self.log("Kiểm tra topic type...")
            topic_type_result = subprocess.run(
                ['ros2', 'topic', 'type', '/livox/lidar'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            use_pointcloud2 = False  # Mặc định dùng CustomMsg vì /livox/lidar là CustomMsg
            if topic_type_result.returncode == 0:
                topic_type = topic_type_result.stdout.strip()
                self.log(f"Topic /livox/lidar có type: {topic_type}")
                if 'CustomMsg' in topic_type:
                    use_pointcloud2 = False
                    self.log("Phát hiện CustomMsg format, sẽ subscribe với CustomMsg")
                elif 'PointCloud2' in topic_type or 'sensor_msgs' in topic_type:
                    use_pointcloud2 = True
                    self.log("⚠️  Phát hiện PointCloud2 format (không mong đợi), sẽ dùng PointCloud2")
                else:
                    self.log(f"⚠️  Topic type không xác định: {topic_type}, dùng CustomMsg mặc định")
            else:
                self.log("⚠️  Không thể kiểm tra topic type, dùng CustomMsg mặc định")
                self.log("   (Topic có thể chưa tồn tại, sẽ thử subscribe với CustomMsg)")
            
            # Tạo node subscriber (có thể subscribe /livox/points2 nếu converter đang chạy)
            self.log("Tạo ROS2 node cho Livox...")
            # Kiểm tra xem converter có đang chạy không
            subscribe_points2 = self.converter_process is not None and self.converter_process.poll() is None
            self.ros_node = LivoxSubscriber(
                self.on_lidar_received,
                self.on_imu_received,
                callback_points2=self.on_points2_received if subscribe_points2 else None,
                use_pointcloud2=use_pointcloud2
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
            # Kiểm tra cả /livox/lidar và livox/lidar (ROS2 có thể normalize)
            if '/livox/lidar' in result.stdout or 'livox/lidar' in result.stdout:
                topics_found.append('/livox/lidar')
                self.log("✓ Topic /livox/lidar đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic /livox/lidar chưa tồn tại")
            
            # Kiểm tra cả livox/imu và /livox/imu (ROS2 có thể normalize)
            if 'livox/imu' in result.stdout or '/livox/imu' in result.stdout:
                topics_found.append('livox/imu')
                self.log("✓ Topic livox/imu đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic livox/imu chưa tồn tại")
            
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
                text="Trạng thái: Đang subscribe /livox/lidar và livox/imu...",
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
        self.points2_info_label.config(text="Chưa nhận dữ liệu")
        self.log("ROS subscriber đã dừng")
    
    def start_converter(self):
        """Start Livox Message Converter node"""
        workspace_path = Path(__file__).parent.parent / "ws"
        launch_file = Path("src/livox_msg_converter/launch/livox_msg_converter.launch.py")
        
        if self._start_ros2_process(
            workspace_path,
            launch_file,
            "Livox Message Converter",
            "converter_process",
            self.monitor_converter_output,
            "Converter đang chạy"
        ):
            self.start_converter_btn.config(state=tk.DISABLED)
            self.stop_converter_btn.config(state=tk.NORMAL)
            
            # Nếu subscriber đang chạy, cần restart để subscribe /livox/points2
            if self.is_ros_running:
                self.log("Đang restart subscriber để subscribe /livox/points2...")
                self.stop_ros_subscriber()
                self.after(1000, self.start_ros_subscriber)
    
    def monitor_converter_output(self):
        """Monitor output từ converter process"""
        self._monitor_process_output(
            self.converter_process,
            "Converter",
            self._handle_converter_stopped
        )
    
    def _handle_converter_stopped(self):
        """Xử lý khi converter dừng"""
        self.status_label.config(
            text="Trạng thái: Converter đã dừng",
            foreground="red"
        )
        self.start_converter_btn.config(state=tk.NORMAL)
        self.stop_converter_btn.config(state=tk.DISABLED)
        self.points2_info_label.config(text="Chưa nhận dữ liệu")
        self.points2_count = 0
    
    
    def stop_converter(self):
        """Stop Converter node"""
        self._stop_process(self.converter_process, "Converter")
        self.converter_process = None
        
        self.status_label.config(
            text="Trạng thái: Converter đã dừng",
            foreground="red"
        )
        self.start_converter_btn.config(state=tk.NORMAL)
        self.stop_converter_btn.config(state=tk.DISABLED)
        self.points2_info_label.config(text="Chưa nhận dữ liệu")
        self.points2_count = 0
    
    def on_points2_received(self, msg):
        """Callback khi nhận được PointCloud2 từ /livox/points2"""
        self.points2_count += 1
        
        # Cập nhật UI trong main thread
        info_text = (
            f"Đã nhận {self.points2_count} point clouds | "
            f"Width: {msg.width} | Height: {msg.height} | "
            f"Point step: {msg.point_step} bytes | "
            f"Type: PointCloud2 (từ converter)"
        )
        
        self.after(0, lambda: self.points2_info_label.config(text=info_text))
        
        # Cập nhật status mỗi 30 message
        if self.points2_count % 30 == 0:
            self.after(0, lambda: self.status_label.config(
                text=f"Trạng thái: Đã nhận {self.lidar_count} lidar, {self.imu_count} IMU, {self.points2_count} points2 messages",
                foreground="green"
            ))

