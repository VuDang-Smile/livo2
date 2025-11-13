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
    """ROS2 Node để subscribe topics livox/points và livox/imu"""
    
    def __init__(self, callback_lidar, callback_imu, use_pointcloud2=True):
        super().__init__('livox_viewer_subscriber')
        
        # Mặc định dùng PointCloud2 vì launch file msg_MID360_launch.py dùng xfer_format=0
        # Nếu use_pointcloud2=False và có CustomMsg, sẽ dùng CustomMsg
        if use_pointcloud2:
            lidar_msg_type = PointCloud2
            self.get_logger().info('Sử dụng PointCloud2 format cho livox/points')
        else:
            if CustomMsg is None:
                self.get_logger().warn(
                    "Không thể import CustomMsg, fallback về PointCloud2. "
                    "Vui lòng source drive_ws/install/setup.sh trước khi chạy GUI."
                )
                lidar_msg_type = PointCloud2
            else:
                lidar_msg_type = CustomMsg
                self.get_logger().info('Sử dụng CustomMsg format cho livox/points')
        
        # Subscribe topic lidar với đúng message type
        # Topic name: livox/points (không có / ở đầu, theo driver code)
        self.subscription_lidar = self.create_subscription(
            lidar_msg_type,
            'livox/points',
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
        lidar_frame = ttk.LabelFrame(info_frame, text="Lidar Topic: livox/points", padding="5")
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
            workspace_path = Path(__file__).parent.parent / "drive_ws"
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
            
            # Kiểm tra topic type trước khi subscribe
            self.log("Kiểm tra topic type...")
            topic_type_result = subprocess.run(
                ['ros2', 'topic', 'type', 'livox/points'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            use_pointcloud2 = True  # Mặc định dùng PointCloud2
            if topic_type_result.returncode == 0:
                topic_type = topic_type_result.stdout.strip()
                self.log(f"Topic livox/points có type: {topic_type}")
                if 'CustomMsg' in topic_type:
                    use_pointcloud2 = False
                    self.log("Phát hiện CustomMsg format, sẽ subscribe với CustomMsg")
                elif 'PointCloud2' in topic_type or 'sensor_msgs' in topic_type:
                    use_pointcloud2 = True
                    self.log("Phát hiện PointCloud2 format")
                else:
                    self.log(f"⚠️  Topic type không xác định: {topic_type}, dùng PointCloud2 mặc định")
            else:
                self.log("⚠️  Không thể kiểm tra topic type, dùng PointCloud2 mặc định")
                self.log("   (Topic có thể chưa tồn tại, sẽ thử subscribe với PointCloud2)")
            
            # Tạo node subscriber
            self.log("Tạo ROS2 node cho Livox...")
            self.ros_node = LivoxSubscriber(
                self.on_lidar_received,
                self.on_imu_received,
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
            # Kiểm tra cả livox/points và /livox/points (ROS2 có thể normalize)
            if 'livox/points' in result.stdout or '/livox/points' in result.stdout:
                topics_found.append('livox/points')
                self.log("✓ Topic livox/points đã tồn tại")
            else:
                self.log("⚠️  Cảnh báo: Topic livox/points chưa tồn tại")
            
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
                text="Trạng thái: Đang subscribe livox/points và livox/imu...",
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

