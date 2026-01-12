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
                translator.get("dialog.error"),
                translator.get("message.ros2_setup_not_found").replace("{path}", ros2_setup)
            )
            return None
        return ros2_setup
    
    def _start_ros2_process(self, workspace_path, launch_file, process_name, process_attr, monitor_func, status_text, package_name=None):
        """Helper function để start ROS2 process"""
        try:
            setup_script = workspace_path / "install" / "setup.sh"
            if not setup_script.exists():
                messagebox.showerror(
                    translator.get("dialog.error"),
                    translator.get("message.workspace_setup_not_found").replace("{path}", str(setup_script))
                )
                return False
            
            launch_path = workspace_path / launch_file
            if not launch_path.exists():
                messagebox.showerror(
                    translator.get("dialog.error"),
                    translator.get("message.launch_file_not_found").replace("{path}", str(launch_path))
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
            
            self.log(translator.get("log.starting_process").replace("{process_name}", process_name))
            self.log(translator.get("log.command").replace("{cmd}", cmd))
            
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
            
            self.status_label.config(text=translator.get("label.status") + ": " + status_text, foreground="orange")
            return True
            
        except Exception as e:

            error_msg = translator.get("log.error_start_process").replace("{process_name}", process_name).replace("{error}", str(e))
            print(error_msg)
            self.log(f"Lỗi: {error_msg}")
            messagebox.showerror(translator.get("dialog.error"), error_msg)
            return False
    
    def start_livox_driver(self):
        """Start Livox MID 360 driver"""
        self.log(translator.get("log.starting_livox_driver"))
        print("Bắt đầu Livox MID 360 driver...")
        try:
            workspace_path = Path(__file__).parent.parent / "dependencies" / "drive_ws"
            launch_file = Path("src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py")
            print("Bắt đầu Livox MID 360 driver line 2")
            
            if self._start_ros2_process(
                workspace_path,
                launch_file,
                "Livox MID 360 driver",
                "livox_driver_process",
                self.monitor_livox_driver_output,
                "Livox Driver đang chạy"
            ):
                print("Bắt đầu Livox MID 360 driver line 3")
                
                self.start_driver_btn.config(state=tk.DISABLED)
                self.stop_driver_btn.config(state=tk.NORMAL)
                self.start_subscriber_btn.config(state=tk.NORMAL)
                self.start_converter()
                self.update_label_livo_connected(True)
            # Converter button đã enable từ đầu, không cần enable lại
        except Exception as e:
            self.update_label_livo_connected(False)
            print(f"Livox MID 360 driver... error {e}")

        # print("Bắt đầu Livox MID 360 driver...")

        self.log("Ket thuc Livox MID 360 driver...")
    
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
            print(translator.get("log.error_reading_output_from").replace("{process_name}", process_name).replace("{error}", str(e)))
            self.log(translator.get("log.error_reading_output_from").replace("{process_name}", process_name).replace("{error}", str(e)))
        
        # Kiểm tra exit code
        if process.poll() is not None:
            exit_code = process.poll()
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
        """Xử lý khi driver dừng"""
        self.status_label.config(
            text=translator.get("label.status_driver_stopped"),
            foreground="red"
        )
        self.start_driver_btn.config(state=tk.NORMAL)
        self.stop_driver_btn.config(state=tk.DISABLED)
        self.start_subscriber_btn.config(state=tk.DISABLED)
        # Converter có thể chạy độc lập, không tự động stop khi driver dừng
        # if self.converter_process:
        #     self.stop_converter()
        if self.is_ros_running:
            self.stop_ros_subscriber()
    
    
    def _stop_process(self, process, process_name, timeout=5):
        """Helper function để stop process"""
        if not process:
            return
        
        try:
            self.log(translator.get("log.stopping_process").replace("{process_name}", process_name))
            process.terminate()
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                process.kill()
            except:
                pass
        except Exception as e:
            self.log(translator.get("log.error_stopping_process").replace("{process_name}", process_name).replace("{error}", str(e)))
    
    def stop_livox_driver(self):
        """Stop Livox driver"""
        self._stop_process(self.livox_driver_process, "Livox Driver")
        self.livox_driver_process = None
        
        self.status_label.config(
            text=translator.get("label.status_driver_stopped"),
            foreground="red"
        )
        self.start_driver_btn.config(state=tk.NORMAL)
        self.stop_driver_btn.config(state=tk.DISABLED)
        self.start_subscriber_btn.config(state=tk.DISABLED)
        
        # Converter có thể chạy độc lập, không tự động stop khi driver dừng
        # if self.converter_process:
        #     self.stop_converter()
        if self.is_ros_running:
            self.stop_ros_subscriber()
        self.update_label_livo_connected(False)
    
    def start_ros_subscriber(self):
        """Bắt đầu ROS subscriber cho livox topics"""
        print("Bắt đầu ROS subscriber cho Livox...")
        self.log(translator.get("log.starting_ros_subscriber"))
        if self.is_ros_running:
            return
        
        try:
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                self.log(translator.get("log.initializing_ros2"))
                rclpy.init()
            
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
            self.status_label.config(
                text=translator.get("log.status_subscribing"),
                foreground="green"
            )
            self.start_subscriber_btn.config(state=tk.DISABLED)
            self.stop_subscriber_btn.config(state=tk.NORMAL)
            
            self.log(translator.get("log.ros_subscriber_started"))
            
        except Exception as e:
            error_msg = translator.get("log.error_start_subscriber").replace("{error}", str(e))
            print(error_msg)
            self.log(f"✗ Lỗi: {error_msg}")
            import traceback
            traceback.print_exc()
            messagebox.showerror(translator.get("dialog.error"), error_msg)
        self.log(translator.get("log.ending_ros_subscriber"))
    
    def ros_spin(self):
        """Spin ROS node trong thread riêng"""
        try:
            self.log(translator.get("log.ros_spin_thread_started"))
            while rclpy.ok() and self.is_ros_running:
                # Kiểm tra executor và node trước khi sử dụng
                if self.ros_executor is not None and self.ros_node is not None:
                    try:
                        self.ros_executor.spin_once(timeout_sec=0.1)
                    except Exception:
                        # Executor có thể đã bị shutdown, thoát khỏi vòng lặp
                        break
                elif self.ros_node is not None:
                    try:
                        rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                    except Exception:
                        # Node có thể đã bị destroy, thoát khỏi vòng lặp
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
                    text=f"Lỗi: {str(e)[:50]}",
                    foreground="red"
                ))
        finally:
            # Đảm bảo flag được reset khi thread kết thúc
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
        """Dừng ROS subscriber với proper cleanup để tránh segmentation fault"""
        # Set flag để spin thread biết dừng
        self.is_ros_running = False
        
        # Đợi một chút để spin thread thoát khỏi vòng lặp
        import time
        time.sleep(0.2)
        
        # Đợi thread kết thúc nếu có
        if self.ros_thread and self.ros_thread.is_alive():
            # Đợi tối đa 1 giây để thread kết thúc
            self.ros_thread.join(timeout=1.0)
        
        # Sau đó mới shutdown executor và destroy node
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
        
        # Cập nhật UI
        self.status_label.config(
            text=translator.get("log.status_subscriber_stopped"),
            foreground="red"
        )
        self.start_subscriber_btn.config(state=tk.NORMAL)
        self.stop_subscriber_btn.config(state=tk.DISABLED)
        
        self.lidar_info_label.config(text=translator.get("label.no_data_received"))
        self.imu_info_label.config(text=translator.get("label.no_data_received"))
        self.points2_info_label.config(text=translator.get("label.no_data_received"))
        self.log(translator.get("log.ros_subscriber_stopped"))
    
    def start_converter(self):
        """Start Livox Message Converter node (độc lập, không phụ thuộc vào driver)"""
        try:
            self.log(translator.get("log.starting_converter"))
            workspace_path = Path(__file__).parent.parent / "ws"
            launch_file = Path("src/livox_msg_converter/launch/livox_msg_converter.launch.py")
            
            # Kiểm tra xem topic /livox/lidar có tồn tại không (có thể từ driver khác hoặc nguồn khác)
            self.log(translator.get("log.checking_topic_livox_lidar"))
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            topic_exists = '/livox/lidar' in result.stdout or 'livox/lidar' in result.stdout
            if not topic_exists:
                self.log(translator.get("log.warning_topic_not_exists_converter"))
                self.log(translator.get("log.converter_will_wait"))
                self.log(translator.get("log.converter_may_start_driver"))
            else:
                self.log(translator.get("log.topic_exists_converter"))
            
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
                # Không cần restart subscriber nữa vì luôn subscribe /livox/points2
        except Exception as e:
            self.log(f"[Error]: start_converter: {e}")
        self.log(translator.get("log.ending_converter"))
    
    def monitor_converter_output(self):
        """Monitor output từ converter process"""
        self._monitor_process_output(
            self.converter_process,
            "Converter",
            self._handle_converter_stopped
        )
    
    def _handle_converter_stopped(self):
        """Xử lý khi converter dừng"""
        # Cập nhật status dựa trên các process khác đang chạy
        if self.livox_driver_process and self.livox_driver_process.poll() is None:
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
    
    
    def stop_converter(self):
        """Stop Converter node"""
        self._stop_process(self.converter_process, "Converter")
        self.converter_process = None
        
        # Cập nhật status dựa trên các process khác đang chạy
        if self.livox_driver_process and self.livox_driver_process.poll() is None:
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

