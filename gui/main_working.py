import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime
import uuid
import subprocess
import os
import sys
import requests
import threading
import time
import zipfile
import shutil
import json
import signal
from pathlib import Path
from datetime import timezone

# Thêm project root vào sys.path để có thể import các module
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from languages.translate_engine import Translator
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS, BACKEND_HOST

# Heartbeat configuration
HEARTBEAT_INTERVAL_SECONDS = 10

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. Pose publishing will be disabled.")


class PoseSubscriber(Node):
    """ROS2 Node để subscribe pose và gửi lên backend"""
    
    def __init__(self, callback_func, node_name='pose_publisher_subscriber'):
        super().__init__(node_name)
        self.callback_func = callback_func
        
        # Subscribe vào topic /localization (Odometry từ transform_fusion)
        # Fallback về /Odometry nếu không có
        self.subscription = self.create_subscription(
            Odometry,
            '/localization',
            self.pose_callback,
            10
        )
        
        # Thử subscribe thêm /Odometry làm backup
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/Odometry',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Pose subscriber đã subscribe vào /localization và /Odometry')
    
    def pose_callback(self, msg):
        """Callback khi nhận được pose mới"""
        try:
            # Extract position và orientation
            position = {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "z": float(msg.pose.pose.position.z)
            }
            
            orientation = {
                "x": float(msg.pose.pose.orientation.x),
                "y": float(msg.pose.pose.orientation.y),
                "z": float(msg.pose.pose.orientation.z),
                "w": float(msg.pose.pose.orientation.w)
            }
            
            # Convert timestamp từ ROS2 time sang datetime object (UTC)
            ros_time = msg.header.stamp
            # ROS2 time có sec và nanosec
            timestamp_sec = ros_time.sec + ros_time.nanosec / 1e9
            timestamp_dt = datetime.fromtimestamp(timestamp_sec, tz=timezone.utc)
            # Backend expect ISO format string
            timestamp_iso = timestamp_dt.isoformat()
            
            # Gọi callback để gửi lên backend
            if self.callback_func:
                self.callback_func(position, orientation, timestamp_iso)
                
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý pose: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


class WorkerInterface:
    def __init__(self, root):
        self.root = root
        self.root.title("Worker Interface | System Monitoring")
        self.root.geometry("1100x700")
        
        self.is_running = False
        
        # Paths
        self.workspace_path = Path(project_root) / "ws"
        self.log_path = self.workspace_path / "src" / "FAST-LIVO2" / "Log"
        self.default_map_root = self.log_path / "fastloc_map"
        self.qr_detect_path = self.log_path / "QR_detect.json"
        self.backend_base_url = BACKEND_HOST.rstrip("/")
        
        # Localization process
        self.loc_process = None
        self.is_localization_running = False
        
        # Livox driver process
        self.livox_driver_process = None
        self.is_livox_driver_running = False
        
        # Livox device connection info
        self.livox_ip = self._get_livox_ip_from_config()  # IP từ config hoặc mặc định
        self.is_livox_device_connected = False  # Trạng thái kết nối thiết bị (ping)
        self.device_connection_timer_id = None
        self.device_connection_update_interval = 5000  # Cập nhật mỗi 5 giây
        
        # Pose publishing
        self.pose_subscriber = None
        self.pose_executor = None
        self.pose_thread = None
        self.vehicle_id = None
        self.is_pose_publishing = False
        self.pose_counter = 0  # Đếm số lần nhận pose
        self.pose_send_interval = 20  # Gửi lên backend mỗi 20 lần
        self.last_pose_received_time = None  # Thời gian nhận pose cuối cùng
        self.pose_first_received_logged = False  # Đã log lần đầu nhận pose chưa
        
        # Heartbeat
        self.heartbeat_timer_id = None
        
        # Translator for multi-language support
        self.translator = Translator('en')
        self.current_lang = 'en'
        
        # Set title with translator
        self.root.title(self.translator.get('title.worker_interface', 'Worker Interface | System Monitoring'))
        
        # Setup language button FIRST (before main container)
        self.setup_language_button()

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        # Pack AFTER language button to ensure language button is on top
        self.main_container.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # --- PHẦN 1: SIDEBAR (Bên trái) ---
        self.sidebar = tk.Frame(self.main_container, width=320, relief=tk.SUNKEN, bd=1)
        self.sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        self.sidebar.pack_propagate(False) # Giữ cố định độ rộng cho Sidebar

        # --- PHẦN 2: SYSTEM LOG (Bên phải - Chiếm trọn phần còn lại) ---
        self.log_container = tk.Frame(self.main_container, padx=5, pady=5)
        self.log_container.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Cài đặt các thành phần bên trong Sidebar
        self.setup_movement_control()
        self.setup_options()
        self.setup_vehicle_info()
        self.setup_map_info()
        self.setup_device_connection_info()
        self.setup_pose_display()

        # Cài đặt phần Log bên phải
        self.setup_log_area()
        
        # Update UI texts after all components are set up
        self.update_ui_texts()
        
        # Cập nhật trạng thái vehicle thành online khi khởi động
        # Gọi sau khi setup_vehicle_info để có vehicle_id
        if self.vehicle_id:
            self.update_vehicle_status("online", refresh_info=True)
            # Bắt đầu heartbeat tự động
            self.start_heartbeat()
        
        # Tự động tải map và QR khi khởi động
        self.download_map_and_qr()
        
        # Bắt đầu monitoring kết nối thiết bị định kỳ
        self.start_device_connection_monitoring()
        
        # Thêm handler khi đóng cửa sổ
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def setup_language_button(self):
        """Setup language toggle button"""
        lang_frame = tk.Frame(self.root)
        lang_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button = tk.Button(
            lang_frame,
            text=f"🌐 {lang_names.get(self.current_lang, 'English')}",
            font=("Arial", 9, "bold"),
            fg="#0066cc",
            bg="#e8e8e8",
            activebackground="#d0d0d0",
            activeforeground="#0066cc",
            relief=tk.RAISED,
            bd=1,
            padx=12,
            pady=6,
            cursor="hand2",
            command=self.toggle_language
        )
        self.lang_button.pack(side=tk.RIGHT)
    
    def toggle_language(self):
        """Toggle between English and Japanese"""
        if self.current_lang == 'en':
            self.change_language('jp')
        else:
            self.change_language('en')
    
    def change_language(self, lang_code):
        """Change language and update UI"""
        self.current_lang = lang_code
        self.translator.switch_language(lang_code)
        
        lang_names = {'en': 'English', 'jp': '日本語'}
        self.lang_button.config(text=f"🌐 {lang_names.get(lang_code, 'English')}")
        
        self.update_ui_texts()
    
    def update_ui_texts(self):
        """Update all UI texts based on current language"""
        self.root.title(self.translator.get('title.worker_interface', 'Worker Interface | System Monitoring'))
        
        # Update movement control frame
        if hasattr(self, 'btn_start'):
            self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'))
        if hasattr(self, 'btn_stop'):
            self.btn_stop.config(text=self.translator.get('button.stop', 'STOP'))
        if hasattr(self, 'status_text'):
            current_text = self.status_text.cget('text')
            if 'Đang tải map' in current_text or 'Loading map' in current_text:
                self.status_text.config(text=self.translator.get('label.loading_map', 'System: Loading map...'))
            elif 'Đã dừng' in current_text or 'Stopped' in current_text:
                self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
            elif 'Localization đang chạy' in current_text or 'Localization running' in current_text:
                self.status_text.config(text=self.translator.get('label.localization_running', 'System: Localization running...'))
            elif 'Ready to start' in current_text or 'Sẵn sàng' in current_text:
                self.status_text.config(text=self.translator.get('label.system_ready', 'System: Ready to start'))
        
        # Update preview frame
        if hasattr(self, 'btn_rviz'):
            self.btn_rviz.config(text="📊 " + self.translator.get("button.launch_rviz2", "Launch RViz2"))
        
        # Update log area
        if hasattr(self, 'log_title_label'):
            self.log_title_label.config(text=self.translator.get('label.system_monitoring_logs', 'SYSTEM MONITORING LOGS'))
        if hasattr(self, 'btn_clear_log'):
            self.btn_clear_log.config(text=self.translator.get('button.clear_log', 'Clear Log'))
        
        # Update vehicle info labels
        if hasattr(self, 'vehicle_info_label_widgets'):
            for key, label_widget in self.vehicle_info_label_widgets.items():
                if key == 'device_id':
                    label_widget.config(text=self.translator.get('label.device_id', 'Device Id') + ":")
                elif key == 'vehicle_id':
                    label_widget.config(text=self.translator.get('label.vehicle_id', 'Vehicle Id') + ":")
                elif key == 'name':
                    label_widget.config(text=self.translator.get('label.name', 'Name') + ":")
                elif key == 'type':
                    label_widget.config(text=self.translator.get('label.type', 'Type') + ":")
                elif key == 'status':
                    label_widget.config(text=self.translator.get('label.status', 'Status') + ":")
        
        # Refresh vehicle info to update translated values (not given, N/A)
        if hasattr(self, 'vehicle_info_labels'):
            self.refresh_vehicle_info()
        
        # Update map info (will be updated by update_map_info)
        if hasattr(self, 'map_info_label'):
            self.update_map_info()
        
        # Update pose display labels
        if hasattr(self, 'pos_label'):
            self.pos_label.config(text=self.translator.get('label.position', 'Position:'))
        if hasattr(self, 'orient_label'):
            self.orient_label.config(text=self.translator.get('label.orientation', 'Orientation:'))
        if hasattr(self, 'pose_status_label'):
            current_text = self.pose_status_label.cget('text')
            if 'Chưa có dữ liệu' in current_text or 'No data' in current_text:
                self.pose_status_label.config(text=self.translator.get('label.no_data', 'No data'))
        
        # Update frame titles
        if hasattr(self, 'movement_control_frame'):
            self.movement_control_frame.config(text=self.translator.get('label.movement_control', 'Movement Control'))
        if hasattr(self, 'preview_frame'):
            self.preview_frame.config(text=self.translator.get('label.preview', 'Preview'))
        if hasattr(self, 'vehicle_info_frame'):
            self.vehicle_info_frame.config(text=self.translator.get('label.vehicle_information', 'Vehicle Information'))
        if hasattr(self, 'map_info_frame'):
            self.map_info_frame.config(text=self.translator.get('label.map_information', 'Map Information'))
        if hasattr(self, 'device_connection_frame'):
            self.device_connection_frame.config(text=self.translator.get('label.device_connection', 'Device Connection'))
            # Cập nhật lại IP và trạng thái
            if hasattr(self, 'device_ip_label'):
                self.device_ip_label.config(text=f"IP: {self.livox_ip}")
            self.check_device_connection()
        if hasattr(self, 'pose_display_frame'):
            self.pose_display_frame.config(text=self.translator.get('label.pose_orientation', 'Pose & Orientation'))

    def setup_movement_control(self):
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.movement_control', 'Movement Control'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.movement_control_frame = frame  # Store reference for update_ui_texts

        self.btn_start = tk.Button(frame, text=self.translator.get('button.start_moving', '▶ START MOVING'), command=self.toggle_movement, state=tk.DISABLED)
        self.btn_start.pack(fill=tk.X, pady=2)

        self.btn_stop = tk.Button(frame, text=self.translator.get('button.stop', 'STOP'), command=self.stop_system, state=tk.DISABLED)
        self.btn_stop.pack(fill=tk.X, pady=2)

        self.status_text = tk.Label(frame, text=self.translator.get('label.loading_map', 'System: Loading map...'), font=("Arial", 9, "italic"))
        self.status_text.pack(pady=5)

    def setup_options(self):
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.preview', 'Preview'), padx=5, pady=5)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.preview_frame = frame  # Store reference for update_ui_texts

        # self.rviz_var = tk.BooleanVar(value=False)
        # self.chk_rviz = tk.Checkbutton(frame, text="Show RViz2 Interface", 
        #                                variable=self.rviz_var, 
        #                                command=self.on_rviz_toggle)
        # self.chk_rviz.pack(anchor="w")

        self.btn_rviz = tk.Button(
            frame, 
            text="📊 " + self.translator.get("button.launch_rviz2", "Launch RViz2"), 
            command=self.open_rviz,
            width=10
        )
        self.btn_rviz.pack(side="right", pady=10)  
    
    def open_rviz(self):
        """Mở RViz2 để xem bản đồ (không khởi động node)"""
        # Kiểm tra map có tồn tại không
        if not self.default_map_root.exists() or not (self.default_map_root / "pose.json").exists():
            self.log(self.translator.get('log.map_not_ready_wait', '⚠️ Map not ready. Please wait for map download to complete.'))
            return
        
        # Tìm rviz config file từ fast_lio_localization package
        rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz" / "fastlio_localization.rviz"
        
        # Nếu không tìm thấy, thử tìm trong rviz_cfg
        if not rviz_config_file.exists():
            rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz_cfg" / "localization.rviz"
        
        self.log(self.translator.get('log.opening_rviz2', '🚀 Opening RViz2...'))
        
        # Kiểm tra xem file có tồn tại không để tránh lỗi im lặng
        if not rviz_config_file.exists():
            self.log(self.translator.get('log.rviz_config_not_found', '⚠️ Warning: Config file not found at {path}. Will open RViz with default.').replace('{path}', str(rviz_config_file)))
            cmd = ['rviz2']
        else:
            cmd = ['rviz2', '-d', str(rviz_config_file)]
            self.log(self.translator.get('log.using_config', '📋 Using config: {name}').replace('{name}', rviz_config_file.name))

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Ẩn log của RViz để đỡ rối terminal
                stderr=subprocess.STDOUT
            )
            self.log(self.translator.get('log.rviz2_opened', '✅ RViz2 opened. Note: Need to start node (START MOVING) to display map.'))
            
        except Exception as e:
            self.log(self.translator.get('log.error_executing_rviz2', '❌ Error executing rviz2: {error}').replace('{error}', str(e)))
    
    def monitor_localization_process(self):
        """Theo dõi output của process localization"""
        proc = self.loc_process
        if not proc:
            return
            
        try:
            # Chỉ đợi process kết thúc, không log output
            proc.wait()
        except Exception as e:
            self.log(self.translator.get('log.error_monitor_localization', 'Error in monitor_localization_process: {error}').replace('{error}', str(e)))
        finally:
            self.root.after(0, self.on_localization_ended)
    
    def on_localization_ended(self):
        """Xử lý khi process localization kết thúc"""
        if self.is_localization_running:
            self.log(self.translator.get('log.localization_process_stopped', '⏹ Localization process stopped'))
            self.root.after(0, self._handle_localization_stopped)
    
    def _handle_localization_stopped(self):
        """Xử lý UI khi localization dừng"""
        self.is_localization_running = False
        self.is_running = False
        self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'), state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
    
    def stop_localization(self):
        """Dừng localization process và kill tất cả các node liên quan"""
        # Kill process group trước
        if self.loc_process:
            try:
                if hasattr(os, 'setsid'):
                    os.killpg(os.getpgid(self.loc_process.pid), signal.SIGTERM)
                else:
                    self.loc_process.terminate()
                self.loc_process.wait(timeout=5)
            except:
                if self.loc_process:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(self.loc_process.pid), signal.SIGKILL)
                    else:
                        self.loc_process.kill()
            finally:
                self.loc_process = None
        
        # Kill tất cả các ROS2 node liên quan đến localization
        self.log(self.translator.get('log.stopping_all_localization_nodes', '🛑 Stopping all localization nodes...'))
        try:
            # Kill node fastlio_mapping
            subprocess.run(
                ['pkill', '-f', 'fastlio_mapping'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill node rviz2 nếu đang chạy với config localization
            subprocess.run(
                ['pkill', '-f', 'fastlio_localization.rviz'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill ros2 launch process
            subprocess.run(
                ['pkill', '-f', 'localization.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
            
            # Kill bằng ros2 node kill (nếu có ROS2 environment)
            try:
                # Source ROS2 và kill node
                ros2_setup = "/opt/ros/jazzy/setup.bash"
                ws_setup = str(self.workspace_path / "install" / "setup.sh")
                
                if os.path.exists(ros2_setup) and os.path.exists(ws_setup):
                    kill_cmd = f"source {ros2_setup} && source {ws_setup} && ros2 node list | grep -E 'fastlio|localization' | xargs -r ros2 node kill"
                    subprocess.run(
                        kill_cmd,
                        shell=True,
                        executable="/bin/bash",
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        timeout=5
                    )
            except:
                pass  # Ignore nếu không có ROS2 environment
            
            self.log(self.translator.get('log.all_localization_nodes_stopped', '✅ All localization nodes stopped'))
            
        except Exception as e:
            self.log(self.translator.get('log.error_killing_nodes', '⚠️ Error killing nodes: {error}').replace('{error}', str(e)))
        
        self.is_localization_running = False
    
    def start_pose_publishing(self):
        """Khởi động ROS2 subscriber để publish pose lên backend"""
        if not ROS2_AVAILABLE:
            self.log(self.translator.get('log.ros2_not_available_pose', '⚠️ ROS2 not available, cannot publish pose'))
            return
        
        if self.is_pose_publishing:
            return
        
        if not self.vehicle_id:
            self.log(self.translator.get('log.vehicle_id_not_available', '⚠️ Vehicle ID not available, cannot publish pose'))
            return
        
        try:
            # Kiểm tra topics có tồn tại không trước khi subscribe
            self.log(self.translator.get('log.checking_pose_topics', '🔍 Checking pose topics...'))
            time.sleep(2)  # Đợi một chút để localization node khởi động
            
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            topics_found = []
            if result.returncode == 0:
                topics = result.stdout
                if '/localization' in topics or 'localization' in topics:
                    topics_found.append('/localization')
                    self.log(self.translator.get('log.topic_exists', '✓ Topic {topic} exists').replace('{topic}', '/localization'))
                if '/Odometry' in topics or 'Odometry' in topics:
                    topics_found.append('/Odometry')
                    self.log(self.translator.get('log.topic_exists', '✓ Topic {topic} exists').replace('{topic}', '/Odometry'))
                
                if not topics_found:
                    self.log(self.translator.get('log.topic_not_found_warning', '⚠️ Warning: Topics /localization or /Odometry not found'))
                    self.log(self.translator.get('log.localization_may_not_ready', '   Localization may not be ready, will try to subscribe and wait...'))
            
            # Khởi tạo ROS2 nếu chưa có
            if not rclpy.ok():
                rclpy.init()
            
            # Tạo subscriber node
            self.pose_subscriber = PoseSubscriber(
                callback_func=self.pose_callback_handler,
                node_name='pose_publisher_node'
            )
            
            # Tạo executor
            self.pose_executor = SingleThreadedExecutor()
            self.pose_executor.add_node(self.pose_subscriber)
            
            # Chạy executor trong background thread
            self.is_pose_publishing = True
            self.pose_thread = threading.Thread(
                target=self._run_pose_executor,
                daemon=True
            )
            self.pose_thread.start()
            
            self.log(self.translator.get('log.pose_publisher_started', '✅ Pose publisher started'))
            self.log(self.translator.get('log.waiting_for_pose', '📡 Waiting for pose from /localization or /Odometry...'))
            
            # Bắt đầu timer để kiểm tra xem có nhận được pose không
            self.last_pose_received_time = None
            self.pose_first_received_logged = False
            self._start_pose_monitoring()
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_pose_publisher', '❌ Error starting pose publisher: {error}').replace('{error}', str(e)))
            self.is_pose_publishing = False
    
    def _run_pose_executor(self):
        """Chạy ROS2 executor trong background thread"""
        try:
            while self.is_pose_publishing and rclpy.ok():
                self.pose_executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.log(self.translator.get('log.error_pose_executor', '❌ Error in pose executor: {error}').replace('{error}', str(e)))
        finally:
            if self.pose_subscriber:
                self.pose_subscriber.destroy_node()
            if not rclpy.ok():
                rclpy.shutdown()
    
    def stop_pose_publishing(self):
        """Dừng pose publishing"""
        if not self.is_pose_publishing:
            return
        
        self.is_pose_publishing = False
        
        if self.pose_executor:
            try:
                if self.pose_subscriber:
                    self.pose_executor.remove_node(self.pose_subscriber)
                    self.pose_subscriber.destroy_node()
            except:
                pass
        
        if self.pose_thread and self.pose_thread.is_alive():
            # Đợi thread kết thúc
            self.pose_thread.join(timeout=2)
        
        self.log(self.translator.get('log.pose_publisher_stopped', '🛑 Pose publisher stopped'))
    
    def pose_callback_handler(self, position, orientation, timestamp):
        """Xử lý pose callback và gửi lên backend"""
        if not self.vehicle_id:
            return
        
        # Cập nhật thời gian nhận pose cuối cùng
        self.last_pose_received_time = time.time()
        
        # Log lần đầu nhận được pose (chỉ 1 lần)
        if not self.pose_first_received_logged:
            self.log(self.translator.get('log.pose_received', '✅ Pose received'))
            self.pose_first_received_logged = True
        
        # Cập nhật UI (chạy trong main thread) - luôn cập nhật để hiển thị real-time
        self.root.after(0, lambda: self.update_pose_display(position, orientation, timestamp))
        
        # Đếm số lần nhận pose
        self.pose_counter += 1
        
        # Chỉ gửi lên backend mỗi 20 lần
        if self.pose_counter >= self.pose_send_interval:
            self.pose_counter = 0  # Reset counter
            # Gửi lên backend trong background thread để không block
            threading.Thread(
                target=self.send_pose_to_backend,
                args=(position, orientation, timestamp),
                daemon=True
            ).start()
    
    def _start_pose_monitoring(self):
        """Bắt đầu monitoring để kiểm tra xem có nhận được pose không"""
        def check_pose():
            if not self.is_pose_publishing:
                return
            
            if self.last_pose_received_time is None:
                # Chưa nhận được pose, kiểm tra lại sau 5 giây
                self.root.after(5000, check_pose)
                return
            
            # Đã nhận được pose, không cần check nữa
            elapsed = time.time() - self.last_pose_received_time
            if elapsed > 10:
                # Không nhận được pose trong 10 giây
                self.log(self.translator.get('log.pose_not_received', '❌ No pose received in 10 seconds'))
                # Check lại sau 10 giây nữa
                self.root.after(10000, check_pose)
            else:
                # Vẫn nhận được pose, check lại sau 10 giây
                self.root.after(10000, check_pose)
        
        # Bắt đầu check sau 5 giây
        self.root.after(5000, check_pose)
    
    def send_pose_to_backend(self, position, orientation, timestamp):
        """Gửi pose lên backend API"""
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/pose"
            
            payload = {
                "position": position,
                "orientation": orientation,
                "timestamp": timestamp
            }
            
            # API sử dụng POST method, không phải PUT
            response = requests.post(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            if response.status_code == 200:
                # Không log mỗi lần thành công để tránh spam
                pass
            elif response.status_code == 405:
                # Method not allowed - log chi tiết để debug
                if not hasattr(self, '_last_pose_error_code') or self._last_pose_error_code != response.status_code:
                    try:
                        error_detail = response.text[:200]
                    except:
                        error_detail = "Không thể đọc error detail"
                    self.log(self.translator.get('log.error_http_405', '⚠️ HTTP 405 error (Method Not Allowed)'))
                    self.log(f"   URL: {url}")
                    self.log(f"   Method: POST")
                    self.log(f"   Response: {error_detail}")
                    self._last_pose_error_code = response.status_code
            else:
                # Chỉ log lỗi một lần để tránh spam
                if not hasattr(self, '_last_pose_error_code') or self._last_pose_error_code != response.status_code:
                    try:
                        error_detail = response.text[:200]
                    except:
                        error_detail = ""
                    self.log(self.translator.get('log.error_sending_pose_http', '⚠️ Error sending pose: HTTP {code} - {detail}').replace('{code}', str(response.status_code)).replace('{detail}', error_detail))
                    self._last_pose_error_code = response.status_code
                
        except requests.exceptions.RequestException as e:
            # Chỉ log lỗi, không spam log
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(self.translator.get('log.error_connection_sending_pose', '⚠️ Connection error when sending pose: {error}').replace('{error}', str(e)))
                self._last_pose_error = datetime.now()
        except Exception as e:
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(self.translator.get('log.error_sending_pose', '⚠️ Error sending pose: {error}').replace('{error}', str(e)))
                self._last_pose_error = datetime.now()
    
    def check_livox_driver_running(self):
        """Kiểm tra xem Livox driver có đang chạy không bằng cách kiểm tra topics"""
        try:
            # Kiểm tra topics /livox/lidar và livox/imu có tồn tại không
            # Lưu ý: Driver publish livox/imu (không có / ở đầu), nhưng ROS2 có thể normalize
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                topics = result.stdout
                # Kiểm tra cả /livox/lidar và livox/lidar (ROS2 có thể normalize)
                has_lidar = '/livox/lidar' in topics or 'livox/lidar' in topics
                # Kiểm tra cả /livox/imu và livox/imu (driver publish livox/imu)
                has_imu = '/livox/imu' in topics or 'livox/imu' in topics
                
                if has_lidar and has_imu:
                    lidar_topic = '/livox/lidar' if '/livox/lidar' in topics else 'livox/lidar'
                    imu_topic = '/livox/imu' if '/livox/imu' in topics else 'livox/imu'
                    self.log(self.translator.get('log.topics_found', '✓ Found topics: lidar={lidar}, imu={imu}').replace('{lidar}', lidar_topic).replace('{imu}', imu_topic))
                    return True
                else:
                    # Log chi tiết để debug (chỉ khi không tìm thấy)
                    if not has_lidar:
                        self.log(self.translator.get('log.topic_not_found', '⚠️ Topic {topic} not found').replace('{topic}', '/livox/lidar'))
                    if not has_imu:
                        self.log(self.translator.get('log.topic_not_found', '⚠️ Topic {topic} not found').replace('{topic}', '/livox/imu'))
                    # Log tất cả topics có chứa 'livox' để debug
                    livox_topics = [t.strip() for t in topics.split('\n') if 'livox' in t.lower()]
                    if livox_topics:
                        self.log(self.translator.get('log.livox_topics_found', '   Topics containing \'livox\': {topics}').replace('{topics}', ', '.join(livox_topics)))
            
            return False
        except Exception as e:
            self.log(self.translator.get('log.error_checking_driver', '⚠️ Error checking driver: {error}').replace('{error}', str(e)))
            return False
    
    def start_livox_driver(self):
        """Khởi động Livox driver"""
        if self.is_livox_driver_running:
            return True
        
        try:
            # Paths
            drive_ws_path = Path(project_root) / "dependencies" / "drive_ws"
            drive_ws_setup = drive_ws_path / "install" / "setup.sh"
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Kiểm tra setup scripts
            if not os.path.exists(ros2_setup):
                self.log(self.translator.get('log.ros2_setup_not_found', '❌ ROS2 setup not found at: {path}').replace('{path}', ros2_setup))
                return False
            
            if not drive_ws_setup.exists():
                self.log(self.translator.get('log.drive_ws_not_built', '❌ Drive workspace not built: {path}').replace('{path}', str(drive_ws_setup)))
                self.log(self.translator.get('log.please_build_drive_ws', 'Please build drive_ws before running driver'))
                return False
            
            # Launch file path
            launch_file = "msg_MID360_launch.py"
            
            # Build command
            cmd = f"source {ros2_setup} && source {drive_ws_setup} && ros2 launch livox_ros_driver2 {launch_file}"
            
            self.log(self.translator.get('log.starting_livox_driver', '🚀 Đang khởi động Livox driver...'))
            self.log(f"Command: {cmd}")
            
            # Start process
            self.livox_driver_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_livox_driver_running = True
            threading.Thread(target=self.monitor_livox_driver_output, daemon=True).start()
            
            # Đợi một chút để driver khởi động (giống như recorder)
            time.sleep(2.0)
            
            # Kiểm tra process có còn chạy không (quan trọng hơn là check topics)
            if self.livox_driver_process and self.livox_driver_process.poll() is None:
                # Process đang chạy, đợi thêm một chút để topics xuất hiện
                time.sleep(2.0)
                
                # Kiểm tra lại topics (nhưng không bắt buộc phải có)
                if self.check_livox_driver_running():
                    self.log(self.translator.get('log.livox_driver_started', '✅ Livox driver đã khởi động thành công'))
                    return True
                else:
                    # Process đang chạy nhưng topics chưa xuất hiện - có thể thiết bị chưa kết nối
                    # Nhưng vẫn return True để tiếp tục với localization (có thể đang replay bag)
                    self.log(self.translator.get('log.livox_driver_running_no_topics', '⚠️ Driver process đang chạy nhưng topics chưa xuất hiện. Tiếp tục với localization...'))
                    return True
            else:
                # Process đã dừng ngay sau khi start - có thể không có thiết bị
                self.log(self.translator.get('log.livox_driver_exited_immediately', '⚠️ Driver process đã dừng ngay sau khi start. Có thể không có thiết bị hoặc đang replay bag.'))
                self.is_livox_driver_running = False
                self.livox_driver_process = None
                return False
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_livox_driver', '❌ Error starting Livox driver: {error}').replace('{error}', str(e)))
            self.is_livox_driver_running = False
            self.livox_driver_process = None
            return False
    
    def monitor_livox_driver_output(self):
        """Monitor output của Livox driver process"""
        proc = self.livox_driver_process
        if not proc:
            return
        
        try:
            for line in iter(proc.stdout.readline, ''):
                if not line:
                    break
                line = line.strip()
                if line:
                    # Chỉ log lỗi và cảnh báo để tránh spam
                    if any(keyword in line.lower() for keyword in ['error', 'fatal', 'exception', 'failed', 'cannot', 'unable']):
                        self.log(f"❌ Driver ERROR: {line}")
                    elif any(keyword in line.lower() for keyword in ['warning', 'warn']):
                        self.log(f"⚠️ Driver WARNING: {line}")
        except Exception as e:
            self.log(self.translator.get('log.error_reading_driver_output', 'Error reading driver output: {error}').replace('{error}', str(e)))
        finally:
            # Khi process kết thúc
            if proc.poll() is not None:
                self.root.after(0, self._handle_driver_stopped)
    
    def _handle_driver_stopped(self):
        """Xử lý khi driver dừng"""
        if self.is_livox_driver_running:
            self.log(self.translator.get('log.livox_driver_stopped', '⏹ Livox driver đã dừng'))
            self.is_livox_driver_running = False
            self.livox_driver_process = None
    
    def stop_livox_driver(self):
        """Dừng Livox driver"""
        if not self.is_livox_driver_running:
            return
        
        if self.livox_driver_process:
            try:
                self.log(self.translator.get('log.stopping_livox_driver', '🛑 Đang dừng Livox driver...'))
                if hasattr(os, 'setsid'):
                    os.killpg(os.getpgid(self.livox_driver_process.pid), signal.SIGTERM)
                else:
                    self.livox_driver_process.terminate()
                self.livox_driver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                if self.livox_driver_process:
                    if hasattr(os, 'setsid'):
                        os.killpg(os.getpgid(self.livox_driver_process.pid), signal.SIGKILL)
                    else:
                        self.livox_driver_process.kill()
            except Exception as e:
                self.log(self.translator.get('log.error_stopping_driver', '⚠️ Error stopping driver: {error}').replace('{error}', str(e)))
            finally:
                self.livox_driver_process = None
        
        # Kill tất cả các node liên quan đến livox driver
        try:
            subprocess.run(
                ['pkill', '-f', 'livox_ros_driver2'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=3
            )
        except:
            pass
        
        self.is_livox_driver_running = False
        self.log(self.translator.get('log.livox_driver_stopped', '⏹ Livox driver đã dừng'))
    
    def setup_vehicle_info(self):
        device_id = self.get_mac_address()
        self.vehicle_id = device_id.lower().replace(':', '')[:12]
        vehicle_id = self.vehicle_id
        print("Vehicle ID:", vehicle_id)

        self.vehicle_info_frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.vehicle_information', 'Vehicle Information'), padx=10, pady=10)
        self.vehicle_info_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Lưu reference đến các label để có thể cập nhật lại
        self.vehicle_info_labels = {}
        
        # Tạo các label ban đầu
        info_keys = ["device_id", "vehicle_id", "name", "type", "status"]
        self.vehicle_info_label_widgets = {}  # Store label widgets for translation
        for key in info_keys:
            row = tk.Frame(self.vehicle_info_frame)
            row.pack(fill=tk.X, pady=2)
            # Use translator for label text
            label_key = f'label.vehicle_{key}' if key != 'name' and key != 'type' and key != 'status' else f'label.vehicle_{key}'
            if key == 'device_id':
                label_text = self.translator.get('label.device_id', 'Device Id') + ":"
            elif key == 'vehicle_id':
                label_text = self.translator.get('label.vehicle_id', 'Vehicle Id') + ":"
            elif key == 'name':
                label_text = self.translator.get('label.name', 'Name') + ":"
            elif key == 'type':
                label_text = self.translator.get('label.type', 'Type') + ":"
            elif key == 'status':
                label_text = self.translator.get('label.status', 'Status') + ":"
            else:
                label_text = key.replace("_", " ").title() + ":"
            label_widget = tk.Label(row, text=label_text, fg="grey", font=("Arial", 8))
            label_widget.pack(side=tk.LEFT)
            self.vehicle_info_label_widgets[key] = label_widget
            value_label = tk.Label(row, text="--", font=("Arial", 9, "bold"))
            value_label.pack(side=tk.RIGHT)
            self.vehicle_info_labels[key] = value_label
        
        # Load thông tin ban đầu
        self.refresh_vehicle_info()
    
    def refresh_vehicle_info(self):
        """Refresh thông tin vehicle từ API và cập nhật UI"""
        if not self.vehicle_id:
            return
        
        try:
            url = f'{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}'
            headers = {
                'accept': 'application/json'
            }
            
            response = requests.get(url, headers=headers, timeout=5)
            if response.status_code == 200:
                data = response.json()
            else:
                # Nếu không tìm thấy, sử dụng giá trị mặc định
                data = {}
        except Exception as e:
            self.log(self.translator.get('log.error_getting_vehicle_info', '⚠️ Error getting vehicle info: {error}').replace('{error}', str(e)))
            data = {}

        # Xử lý name: nếu null hoặc None thì hiển thị "not given"
        name = data.get("name")
        if name is None or name == "":
            name = self.translator.get('label.not_given', 'not given')
        
        # Xử lý type: nếu null hoặc None thì hiển thị "not given"
        vehicle_type = data.get("vehicle_type") or data.get("type")
        if vehicle_type is None or vehicle_type == "":
            vehicle_type = self.translator.get('label.not_given', 'not given')
        
        # Xử lý status: lấy từ API, nếu không có thì dùng "N/A"
        status = data.get("status") or self.translator.get('label.not_available', 'N/A')
        
        # Lấy device_id
        device_id = self.get_mac_address()

        # Cập nhật các label
        if "device_id" in self.vehicle_info_labels:
            self.vehicle_info_labels["device_id"].config(text=device_id)
        if "vehicle_id" in self.vehicle_info_labels:
            self.vehicle_info_labels["vehicle_id"].config(text=self.vehicle_id)
        if "name" in self.vehicle_info_labels:
            self.vehicle_info_labels["name"].config(text=name)
        if "type" in self.vehicle_info_labels:
            self.vehicle_info_labels["type"].config(text=vehicle_type)
        if "status" in self.vehicle_info_labels:
            self.vehicle_info_labels["status"].config(text=status)
    
    def setup_map_info(self):
        """Thiết lập phần hiển thị thông tin bản đồ"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.map_information', 'Map Information'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.map_info_frame = frame  # Store reference for update_ui_texts
        
        self.map_info_label = tk.Label(
            frame,
            text=self.translator.get('label.checking_map', 'Checking map...'),
            foreground="gray",
            font=("Arial", 8),
            wraplength=280,
            justify=tk.LEFT
        )
        self.map_info_label.pack(anchor=tk.W, pady=2)
        
        # Hiển thị thông tin chi tiết
        self.map_detail_label = tk.Label(
            frame,
            text="",
            foreground="gray",
            font=("Arial", 7),
            wraplength=280,
            justify=tk.LEFT
        )
        self.map_detail_label.pack(anchor=tk.W, pady=2)
        
        # Cập nhật thông tin map ban đầu
        self.update_map_info()
    
    def update_map_info(self):
        """Kiểm tra và hiển thị thông tin bản đồ từ fastloc_map"""
        map_path = self.default_map_root
        
        if not map_path.exists():
            self.map_info_label.config(text=self.translator.get('label.map_directory_not_exists', '❌ Map directory does not exist'), foreground="red")
            self.map_detail_label.config(text="")
            return

        pose_json = map_path / "pose.json"
        pcd_dir = map_path / "pcd"
        index_json = map_path / "index.json"
        
        issues = []
        if not pose_json.exists():
            issues.append(self.translator.get('label.missing_pose_json', 'missing pose.json'))
        if not pcd_dir.exists() or not any(pcd_dir.glob("*.pcd")):
            issues.append(self.translator.get('label.missing_pcd_files', 'missing pcd files'))
            
        if issues:
            self.map_info_label.config(text=self.translator.get('label.map_invalid', '⚠️ Map invalid: {issues}').replace('{issues}', ', '.join(issues)), foreground="orange")
            self.map_detail_label.config(text="")
        else:
            # Đếm số tile từ PCD files
            num_tiles = len(list(pcd_dir.glob("*.pcd")))
            
            # Đọc thông tin từ index.json nếu có
            detail_text = ""
            if index_json.exists():
                try:
                    with open(index_json, 'r', encoding='utf-8') as f:
                        index_data = json.load(f)
                        num_tiles_from_index = index_data.get("num_tiles", num_tiles)
                        if num_tiles_from_index != num_tiles:
                            num_tiles = num_tiles_from_index
                        
                        # Hiển thị thông tin chi tiết
                        tiles = index_data.get("tiles", [])
                        if tiles:
                            total_points = sum(tile.get("num_points", 0) for tile in tiles)
                            detail_text = self.translator.get('label.total_points_tiles', 'Total points: {points:,} | Tiles: {tiles}').replace('{points:,}', f'{total_points:,}').replace('{tiles}', str(num_tiles))
                except Exception as e:
                    detail_text = self.translator.get('label.tiles_count', 'Tiles: {count}').replace('{count}', str(num_tiles))
            else:
                detail_text = self.translator.get('label.tiles_count', 'Tiles: {count}').replace('{count}', str(num_tiles))
            
            self.map_info_label.config(text=self.translator.get('label.map_valid', '✅ Map valid: {count} tiles').replace('{count}', str(num_tiles)), foreground="green")
            self.map_detail_label.config(text=detail_text)
    
    def _get_livox_ip_from_config(self):
        """Đọc IP của Livox MID360 từ config file"""
        try:
            config_path = Path(project_root) / "dependencies" / "drive_ws" / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    # Thử lấy từ lidar_configs trước
                    if 'lidar_configs' in config and len(config['lidar_configs']) > 0:
                        ip = config['lidar_configs'][0].get('ip')
                        if ip:
                            return ip
                    # Nếu không có, thử lấy từ host_net_info
                    if 'MID360' in config and 'host_net_info' in config['MID360']:
                        if isinstance(config['MID360']['host_net_info'], list) and len(config['MID360']['host_net_info']) > 0:
                            lidar_ip = config['MID360']['host_net_info'][0].get('lidar_ip')
                            if lidar_ip and isinstance(lidar_ip, list) and len(lidar_ip) > 0:
                                return lidar_ip[0]
        except Exception as e:
            pass  # Ignore errors, dùng IP mặc định
        
        # IP mặc định
        return "192.168.1.109"
    
    def setup_device_connection_info(self):
        """Thiết lập phần hiển thị thông tin kết nối thiết bị Livox"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.device_connection', 'Device Connection'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.device_connection_frame = frame  # Store reference for update_ui_texts
        
        # Label hiển thị IP
        self.device_ip_label = tk.Label(
            frame,
            text=f"IP: {self.livox_ip}",
            foreground="gray",
            font=("Arial", 8),
            wraplength=280,
            justify=tk.LEFT
        )
        self.device_ip_label.pack(anchor=tk.W, pady=2)
        
        # Label hiển thị trạng thái kết nối
        self.device_status_label = tk.Label(
            frame,
            text=self.translator.get('label.checking_connection', 'Checking connection...'),
            foreground="gray",
            font=("Arial", 9, "bold"),
            wraplength=280,
            justify=tk.LEFT
        )
        self.device_status_label.pack(anchor=tk.W, pady=2)
        
        # Cập nhật trạng thái ban đầu
        self.check_device_connection()
    
    def check_device_connection(self):
        """Kiểm tra kết nối thiết bị Livox bằng ping"""
        if not self.livox_ip:
            return
        
        def check():
            try:
                # Ping nhanh (1 gói tin, chờ 1 giây)
                # Dùng 'n' cho Windows, 'c' cho Linux
                param = '-n' if os.name == 'nt' else '-c'
                result = subprocess.run(
                    ['ping', param, '1', '-W', '1', self.livox_ip],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                
                is_connected = (result.returncode == 0)
                
                # Cập nhật UI trong main thread
                self.root.after(0, lambda: self._update_device_connection_ui(is_connected))
                
            except Exception as e:
                # Nếu có lỗi, coi như không kết nối
                self.root.after(0, lambda: self._update_device_connection_ui(False))
        
        # Chạy trong thread riêng để không block UI
        threading.Thread(target=check, daemon=True).start()
    
    def _update_device_connection_ui(self, is_connected):
        """Cập nhật UI hiển thị trạng thái kết nối thiết bị"""
        if not hasattr(self, 'device_status_label'):
            return
        
        self.is_livox_device_connected = is_connected
        
        if is_connected:
            self.device_status_label.config(
                text="● " + self.translator.get('label.device_connected', 'Device: Connected'),
                foreground="green"
            )
        else:
            self.device_status_label.config(
                text="● " + self.translator.get('label.device_disconnected', 'Device: Disconnected'),
                foreground="red"
            )
    
    def start_device_connection_monitoring(self):
        """Bắt đầu monitoring kết nối thiết bị định kỳ"""
        def update_periodically():
            if hasattr(self, 'device_status_label'):
                self.check_device_connection()
            # Lên lịch cập nhật tiếp theo
            self.device_connection_timer_id = self.root.after(
                self.device_connection_update_interval,
                update_periodically
            )
        
        # Bắt đầu cập nhật ngay lập tức và sau đó định kỳ
        update_periodically()
    
    def stop_device_connection_monitoring(self):
        """Dừng monitoring kết nối thiết bị"""
        if self.device_connection_timer_id:
            self.root.after_cancel(self.device_connection_timer_id)
            self.device_connection_timer_id = None
    
    def setup_pose_display(self):
        """Thiết lập phần hiển thị pose và orientation"""
        frame = tk.LabelFrame(self.sidebar, text=self.translator.get('label.pose_orientation', 'Pose & Orientation'), padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        self.pose_display_frame = frame  # Store reference for update_ui_texts
        
        # Position section
        self.pos_label = tk.Label(frame, text=self.translator.get('label.position', 'Position:'), font=("Arial", 8, "bold"), fg="gray")
        self.pos_label.pack(anchor=tk.W, pady=(0, 2))
        
        self.pos_x_label = tk.Label(frame, text="X: --", font=("Arial", 8), fg="black")
        self.pos_x_label.pack(anchor=tk.W, padx=10)
        
        self.pos_y_label = tk.Label(frame, text="Y: --", font=("Arial", 8), fg="black")
        self.pos_y_label.pack(anchor=tk.W, padx=10)
        
        self.pos_z_label = tk.Label(frame, text="Z: --", font=("Arial", 8), fg="black")
        self.pos_z_label.pack(anchor=tk.W, padx=10)
        
        # Separator
        separator = tk.Frame(frame, height=1, bg="gray")
        separator.pack(fill=tk.X, pady=5)
        
        # Orientation section
        self.orient_label = tk.Label(frame, text=self.translator.get('label.orientation', 'Orientation:'), font=("Arial", 8, "bold"), fg="gray")
        self.orient_label.pack(anchor=tk.W, pady=(5, 2))
        
        self.orient_x_label = tk.Label(frame, text="X: --", font=("Arial", 8), fg="black")
        self.orient_x_label.pack(anchor=tk.W, padx=10)
        
        self.orient_y_label = tk.Label(frame, text="Y: --", font=("Arial", 8), fg="black")
        self.orient_y_label.pack(anchor=tk.W, padx=10)
        
        self.orient_z_label = tk.Label(frame, text="Z: --", font=("Arial", 8), fg="black")
        self.orient_z_label.pack(anchor=tk.W, padx=10)
        
        self.orient_w_label = tk.Label(frame, text="W: --", font=("Arial", 8), fg="black")
        self.orient_w_label.pack(anchor=tk.W, padx=10)
        
        # Status
        self.pose_status_label = tk.Label(
            frame,
            text=self.translator.get('label.no_data', 'No data'),
            font=("Arial", 7),
            fg="gray"
        )
        self.pose_status_label.pack(anchor=tk.W, pady=(5, 0))
    
    def update_pose_display(self, position, orientation, timestamp):
        """Cập nhật hiển thị pose và orientation"""
        try:
            # Cập nhật position
            self.pos_x_label.config(text=f"X: {position['x']:.3f}")
            self.pos_y_label.config(text=f"Y: {position['y']:.3f}")
            self.pos_z_label.config(text=f"Z: {position['z']:.3f}")
            
            # Cập nhật orientation
            self.orient_x_label.config(text=f"X: {orientation['x']:.4f}")
            self.orient_y_label.config(text=f"Y: {orientation['y']:.4f}")
            self.orient_z_label.config(text=f"Z: {orientation['z']:.4f}")
            self.orient_w_label.config(text=f"W: {orientation['w']:.4f}")
            
            # Cập nhật timestamp (chỉ hiển thị thời gian, không hiển thị ngày)
            try:
                dt = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
                time_str = dt.strftime("%H:%M:%S.%f")[:-3]  # Hiển thị đến millisecond
                self.pose_status_label.config(
                    text=f"Cập nhật: {time_str}",
                    fg="green"
                )
            except:
                self.pose_status_label.config(
                    text="Đang cập nhật...",
                    fg="green"
                )
        except Exception as e:
            self.pose_status_label.config(
                text=f"Lỗi: {e}",
                fg="red"
            )
    
    def reset_pose_display(self):
        """Reset pose và orientation về giá trị mặc định"""
        try:
            # Reset position
            self.pos_x_label.config(text="X: --")
            self.pos_y_label.config(text="Y: --")
            self.pos_z_label.config(text="Z: --")
            
            # Reset orientation
            self.orient_x_label.config(text="X: --")
            self.orient_y_label.config(text="Y: --")
            self.orient_z_label.config(text="Z: --")
            self.orient_w_label.config(text="W: --")
            
            # Reset status
            self.pose_status_label.config(
                text=self.translator.get('label.no_data', 'No data'),
                fg="gray"
            )
        except Exception as e:
            pass  # Ignore errors khi reset

    def setup_log_area(self):
        # Tiêu đề của Log
        log_header = tk.Frame(self.log_container)
        log_header.pack(fill=tk.X, pady=(0, 5))
        
        self.log_title_label = tk.Label(log_header, text=self.translator.get('label.system_monitoring_logs', 'SYSTEM MONITORING LOGS'), font=("Arial", 9, "bold"))
        self.log_title_label.pack(side=tk.LEFT)
        
        self.btn_clear_log = tk.Button(log_header, text=self.translator.get('button.clear_log', 'Clear Log'), 
                              command=self.clear_log, 
                              font=("Arial", 7))
        self.btn_clear_log.pack(side=tk.RIGHT)

        # Log Panel chiếm toàn bộ không gian còn lại
        # Sử dụng scrolledtext để tự động có thanh cuộn nếu log quá dài
        self.log_panel = scrolledtext.ScrolledText(self.log_container, state='disabled', 
                                                   font=("Consolas", 10), relief=tk.SUNKEN, bd=1)
        self.log_panel.pack(fill=tk.BOTH, expand=True)
        
        self.log(self.translator.get('log.system_ready', '✅ System ready'))

    def get_mac_address(self):
        try:
            mac = uuid.getnode()
            return ':'.join(['{:02x}'.format((mac >> elements) & 0xff) for elements in range(40, -1, -8)])
        except: return "unknown"

    def log(self, message):
        self.log_panel.config(state='normal')
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_panel.insert(tk.END, f"[{time_str}] {message}\n")
        self.log_panel.see(tk.END)
        self.log_panel.config(state='disabled')

    def clear_log(self):
        self.log_panel.config(state='normal')
        self.log_panel.delete('1.0', tk.END)
        self.log_panel.config(state='disabled')

    def on_rviz_toggle(self):
        status = "ENABLED" if self.rviz_var.get() else "DISABLED"
        self.log(f"CONFIG: RViz2 visualization {status}.")

    def toggle_movement(self):
        if not self.is_running:
            # Khởi động localization node
            self.start_localization()
        else:
            self.stop_system()
    
    def start_localization(self):
        """Khởi động localization node"""
        if self.is_localization_running:
            return
        
        # Kiểm tra map có tồn tại không
        if not self.default_map_root.exists() or not (self.default_map_root / "pose.json").exists():
            self.log(self.translator.get('log.map_not_ready_wait', '⚠️ Map not ready. Please wait for map download to complete.'))
            return
        
        # Luôn luôn thử start driver khi người dùng bấm START MOVING (giống như recorder)
        # Không check topics trước vì khi thiết bị chưa start thì không có topics
        self.log("=" * 60)
        self.log(self.translator.get('log.checking_livox_driver', '🔍 Kiểm tra Livox driver...'))
        
        if not self.is_livox_driver_running:
            self.log(self.translator.get('log.livox_driver_not_running', '⚠️ Livox driver chưa chạy, đang khởi động...'))
            # Thử start driver (sẽ tự động detect nếu có thiết bị)
            driver_started = self.start_livox_driver()
            if driver_started:
                self.log(self.translator.get('log.livox_driver_started', '✅ Livox driver đã khởi động thành công'))
            else:
                # Driver không start được, có thể không có thiết bị hoặc đang replay bag
                # Vẫn tiếp tục với localization (có thể đang replay bag)
                self.log(self.translator.get('log.livox_driver_start_failed_continue', '⚠️ Không thể khởi động Livox driver. Tiếp tục với localization (có thể đang replay bag)...'))
        else:
            self.log(self.translator.get('log.livox_driver_already_running', '✅ Livox driver đã đang chạy'))
        
        # Script helper
        run_script = Path(project_root) / "scripts" / "run_localization.sh"
        if not run_script.exists():
            self.log(self.translator.get('log.localization_script_not_found', '❌ Localization script not found at: {path}').replace('{path}', str(run_script)))
            return
        
        self.log("=" * 60)
        self.log(self.translator.get('log.starting_localization_node', '🚀 Starting Localization node...'))
        self.log(self.translator.get('log.map_path', '📁 Map: {path}').replace('{path}', str(self.default_map_root)))
        
        try:
            # Chạy localization script với RViz disabled (chỉ chạy node)
            map_root_str = str(self.default_map_root)
            cmd = f"{run_script} {map_root_str} False mid360.yaml"
            
            # Start process
            self.loc_process = subprocess.Popen(
                cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_localization_running = True
            self.is_running = True
            threading.Thread(target=self.monitor_localization_process, daemon=True).start()
            
            # Bắt đầu publish pose lên backend
            self.start_pose_publishing()
            
            # Cập nhật UI
            self.btn_start.config(text=self.translator.get('button.stop_movement', '■ STOP MOVEMENT'), state=tk.NORMAL)
            self.btn_stop.config(state=tk.NORMAL)
            self.status_text.config(text=self.translator.get('label.localization_running', 'System: Localization running...'))
            self.log(self.translator.get('log.localization_node_started', '✅ Localization node started.'))
            
        except Exception as e:
            self.log(self.translator.get('log.error_starting_localization', '❌ Error starting localization: {error}').replace('{error}', str(e)))
            self.is_localization_running = False
            self.is_running = False

    def stop_system(self):
        """Dừng localization node"""
        self.stop_pose_publishing()
        self.stop_localization()
        # Dừng Livox driver nếu đã khởi động tự động
        if self.is_livox_driver_running:
            self.stop_livox_driver()
        self.is_running = False
        self.btn_start.config(text=self.translator.get('button.start_moving', '▶ START MOVING'), state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_text.config(text=self.translator.get('label.stopped', 'System: Stopped'))
        # Reset pose display
        self.reset_pose_display()
        self.log(self.translator.get('log.stop_localization_requested', 'COMMAND: Stop localization requested.'))
    
    def download_map_and_qr(self):
        """Tự động tải fastloc_map và QR từ backend"""
        self.log(self.translator.get('log.downloading_map_qr', '🌐 Loading map and QR from backend...'))
        threading.Thread(target=self._download_worker, daemon=True).start()
    
    def _download_worker(self):
        """Worker thread để tải map và QR từ backend"""
        try:
            base_url = self.backend_base_url
            
            # Tải fastloc_map
            fastloc_url = f"{base_url}/api/v1/maps/localization/fastloc/download"
            self.log(self.translator.get('log.downloading_fastloc_map', '⬇️ Downloading fastloc_map from backend...'))
            
            dest_root = self.log_path
            dest_root.mkdir(parents=True, exist_ok=True)
            dest_zip = dest_root / "fastloc_map.zip"
            
            try:
                with requests.get(fastloc_url, stream=True, timeout=120) as r:
                    if r.status_code != 200:
                        self.log(self.translator.get('log.fastloc_map_download_failed', '❌ Failed to download fastloc_map (HTTP {code})').replace('{code}', str(r.status_code)))
                        return
                    with open(dest_zip, "wb") as f:
                        for chunk in r.iter_content(chunk_size=8192):
                            if chunk:
                                f.write(chunk)
                
                self.log(self.translator.get('log.extracting_fastloc_map', '📦 Extracting fastloc_map...'))
                # Xóa map cũ để tránh lẫn file
                if self.default_map_root.exists():
                    shutil.rmtree(self.default_map_root, ignore_errors=True)
                
                # Giải nén vào temp folder trước
                temp_extract_dir = dest_root / "temp_extract"
                if temp_extract_dir.exists():
                    shutil.rmtree(temp_extract_dir, ignore_errors=True)
                temp_extract_dir.mkdir(parents=True, exist_ok=True)
                
                # Giải nén zip vào temp folder
                with zipfile.ZipFile(dest_zip, 'r') as zip_ref:
                    zip_ref.extractall(temp_extract_dir)
                
                # Tìm folder fastloc_map trong temp folder hoặc di chuyển nội dung
                extracted_fastloc = None
                for item in temp_extract_dir.iterdir():
                    if item.is_dir() and item.name == "fastloc_map":
                        extracted_fastloc = item
                        break
                
                if extracted_fastloc:
                    # Zip chứa folder fastloc_map, di chuyển vào đúng vị trí
                    shutil.move(str(extracted_fastloc), str(self.default_map_root))
                else:
                    # Zip chứa file trực tiếp, tạo folder fastloc_map và di chuyển nội dung vào
                    self.default_map_root.mkdir(parents=True, exist_ok=True)
                    for item in temp_extract_dir.iterdir():
                        dest_item = self.default_map_root / item.name
                        if item.is_dir():
                            if dest_item.exists():
                                shutil.rmtree(dest_item, ignore_errors=True)
                            shutil.move(str(item), str(dest_item))
                        else:
                            shutil.move(str(item), str(dest_item))
                
                # Xóa temp folder
                if temp_extract_dir.exists():
                    shutil.rmtree(temp_extract_dir, ignore_errors=True)
                
                # Xóa file zip tạm
                try:
                    dest_zip.unlink()
                    self.log(self.translator.get('log.deleted_temp_zip', '🧹 Deleted temporary zip file: {filename}').replace('{filename}', dest_zip.name))
                except Exception as e:
                    self.log(self.translator.get('log.cannot_delete_temp_zip', '⚠️ Cannot delete temporary zip file: {error}').replace('{error}', str(e)))
                
                # Kiểm tra lại folder fastloc_map
                if self.default_map_root.exists():
                    self.log(self.translator.get('log.fastloc_map_downloaded', '✅ fastloc_map downloaded and extracted successfully'))
                    # Cập nhật thông tin map sau khi tải xong
                    self.root.after(0, self._on_map_downloaded)
                else:
                    self.log(self.translator.get('log.fastloc_map_not_exists', '⚠️ fastloc_map folder does not exist after extraction'))
                
            except requests.exceptions.RequestException as e:
                self.log(self.translator.get('log.error_downloading_fastloc_map', '❌ Error downloading fastloc_map: {error}').replace('{error}', str(e)))
                return
            except Exception as e:
                self.log(self.translator.get('log.error_extracting_fastloc_map', '❌ Error extracting fastloc_map: {error}').replace('{error}', str(e)))
                return
            
            # Tải QR_detect.json
            qr_url = f"{base_url}/api/v1/maps/localization/qr"
            self.log(self.translator.get('log.downloading_qr_detect', '⬇️ Downloading QR_detect.json from backend...'))
            
            try:
                resp = requests.get(qr_url, timeout=30)
                if resp.status_code == 200:
                    with open(self.qr_detect_path, 'wb') as f:
                        f.write(resp.content)
                    self.log(self.translator.get('log.qr_detect_downloaded', '✅ QR_detect.json downloaded successfully'))
                else:
                    self.log(self.translator.get('log.qr_detect_download_failed', '⚠️ Failed to download QR_detect.json (HTTP {code})').replace('{code}', str(resp.status_code)))
            except requests.exceptions.RequestException as e:
                self.log(self.translator.get('log.error_downloading_qr_detect', '⚠️ Error downloading QR_detect.json: {error}').replace('{error}', str(e)))
            
            # Map đã sẵn sàng, người dùng có thể bấm nút Preview để mở RViz
            self.log(self.translator.get('log.map_qr_ready', '✅ Map and QR are ready. Click Preview button to open RViz.'))
            
        except Exception as e:
            self.log(self.translator.get('log.unexpected_error_downloading', '❌ Unexpected error downloading map/QR: {error}').replace('{error}', str(e)))
    
    def _on_map_downloaded(self):
        """Xử lý sau khi tải map thành công"""
        self.update_map_info()
        # Enable nút START MOVING
        self.btn_start.config(state=tk.NORMAL)
        self.status_text.config(text=self.translator.get('label.system_ready', 'System: Ready to start'))
    
    def update_vehicle_status(self, status, refresh_info=False):
        """Cập nhật trạng thái vehicle lên backend"""
        if not self.vehicle_id:
            return
        
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/status"
            payload = {
                "status": status
            }
            
            response = requests.patch(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            if response.status_code == 200:
                self.log(self.translator.get('log.vehicle_status_updated', '✅ Vehicle status updated: {status}').replace('{status}', status))
                # Chỉ refresh vehicle info khi status là "online" và refresh_info=True
                if status == "online" and refresh_info:
                    self.root.after(0, self.refresh_vehicle_info)
            else:
                self.log(self.translator.get('log.cannot_update_vehicle_status', '⚠️ Cannot update vehicle status: HTTP {code}').replace('{code}', str(response.status_code)))
                
        except requests.exceptions.RequestException as e:
            self.log(self.translator.get('log.error_connection_updating_status', '⚠️ Connection error when updating status: {error}').replace('{error}', str(e)))
        except Exception as e:
            self.log(self.translator.get('log.error_updating_status', '⚠️ Error updating status: {error}').replace('{error}', str(e)))
    
    def start_heartbeat(self):
        """Bắt đầu heartbeat loop để gửi status online định kỳ"""
        if not self.vehicle_id:
            return
        
        # Dừng heartbeat cũ nếu có
        self.stop_heartbeat()
        
        # Bắt đầu heartbeat loop
        self._heartbeat_loop()
    
    def stop_heartbeat(self):
        """Dừng heartbeat loop"""
        if self.heartbeat_timer_id is not None:
            self.root.after_cancel(self.heartbeat_timer_id)
            self.heartbeat_timer_id = None
    
    def _heartbeat_loop(self):
        """Gửi heartbeat và schedule lần tiếp theo"""
        if not self.vehicle_id:
            return
        
        # Gửi heartbeat trong background thread để không block UI
        threading.Thread(target=self._send_heartbeat_async, daemon=True).start()
        
        # Schedule lần tiếp theo
        self.heartbeat_timer_id = self.root.after(HEARTBEAT_INTERVAL_SECONDS * 1000, self._heartbeat_loop)
    
    def _send_heartbeat_async(self):
        """Gửi heartbeat trong background thread để không block UI"""
        if not self.vehicle_id:
            return
        
        try:
            url = f"{self.backend_base_url}/api/v1/vehicles/{self.vehicle_id}/status"
            payload = {
                "status": "online"
            }
            
            response = requests.patch(
                url,
                json=payload,
                headers=HEADERS,
                timeout=API_TIMEOUT
            )
            
            # Update UI trong main thread (thread-safe)
            if response.status_code == 200:
                self.root.after(0, lambda: self.log(
                    self.translator.get('log.vehicle_status_updated', '✅ Vehicle status updated: {status}').replace('{status}', 'online')
                ))
            else:
                self.root.after(0, lambda: self.log(
                    self.translator.get('log.cannot_update_vehicle_status', '⚠️ Cannot update vehicle status: HTTP {code}').replace('{code}', str(response.status_code))
                ))
                
        except requests.exceptions.RequestException as e:
            # Log lỗi trong main thread (thread-safe)
            self.root.after(0, lambda: self.log(
                self.translator.get('log.error_connection_updating_status', '⚠️ Connection error when updating status: {error}').replace('{error}', str(e))
            ))
        except Exception as e:
            # Log lỗi trong main thread (thread-safe)
            self.root.after(0, lambda: self.log(
                self.translator.get('log.error_heartbeat', '⚠️ Heartbeat error: {error}').replace('{error}', str(e))
            ))
    
    def on_closing(self):
        """Xử lý khi đóng cửa sổ"""
        # Dừng monitoring kết nối thiết bị
        self.stop_device_connection_monitoring()
        
        # Dừng heartbeat
        self.stop_heartbeat()
        
        # Cập nhật trạng thái vehicle thành offline
        if self.vehicle_id:
            self.update_vehicle_status("offline")
        
        # Dừng tất cả các process đang chạy
        if self.is_running:
            self.stop_system()
        
        # Dừng Livox driver nếu đang chạy
        if self.is_livox_driver_running:
            self.stop_livox_driver()
        
        # Đóng cửa sổ
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkerInterface(root)
    root.mainloop()