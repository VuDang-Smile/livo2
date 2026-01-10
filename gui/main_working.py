import tkinter as tk
from tkinter import scrolledtext
from datetime import datetime
import uuid
import subprocess
import os
import sys
import requests
import threading
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

from languages.translate_engine import translator
from contants.API import VEHICLE_ENDPOINT, API_TIMEOUT, HEADERS, BACKEND_HOST

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
        
        # Pose publishing
        self.pose_subscriber = None
        self.pose_executor = None
        self.pose_thread = None
        self.vehicle_id = None
        self.is_pose_publishing = False
        self.pose_counter = 0  # Đếm số lần nhận pose
        self.pose_send_interval = 20  # Gửi lên backend mỗi 20 lần

        # --- Main Container ---
        self.main_container = tk.Frame(root)
        self.main_container.pack(fill=tk.BOTH, expand=True)

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
        self.setup_pose_display()

        # Cài đặt phần Log bên phải
        self.setup_log_area()
        
        # Cập nhật trạng thái vehicle thành online khi khởi động
        # Gọi sau khi setup_vehicle_info để có vehicle_id
        if self.vehicle_id:
            self.update_vehicle_status("online", refresh_info=True)
        
        # Tự động tải map và QR khi khởi động
        self.download_map_and_qr()
        
        # Thêm handler khi đóng cửa sổ
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_movement_control(self):
        frame = tk.LabelFrame(self.sidebar, text="Movement Control", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)

        self.btn_start = tk.Button(frame, text="▶ START MOVING", command=self.toggle_movement, state=tk.DISABLED)
        self.btn_start.pack(fill=tk.X, pady=2)

        self.btn_stop = tk.Button(frame, text="STOP", command=self.stop_system, state=tk.DISABLED)
        self.btn_stop.pack(fill=tk.X, pady=2)

        self.status_text = tk.Label(frame, text="System: Đang tải map...", font=("Arial", 9, "italic"))
        self.status_text.pack(pady=5)

    def setup_options(self):
        frame = tk.LabelFrame(self.sidebar, text="Preview", padx=5, pady=5)
        frame.pack(fill=tk.X, pady=5, padx=5)

        # self.rviz_var = tk.BooleanVar(value=False)
        # self.chk_rviz = tk.Checkbutton(frame, text="Show RViz2 Interface", 
        #                                variable=self.rviz_var, 
        #                                command=self.on_rviz_toggle)
        # self.chk_rviz.pack(anchor="w")

        self.btn_rviz = tk.Button(
            frame, 
            text="📊 " + translator.get("button.launch_rviz2"), 
            command=self.open_rviz,
            width=10
        )
        self.btn_rviz.pack(side="right", pady=10)  
    
    def open_rviz(self):
        """Mở RViz2 để xem bản đồ (không khởi động node)"""
        # Kiểm tra map có tồn tại không
        if not self.default_map_root.exists() or not (self.default_map_root / "pose.json").exists():
            self.log("⚠️ Map chưa sẵn sàng. Vui lòng đợi tải map hoàn tất.")
            return
        
        # Tìm rviz config file từ fast_lio_localization package
        rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz" / "fastlio_localization.rviz"
        
        # Nếu không tìm thấy, thử tìm trong rviz_cfg
        if not rviz_config_file.exists():
            rviz_config_file = self.workspace_path / "src" / "fast_lio_localization" / "rviz_cfg" / "localization.rviz"
        
        self.log("🚀 Đang mở RViz2...")
        
        # Kiểm tra xem file có tồn tại không để tránh lỗi im lặng
        if not rviz_config_file.exists():
            self.log(f"⚠️ Cảnh báo: Không tìm thấy file cấu hình tại {rviz_config_file}. Sẽ mở RViz mặc định.")
            cmd = ['rviz2']
        else:
            cmd = ['rviz2', '-d', str(rviz_config_file)]
            self.log(f"📋 Sử dụng config: {rviz_config_file.name}")

        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # Ẩn log của RViz để đỡ rối terminal
                stderr=subprocess.STDOUT
            )
            self.log("✅ RViz2 đã được mở. Lưu ý: Cần khởi động node (START MOVING) để hiển thị map.")
            
        except Exception as e:
            self.log(f"❌ Lỗi khi thực thi lệnh rviz2: {e}")
    
    def monitor_localization_process(self):
        """Theo dõi output của process localization"""
        proc = self.loc_process
        if not proc:
            return
            
        try:
            # Chỉ đợi process kết thúc, không log output
            proc.wait()
        except Exception as e:
            self.log(f"Lỗi trong monitor_localization_process: {e}")
        finally:
            self.root.after(0, self.on_localization_ended)
    
    def on_localization_ended(self):
        """Xử lý khi process localization kết thúc"""
        if self.is_localization_running:
            self.log("⏹ Localization process đã dừng.")
            self.root.after(0, self._handle_localization_stopped)
    
    def _handle_localization_stopped(self):
        """Xử lý UI khi localization dừng"""
        self.is_localization_running = False
        self.is_running = False
        self.btn_start.config(text="▶ START MOVING", state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_text.config(text="System: Đã dừng")
    
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
        self.log("🛑 Đang dừng tất cả các node localization...")
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
            
            self.log("✅ Đã dừng tất cả các node localization")
            
        except Exception as e:
            self.log(f"⚠️ Lỗi khi kill node: {e}")
        
        self.is_localization_running = False
    
    def start_pose_publishing(self):
        """Khởi động ROS2 subscriber để publish pose lên backend"""
        if not ROS2_AVAILABLE:
            self.log("⚠️ ROS2 không khả dụng, không thể publish pose")
            return
        
        if self.is_pose_publishing:
            return
        
        if not self.vehicle_id:
            self.log("⚠️ Vehicle ID chưa có, không thể publish pose")
            return
        
        try:
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
            
            self.log("✅ Đã khởi động pose publisher")
            
        except Exception as e:
            self.log(f"❌ Lỗi khi khởi động pose publisher: {e}")
            self.is_pose_publishing = False
    
    def _run_pose_executor(self):
        """Chạy ROS2 executor trong background thread"""
        try:
            while self.is_pose_publishing and rclpy.ok():
                self.pose_executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            self.log(f"❌ Lỗi trong pose executor: {e}")
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
        
        self.log("🛑 Đã dừng pose publisher")
    
    def pose_callback_handler(self, position, orientation, timestamp):
        """Xử lý pose callback và gửi lên backend"""
        if not self.vehicle_id:
            return
        
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
                    self.log(f"⚠️ Lỗi HTTP 405 (Method Not Allowed)")
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
                    self.log(f"⚠️ Lỗi khi gửi pose: HTTP {response.status_code} - {error_detail}")
                    self._last_pose_error_code = response.status_code
                
        except requests.exceptions.RequestException as e:
            # Chỉ log lỗi, không spam log
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(f"⚠️ Lỗi kết nối khi gửi pose: {e}")
                self._last_pose_error = datetime.now()
        except Exception as e:
            if not hasattr(self, '_last_pose_error') or (datetime.now() - self._last_pose_error).seconds > 10:
                self.log(f"⚠️ Lỗi khi gửi pose: {e}")
                self._last_pose_error = datetime.now()
    
    def setup_vehicle_info(self):
        device_id = self.get_mac_address()
        self.vehicle_id = device_id.lower().replace(':', '')[:12]
        vehicle_id = self.vehicle_id
        print("Vehicle ID:", vehicle_id)

        self.vehicle_info_frame = tk.LabelFrame(self.sidebar, text="Vehicle Information", padx=10, pady=10)
        self.vehicle_info_frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Lưu reference đến các label để có thể cập nhật lại
        self.vehicle_info_labels = {}
        
        # Tạo các label ban đầu
        info_keys = ["device_id", "vehicle_id", "name", "type", "status"]
        for key in info_keys:
            row = tk.Frame(self.vehicle_info_frame)
            row.pack(fill=tk.X, pady=2)
            label_text = key.replace("_", " ").title() + ":"
            tk.Label(row, text=label_text, fg="grey", font=("Arial", 8)).pack(side=tk.LEFT)
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
            self.log(f"⚠️ Lỗi khi lấy thông tin vehicle: {e}")
            data = {}

        # Xử lý name: nếu null hoặc None thì hiển thị "not given"
        name = data.get("name")
        if name is None or name == "":
            name = "not given"
        
        # Xử lý type: nếu null hoặc None thì hiển thị "not given"
        vehicle_type = data.get("vehicle_type") or data.get("type")
        if vehicle_type is None or vehicle_type == "":
            vehicle_type = "not given"
        
        # Xử lý status: lấy từ API, nếu không có thì dùng "N/A"
        status = data.get("status") or "N/A"
        
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
        frame = tk.LabelFrame(self.sidebar, text="Map Information", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        
        self.map_info_label = tk.Label(
            frame,
            text="Đang kiểm tra bản đồ...",
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
            self.map_info_label.config(text="❌ Thư mục map không tồn tại", foreground="red")
            self.map_detail_label.config(text="")
            return

        pose_json = map_path / "pose.json"
        pcd_dir = map_path / "pcd"
        index_json = map_path / "index.json"
        
        issues = []
        if not pose_json.exists():
            issues.append("thiếu pose.json")
        if not pcd_dir.exists() or not any(pcd_dir.glob("*.pcd")):
            issues.append("thiếu file pcd")
            
        if issues:
            self.map_info_label.config(text=f"⚠️ Bản đồ không hợp lệ: {', '.join(issues)}", foreground="orange")
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
                            detail_text = f"Tổng điểm: {total_points:,} | Tiles: {num_tiles}"
                except Exception as e:
                    detail_text = f"Tiles: {num_tiles}"
            else:
                detail_text = f"Tiles: {num_tiles}"
            
            self.map_info_label.config(text=f"✅ Bản đồ hợp lệ: {num_tiles} tiles", foreground="green")
            self.map_detail_label.config(text=detail_text)
    
    def setup_pose_display(self):
        """Thiết lập phần hiển thị pose và orientation"""
        frame = tk.LabelFrame(self.sidebar, text="Pose & Orientation", padx=10, pady=10)
        frame.pack(fill=tk.X, pady=5, padx=5)
        
        # Position section
        pos_label = tk.Label(frame, text="Position:", font=("Arial", 8, "bold"), fg="gray")
        pos_label.pack(anchor=tk.W, pady=(0, 2))
        
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
        orient_label = tk.Label(frame, text="Orientation:", font=("Arial", 8, "bold"), fg="gray")
        orient_label.pack(anchor=tk.W, pady=(5, 2))
        
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
            text="Chưa có dữ liệu",
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
                text="Chưa có dữ liệu",
                fg="gray"
            )
        except Exception as e:
            pass  # Ignore errors khi reset

    def setup_log_area(self):
        # Tiêu đề của Log
        log_header = tk.Frame(self.log_container)
        log_header.pack(fill=tk.X, pady=(0, 5))
        
        tk.Label(log_header, text="SYSTEM MONITORING LOGS", font=("Arial", 9, "bold")).pack(side=tk.LEFT)
        
        btn_clear = tk.Button(log_header, text="Clear Log", 
                              command=self.clear_log, 
                              font=("Arial", 7))
        btn_clear.pack(side=tk.RIGHT)

        # Log Panel chiếm toàn bộ không gian còn lại
        # Sử dụng scrolledtext để tự động có thanh cuộn nếu log quá dài
        self.log_panel = scrolledtext.ScrolledText(self.log_container, state='disabled', 
                                                   font=("Consolas", 10), relief=tk.SUNKEN, bd=1)
        self.log_panel.pack(fill=tk.BOTH, expand=True)
        
        self.log("INFO: Connection established with Device.")
        self.log("INFO: LiDAR sensors calibrated.")
        self.log("SUCCESS: System ready for movement.")

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
            self.log("⚠️ Map chưa sẵn sàng. Vui lòng đợi tải map hoàn tất.")
            return
        
        # Script helper
        run_script = Path(project_root) / "scripts" / "run_localization.sh"
        if not run_script.exists():
            self.log(f"❌ Không tìm thấy script chạy localization tại: {run_script}")
            return
        
        self.log("=" * 60)
        self.log("🚀 Đang khởi động Localization node...")
        self.log(f"📁 Map: {self.default_map_root}")
        
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
            self.btn_start.config(text="■ STOP MOVEMENT", state=tk.NORMAL)
            self.btn_stop.config(state=tk.NORMAL)
            self.status_text.config(text="System: Localization đang chạy...")
            self.log("✅ Đã khởi động Localization node.")
            
        except Exception as e:
            self.log(f"❌ Lỗi khi khởi động localization: {e}")
            self.is_localization_running = False
            self.is_running = False

    def stop_system(self):
        """Dừng localization node"""
        self.stop_pose_publishing()
        self.stop_localization()
        self.is_running = False
        self.btn_start.config(text="▶ START MOVING", state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status_text.config(text="System: Đã dừng")
        # Reset pose display
        self.reset_pose_display()
        # Cập nhật trạng thái vehicle thành offline
        if self.vehicle_id:
            self.update_vehicle_status("offline")
        self.log("COMMAND: Stop localization requested.")
    
    def download_map_and_qr(self):
        """Tự động tải fastloc_map và QR từ backend"""
        self.log("🌐 Đang tải map và QR từ backend...")
        threading.Thread(target=self._download_worker, daemon=True).start()
    
    def _download_worker(self):
        """Worker thread để tải map và QR từ backend"""
        try:
            base_url = self.backend_base_url
            
            # Tải fastloc_map
            fastloc_url = f"{base_url}/api/v1/maps/localization/fastloc/download"
            self.log("⬇️ Đang tải fastloc_map từ backend...")
            
            dest_root = self.log_path
            dest_root.mkdir(parents=True, exist_ok=True)
            dest_zip = dest_root / "fastloc_map.zip"
            
            try:
                with requests.get(fastloc_url, stream=True, timeout=120) as r:
                    if r.status_code != 200:
                        self.log(f"❌ Tải fastloc_map thất bại (HTTP {r.status_code})")
                        return
                    with open(dest_zip, "wb") as f:
                        for chunk in r.iter_content(chunk_size=8192):
                            if chunk:
                                f.write(chunk)
                
                self.log("📦 Đang giải nén fastloc_map...")
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
                    self.log(f"🧹 Đã xóa file zip tạm: {dest_zip.name}")
                except Exception as e:
                    self.log(f"⚠️ Không thể xóa file zip tạm: {e}")
                
                # Kiểm tra lại folder fastloc_map
                if self.default_map_root.exists():
                    self.log("✅ Đã tải và giải nén fastloc_map thành công")
                    # Cập nhật thông tin map sau khi tải xong
                    self.root.after(0, self._on_map_downloaded)
                else:
                    self.log("⚠️ Folder fastloc_map không tồn tại sau khi giải nén")
                
            except requests.exceptions.RequestException as e:
                self.log(f"❌ Lỗi khi tải fastloc_map: {e}")
                return
            except Exception as e:
                self.log(f"❌ Lỗi giải nén fastloc_map: {e}")
                return
            
            # Tải QR_detect.json
            qr_url = f"{base_url}/api/v1/maps/localization/qr"
            self.log("⬇️ Đang tải QR_detect.json từ backend...")
            
            try:
                resp = requests.get(qr_url, timeout=30)
                if resp.status_code == 200:
                    with open(self.qr_detect_path, 'wb') as f:
                        f.write(resp.content)
                    self.log("✅ Đã tải QR_detect.json thành công")
                else:
                    self.log(f"⚠️ Không tải được QR_detect.json (HTTP {resp.status_code})")
            except requests.exceptions.RequestException as e:
                self.log(f"⚠️ Lỗi khi tải QR_detect.json: {e}")
            
            # Map đã sẵn sàng, người dùng có thể bấm nút Preview để mở RViz
            self.log("✅ Map và QR đã sẵn sàng. Bấm nút Preview để mở RViz.")
            
        except Exception as e:
            self.log(f"❌ Lỗi không mong đợi khi tải map/QR: {e}")
    
    def _on_map_downloaded(self):
        """Xử lý sau khi tải map thành công"""
        self.update_map_info()
        # Enable nút START MOVING
        self.btn_start.config(state=tk.NORMAL)
        self.status_text.config(text="System: Sẵn sàng khởi động")
    
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
                self.log(f"✅ Đã cập nhật trạng thái vehicle: {status}")
                # Chỉ refresh vehicle info khi status là "online" và refresh_info=True
                if status == "online" and refresh_info:
                    self.root.after(0, self.refresh_vehicle_info)
            else:
                self.log(f"⚠️ Không thể cập nhật trạng thái vehicle: HTTP {response.status_code}")
                
        except requests.exceptions.RequestException as e:
            self.log(f"⚠️ Lỗi kết nối khi cập nhật trạng thái: {e}")
        except Exception as e:
            self.log(f"⚠️ Lỗi khi cập nhật trạng thái: {e}")
    
    def on_closing(self):
        """Xử lý khi đóng cửa sổ"""
        # Cập nhật trạng thái vehicle thành offline
        if self.vehicle_id:
            self.update_vehicle_status("offline")
        
        # Dừng tất cả các process đang chạy
        if self.is_running:
            self.stop_system()
        
        # Đóng cửa sổ
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = WorkerInterface(root)
    root.mainloop()