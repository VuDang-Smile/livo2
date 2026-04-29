import subprocess
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

class Recorder:
    SYNC_TOPIC_MAP = {
        "/livox/lidar": "/record_sync/livox/lidar",
        "/livox/imu": "/record_sync/livox/imu",
        "/image_raw": "/record_sync/image_raw",
        "/camera_info": "/record_sync/camera_info",
    }

    def __init__(self, log_callback):
        self.log = log_callback # Hàm callback để ghi log ra giao diện
        self.is_recording = False
        self.record_process = None
        self.sync_process = None
        self.output_path = None

    def verify_source(self, setup_script, name):
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            verify_cmd = f"source {ros2_setup} && source {setup_script} && ros2 pkg list | head -1"
            result = subprocess.run(
                verify_cmd, shell=True, executable="/bin/bash",
                capture_output=True, text=True, timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            self.log(f"⚠️  Lỗi khi verify source {name}: {e}")
            return False

    def _build_source_prefix(self, ws_setup, drive_ws_setup=None, use_drive_ws=False):
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        parts = [f"source {ros2_setup}"]
        if use_drive_ws and drive_ws_setup and drive_ws_setup.exists():
            parts.append(f"source {drive_ws_setup}")
        parts.append(f"source {ws_setup}")
        return " && ".join(parts)

    def _start_sync_relay(self, ws_setup, drive_ws_setup, use_drive_ws, env):
        source_prefix = self._build_source_prefix(ws_setup, drive_ws_setup, use_drive_ws)
        relay_cmd = f"{source_prefix} && ros2 run livox_msg_converter recording_sync_relay_node"

        self.log("⏱️  Khởi động recording sync relay...")
        self.sync_process = subprocess.Popen(
            relay_cmd,
            shell=True,
            executable="/bin/bash",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            env=env
        )

        threading.Thread(
            target=self._monitor_process,
            args=(self.sync_process, "[SYNC]"),
            daemon=True
        ).start()

        sync_topics = list(self.SYNC_TOPIC_MAP.values())
        deadline = time.time() + 4.0
        while time.time() < deadline:
            if self.sync_process.poll() is not None:
                self.log("⚠️  recording_sync_relay_node đã thoát sớm, fallback về raw topics")
                self.sync_process = None
                return False

            check_cmd = f"{source_prefix} && ros2 topic list"
            result = subprocess.run(
                check_cmd,
                shell=True,
                executable="/bin/bash",
                capture_output=True,
                text=True,
                timeout=2,
                env=env
            )
            if result.returncode == 0:
                topic_list = result.stdout
                if all(topic in topic_list for topic in sync_topics):
                    self.log("✅ Recording sync relay đã sẵn sàng")
                    return True
            time.sleep(0.3)

        self.log("⚠️  Không xác nhận được sync relay kịp thời, fallback về raw topics")
        try:
            self.sync_process.terminate()
            self.sync_process.wait(timeout=2)
        except Exception:
            try:
                self.sync_process.kill()
                self.sync_process.wait(timeout=2)
            except Exception:
                pass
        self.sync_process = None
        return False

    def start(self, output_dir_str, topic_vars, workspace_path, drive_ws_path, max_bag_size_str):
        if self.is_recording:
            return False

        output_dir = Path(output_dir_str)
        if not output_dir.exists():
            output_dir.mkdir(parents=True, exist_ok=True)

        # Logic kiểm tra source scripts
        drive_ws_setup = drive_ws_path / "install" / "setup.sh"
        ws_setup = workspace_path / "install" / "setup.sh"
        
        use_drive_ws = drive_ws_setup.exists() and self.verify_source(drive_ws_setup, "drive_ws")
        if not ws_setup.exists():
            self.log(f"❌ Không tìm thấy workspace setup: {ws_setup}")
            return False
        
        # Xử lý Topics
        selected_topics = [topic for topic, var in topic_vars.items() if var.get()]
        if not selected_topics:
            from languages.translate_engine import translator
            messagebox.showerror(translator.get("dialog.error"), translator.get("message.select_at_least_one_topic"))
            return False

        # Xử lý Bag Size
        try:
            max_size = float(max_bag_size_str)
            max_size_bytes = int(max_size * 1024 * 1024 * 1024) if max_size > 0 else 0
        except:
            max_size_bytes = 2 * 1024 * 1024 * 1024

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"recording_{timestamp}"
        self.output_path = output_dir / bag_name

        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '0')
            env['PYTHONUNBUFFERED'] = '1'

            actual_topics = list(selected_topics)
            syncable_topics = [topic for topic in selected_topics if topic in self.SYNC_TOPIC_MAP]
            if syncable_topics:
                sync_ready = self._start_sync_relay(ws_setup, drive_ws_setup, use_drive_ws, env)
                if sync_ready:
                    actual_topics = [self.SYNC_TOPIC_MAP.get(topic, topic) for topic in selected_topics]
                    self.log("🕒 Sẽ record các topic đã đồng bộ timestamp:")
                    for raw_topic in syncable_topics:
                        self.log(f"   {raw_topic} -> {self.SYNC_TOPIC_MAP[raw_topic]}")
                else:
                    self.log("⚠️  Sync relay chưa sẵn sàng, tiếp tục record raw topics")

            source_prefix = self._build_source_prefix(ws_setup, drive_ws_setup, use_drive_ws)
            topics_str = " ".join(actual_topics)
            bag_cmd = f"ros2 bag record -o {self.output_path} {topics_str}"
            if max_size_bytes > 0:
                bag_cmd += f" --max-bag-size {max_size_bytes}"

            full_cmd = f"{source_prefix} && {bag_cmd}"
            
            self.record_process = subprocess.Popen(
                full_cmd, shell=True, executable="/bin/bash",
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, bufsize=1, env=env
            )
            self.is_recording = True
            threading.Thread(
                target=self._monitor_process,
                args=(self.record_process, "[ROS2]"),
                daemon=True
            ).start()
            return True
        except Exception as e:
            self.log(f"❌ Lỗi khởi động: {e}")
            if self.sync_process:
                try:
                    self.sync_process.terminate()
                    self.sync_process.wait(timeout=2)
                except Exception:
                    pass
                self.sync_process = None
            return False

    def stop(self):
        if self.record_process:
            try:
                # Terminate process
                self.record_process.terminate()
                # Wait với timeout 5 giây
                try:
                    self.record_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # Nếu timeout, kill process
                    try:
                        self.record_process.kill()
                        self.record_process.wait()
                    except Exception as e:
                        self.log(f"⚠️  Lỗi khi kill process: {e}")
            except Exception as e:
                self.log(f"⚠️  Lỗi khi dừng process: {e}")
            finally:
                self.record_process = None

        if self.sync_process:
            try:
                self.sync_process.terminate()
                try:
                    self.sync_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self.sync_process.kill()
                    self.sync_process.wait(timeout=2)
            except Exception as e:
                self.log(f"⚠️  Lỗi khi dừng sync relay: {e}")
            finally:
                self.sync_process = None

        self.is_recording = False
        return self.output_path

    def _monitor_process(self, process, prefix):
        """Monitor process output trong thread riêng - tránh crash khi process bị None"""
        if not process:
            return
        
        try:
            for line in iter(process.stdout.readline, ''):
                # Kiểm tra process còn tồn tại và đang recording
                if process.poll() is not None:
                    break
                
                line = line.strip()
                if line and any(k in line.lower() for k in ['error', 'warn', 'recording', 'bag']):
                    self.log(f"{prefix} {line}")
        except (ValueError, AttributeError):
            # Process đã bị terminate hoặc stdout đã đóng - đây là bình thường
            pass
        except Exception as e:
            # Log các exception khác nhưng không crash
            self.log(f"⚠️  Lỗi trong monitor thread {prefix}: {e}")
