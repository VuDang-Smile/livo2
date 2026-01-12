import subprocess
import os
import threading
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

class Recorder:
    def __init__(self, log_callback):
        self.log = log_callback # Hàm callback để ghi log ra giao diện
        self.is_recording = False
        self.record_process = None
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

        # Build Command
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        topics_str = " ".join(selected_topics)
        bag_cmd = f"ros2 bag record -o {self.output_path} {topics_str}"
        if max_size_bytes > 0:
            bag_cmd += f" --max-bag-size {max_size_bytes}"

        full_cmd = f"source {ros2_setup} && "
        if use_drive_ws: full_cmd += f"source {drive_ws_setup} && "
        full_cmd += f"source {ws_setup} && {bag_cmd}"

        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '0')
            
            self.record_process = subprocess.Popen(
                full_cmd, shell=True, executable="/bin/bash",
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, bufsize=1, env=env
            )
            self.is_recording = True
            threading.Thread(target=self._monitor_process, daemon=True).start()
            return True
        except Exception as e:
            self.log(f"❌ Lỗi khởi động: {e}")
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
        self.is_recording = False
        return self.output_path

    def _monitor_process(self):
        """Monitor process output trong thread riêng - tránh crash khi process bị None"""
        # Lưu reference của process trước khi đọc để tránh race condition
        process = self.record_process
        if not process:
            return
        
        try:
            for line in iter(process.stdout.readline, ''):
                # Kiểm tra process còn tồn tại và đang recording
                if not self.is_recording or process.poll() is not None:
                    break
                
                line = line.strip()
                if line and any(k in line.lower() for k in ['error', 'warn', 'recording', 'bag']):
                    self.log(f"[ROS2] {line}")
        except (ValueError, AttributeError):
            # Process đã bị terminate hoặc stdout đã đóng - đây là bình thường
            pass
        except Exception as e:
            # Log các exception khác nhưng không crash
            if self.is_recording:
                self.log(f"⚠️  Lỗi trong monitor thread: {e}")