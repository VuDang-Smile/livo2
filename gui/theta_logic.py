import subprocess
import os
import threading
from datetime import datetime
from pathlib import Path
from languages.translate_engine import translator

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

class ThetaDriver(ttk.Frame):
    def __init__(self, log_callback, update_ui_theta_connected=None):
        super().__init__()
        self.log = log_callback # Hàm callback để ghi log ra giao diện
        self.update_ui_theta_connected = update_ui_theta_connected
        self.record_process = None
        self.output_path = None
        self.camera_info_publisher_process = None
        self.is_camera_connected = False
        self.is_active_theta = False
        self.create_control_panel()
    
    def create_control_panel(self):
        self.use4k_var = tk.BooleanVar(value=True)
        self.image_quality_var = tk.StringVar(value="high")  # Default: high
        self.jpeg_quality_var = tk.StringVar(value="85")  # Default: 85

    def launch_theta_driver(self, on_success_callback=None):
        """Launch theta_driver node với parameters"""
        # on_success_callback(True)
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
            
            self.log(f"Launching theta_driver with command: {cmd}")
            
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
                
                self.log(f"✗ theta_driver failed to start:")
                self.log(f"  Output: {error_msg}")
                self.is_active_theta = False
                if on_success_callback:
                    on_success_callback(self.is_active_theta)
                messagebox.showerror(
                    "Lỗi Launch",
                    translator.get("message.cannot_launch_theta_driver")
                )
                # self.launch_theta_btn.config(state=tk.NORMAL)
                return
            
            # self.log(f"✓ theta_driver process started (PID: {self.theta_driver_process.pid})")
            
            # Update status based on running processes
            if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                # Trạng thái: Theta Driver + Camera Info Publisher đang chạy
                self.is_active_theta = True
            else:
                self.is_active_theta = False
            
            if on_success_callback:
                on_success_callback(self.is_active_theta)

            # self.launch_theta_btn.config(state=tk.DISABLED)
            # self.tab_equirect.start_btn.config(state=tk.NORMAL)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_theta_driver_process(on_success_callback))
            
        except Exception as e:
            messagebox.showerror("Lỗi", translator.get("message.cannot_launch_theta_driver"))
    
    def check_theta_usb_connection(self, check_theta_usb_callback=None):
        """Check if Theta X camera is connected via USB"""
        def check_usb():
            try:
                # Check if Theta X is connected via USB using lsusb
                result = subprocess.run(['lsusb'], 
                                      capture_output=True, text=True, timeout=10)
                
                if result.returncode == 0 and 'theta' in result.stdout.lower():
                    self.is_camera_connected = True
                    self.log("✅ Theta X Connected")
                    self.log("Theta X camera detected via USB")
                else:
                    self.is_camera_connected = False
                    self.log("❌ Theta X Not Found")
                    self.log("Theta X camera not detected")
                    
            except Exception as e:
                self.is_camera_connected = False
                self.log("❌ Error Checking")
                self.log(f"Error checking Theta USB connection: {str(e)}")

            if check_theta_usb_callback:
                check_theta_usb_callback(self.is_camera_connected)
                
        threading.Thread(target=check_usb, daemon=True).start()
    
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
            
            self.log(f"Launching camera_info_publisher with command: {cmd}")
            
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
                
                # self.log(f"✗ camera_info_publisher failed to start:")
                self.log(f"  Output: {error_msg}")
                
                messagebox.showerror(
                    "Lỗi Launch",
                    translator.get("message.cannot_launch_camera_info_publisher")
                )
                # self.launch_camera_info_btn.config(state=tk.NORMAL)
                return
            
            self.log(f"✓ camera_info_publisher process started (PID: {self.camera_info_publisher_process.pid})")
            
            # Update status based on running processes
            if self.theta_driver_process and self.theta_driver_process.poll() is None:
                self.is_active_theta = True
                # self.status_label.config(
                #     text="Trạng thái: Theta Driver + Camera Info Publisher đang chạy",
                #     foreground="green"
                # )
            else:
                self.is_active_theta = False
            if self.update_ui_theta_connected:
                self.update_ui_theta_connected(self.is_active_theta)
                # self.status_label.config(
                #     text="Trạng thái: Camera Info Publisher đang chạy",
                #     foreground="green"
                # )
            # self.launch_camera_info_btn.config(state=tk.DISABLED)
            
            # Kiểm tra process sau 2 giây
            self.after(2000, self.check_camera_info_publisher_process(on_success_callback=None))
            
        except Exception as e:
            self.log(f"✗ camera_info_publisher failed to start: {e}")
            messagebox.showerror("Lỗi", translator.get("message.cannot_launch_camera_info_publisher"))
    
    def check_camera_info_publisher_process(self):
        """Kiểm tra xem camera_info_publisher process còn chạy không"""
        if self.camera_info_publisher_process:
            if self.camera_info_publisher_process.poll() is not None:
                # Update status based on other running processes
                if self.theta_driver_process and self.theta_driver_process.poll() is None:
                    self.is_active_theta = True
                    self.log("Camera Info Publisher is still running")
                    # self.status_label.config(
                    #     text="Trạng thái: Theta Driver đang chạy",
                    #     foreground="orange"
                    # )
                else:
                    self.is_active_theta = False
                    
                    self.log("Camera Info Publisher is not running")

                if self.update_ui_theta_connected:
                    self.update_ui_theta_connected(self.is_active_theta)

                    # self.status_label.config(
                    #     text="Trạng thái: Camera Info Publisher đã dừng",
                    #     foreground="orange"
                    # )

    def check_theta_driver_process(self):
        """Kiểm tra xem theta_driver process còn chạy không"""
        if self.theta_driver_process:
            if self.theta_driver_process.poll() is not None:
                # Update status based on other running processes
                if self.camera_info_publisher_process and self.camera_info_publisher_process.poll() is None:
                    self.is_active_theta = True

                    # self.status_label.config(
                    #     text="Trạng thái: Camera Info Publisher đang chạy",
                    #     foreground="green"
                    # )
                else:
                    self.is_active_theta = False
                    # self.status_label.config(
                    #     text="Trạng thái: Theta Driver đã dừng",
                    #     foreground="red"
                    # )
                if self.update_ui_theta_connected:
                    self.update_ui_theta_connected(self.is_active_theta)
                # self.launch_theta_btn.config(state=tk.NORMAL)
                self.stop_all()
    
    def stop_all(self):
        """Stop tất cả"""
        # Stop equirectangular tab
        # if self.tab_equirect.is_running:
        #     self.tab_equirect.stop_all()
        
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
            # self.is_active_theta = False
        
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
            self.is_active_theta = False
            if self.update_ui_theta_connected:
                self.update_ui_theta_connected(self.is_active_theta)
        
        # self.status_label.config(
        #     text="Trạng thái: Đã dừng",
        #     foreground="red"
        # )
        # self.launch_theta_btn.config(state=tk.NORMAL)
        # self.launch_camera_info_btn.config(state=tk.NORMAL)