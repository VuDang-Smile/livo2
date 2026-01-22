"""
Core Mapping Module
Chứa logic chính cho mapping: start mapping, bag playback, process monitoring, stop
"""
import subprocess
import os
import signal
import threading
import time
from pathlib import Path
from tkinter import messagebox

# Try to import ROS2
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    ROS2_AVAILABLE = True
except (ImportError, SystemError, OSError, AttributeError):
    ROS2_AVAILABLE = False


class MappingCore:
    """Quản lý core mapping logic"""
    
    def __init__(self, config, translator, logger_callback, validation_funcs, 
                 qr_scanner, qr_scan_enabled, qr_scan_frame_interval):
        """
        Initialize mapping core
        
        Args:
            config: BagMappingConfig instance
            translator: Translator instance
            logger_callback: Function để log messages
            validation_funcs: Dict chứa validation functions:
                - validate_config_file
                - validate_config_parameters
                - check_system_resources
                - validate_bag_file
            qr_scanner: QRScanner instance (có thể None)
            qr_scan_enabled: Boolean - enable QR scanning
            qr_scan_frame_interval: Integer - scan QR every N frames
        """
        self.config = config
        self.translator = translator
        self.log = logger_callback
        self.validation = validation_funcs
        self.qr_scanner = qr_scanner
        self.qr_scan_enabled = qr_scan_enabled
        self.qr_scan_frame_interval = qr_scan_frame_interval
        
        # Process state
        self.mapping_process = None
        self.bag_process = None
        self.is_mapping_running = False
        self.is_bag_playing = False
        self.is_stopping = False
    
    def start_mapping(self, bag_path, config_path, use_rviz, workspace_path, 
                     drive_ws_path, root, ui_callbacks):
        """
        Bắt đầu mapping process
        
        Args:
            bag_path: Path đến bag file/folder
            config_path: Path đến config file
            use_rviz: Boolean - sử dụng RViz2
            workspace_path: Workspace path
            drive_ws_path: Drive workspace path
            root: Tkinter root window
            ui_callbacks: Dict chứa UI callbacks:
                - update_status_label: function(text, color)
                - disable_start_button: function()
                - start_bag_playback: function()
                - start_qr_scanning: function()
        
        Returns:
            True nếu thành công, False nếu không
        """
        if self.is_mapping_running:
            self.log(self.translator.get('log.mapping_already_running', '⚠️ Mapping is already running'))
            return False
        
        if not bag_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_bag_first', 'Please select bag folder before starting mapping & playback.')
            )
            return False
        
        ws_setup = self.config.get_workspace_setup_script()
        if not ws_setup.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"Workspace setup not found at: {ws_setup}\n"
                f"{self.translator.get('message.build_workspace_first', 'Please build workspace first.')}"
            )
            return False
        
        drive_ws_setup = self.config.get_drive_ws_setup_script()
        use_drive_ws = drive_ws_setup.exists()
        
        if not config_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_config_first', 'Please select config file before starting mapping.')
            )
            return False
        
        config_path_obj = Path(config_path)
        if not config_path_obj.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"{self.translator.get('message.config_not_exists', 'Config file does not exist')}: {config_path}\n"
                f"{self.translator.get('message.select_config_again', 'Please select config file again.')}"
            )
            return False
        
        # Pre-flight checks
        self.log(self.translator.get('log.preflight_checks', '🔍 Running pre-flight checks to prevent crashes...'))
        
        if not self.validation['validate_config_file'](config_path_obj):
            return False
        
        if not self.validation['validate_config_parameters'](config_path_obj):
            return False
        
        if not self.validation['check_system_resources']():
            return False
        
        if not self.validation['validate_bag_file'](bag_path, workspace_path):
            return False
        
        self.log(self.translator.get('log.preflight_checks_passed', '✅ All pre-flight checks passed!'))
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            rviz_arg = "True" if use_rviz else "False"
            launch_file = "mapping_mid360_equirectangular.launch.py"
            config_name = config_path_obj.name
            
            if use_drive_ws:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={config_path}"
                )
                self.log(self.translator.get('log.source_drive_ws', '✅ Will source: ROS2 base -> drive_ws -> ws'))
            else:
                mapping_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"ros2 launch fast_livo {launch_file} "
                    f"use_rviz:={rviz_arg} "
                    f"params_file:={config_path}"
                )
                self.log(self.translator.get('log.source_ws_only', '⚠️ Will source: ROS2 base -> ws (no drive_ws)'))
            
            # Cleanup ROS2 resources trước khi start
            from .utils import cleanup_ros2_resources
            cleanup_ros2_resources(logger_callback=self.log)
            
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            env['RMW_FASTRTPS_USE_QOS_FROM_XML'] = '0'
            
            self.log("=" * 60)
            self.log(self.translator.get('log.starting_mapping_node', '🚀 Starting Mapping Node'))
            self.log(f"{self.translator.get('log.config_file', '📋 Config file')}: {config_name}")
            self.log(f"{self.translator.get('log.config_path', '📁 Config path')}: {config_path}")
            self.log(f"{self.translator.get('log.launch_file', '🎯 Launch file')}: {launch_file}")
            rviz_text = self.translator.get('log.rviz_enabled', 'Yes') if use_rviz else self.translator.get('log.rviz_disabled', 'No')
            self.log(f"{self.translator.get('log.rviz2', '👁️ RViz2')}: {rviz_text}")
            self.log("=" * 60)
            
            self.log(self.translator.get('log.starting_mapping_node_process', '📡 Starting mapping node...'))
            self.mapping_process = subprocess.Popen(
                mapping_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            # Graceful startup với health check
            if not self._wait_for_process_health(max_wait_time=10):
                if self.mapping_process:
                    if self.mapping_process.poll() is not None:
                        error_output = ""
                        try:
                            error_output = self.mapping_process.stdout.read() if self.mapping_process.stdout else self.translator.get('log.no_output', 'No output')
                        except:
                            pass
                        self.log(f"❌ {self.translator.get('log.mapping_process_exited', 'Mapping process exited immediately with code')}: {self.mapping_process.returncode}")
                        if error_output:
                            self.log(f"{self.translator.get('log.output', 'Output')}: {error_output[:500]}")
                        messagebox.showerror(
                            self.translator.get('dialog.error', 'Error'),
                            self.translator.get('message.mapping_exited_immediately', 'Mapping process exited immediately. Check log for details.')
                        )
                    self.mapping_process = None
                return False
            
            self.is_mapping_running = True
            self.log(self.translator.get('log.mapping_node_started', '✅ Mapping node started successfully'))
            
            ui_callbacks['disable_start_button']()
            ui_callbacks['update_status_label'](
                self.translator.get('label.status_mapping_running', 'Status: 📡 Mapping node running'),
                "orange"
            )
            
            # Start monitoring thread
            threading.Thread(target=self.monitor_mapping_process, args=(root, ui_callbacks), daemon=True).start()
            
            self.log("=" * 60)
            self.log(self.translator.get('log.mapping_node_ready', '✅ Mapping node is ready!'))
            self.log("=" * 60)
            
            # Auto-start bag playback
            self.log(self.translator.get('log.auto_start_bag', '▶️ Auto-starting bag playback...'))
            return True
            
        except Exception as e:
            error_msg = f"{self.translator.get('message.cannot_start_mapping', 'Cannot start mapping')}: {e}"
            self.log(f"❌ {error_msg}")
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            return False
    
    def start_bag_playback(self, bag_path, bag_rate, workspace_path, drive_ws_path, 
                           root, ui_callbacks):
        """
        Bắt đầu play bag sau khi mapping node đã chạy
        
        Args:
            bag_path: Path đến bag file/folder
            bag_rate: Bag playback rate (float)
            workspace_path: Workspace path
            drive_ws_path: Drive workspace path
            root: Tkinter root window
            ui_callbacks: Dict chứa UI callbacks:
                - start_qr_scanning: function()
        
        Returns:
            True nếu thành công, False nếu không
        """
        if self.is_bag_playing:
            self.log(self.translator.get('log.bag_already_playing', '⚠️ Bag is already playing'))
            return False
        
        if not self.is_mapping_running:
            self.log(self.translator.get('log.mapping_not_running', '⚠️ Mapping node is not running. Cannot start bag playback.'))
            return False
        
        if not bag_path:
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                self.translator.get('message.select_bag_first', 'Please select bag folder before starting mapping & playback.')
            )
            return False
        
        bag_path_obj = Path(bag_path)
        if not bag_path_obj.exists():
            error_msg = self.translator.get('message.bag_folder_not_exists', 'Bag folder does not exist: {path}').replace('{path}', bag_path)
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            return False
        
        ws_setup = self.config.get_workspace_setup_script()
        if not ws_setup.exists():
            messagebox.showerror(
                self.translator.get('dialog.error', 'Error'),
                f"Workspace setup not found at: {ws_setup}\n"
                f"{self.translator.get('message.build_workspace_first', 'Please build workspace first.')}"
            )
            return False
        
        drive_ws_setup = self.config.get_drive_ws_setup_script()
        use_drive_ws = drive_ws_setup.exists()
        
        try:
            ros2_setup = "/opt/ros/jazzy/setup.bash"
            
            # Dùng bag_rate
            try:
                bag_rate_value = float(bag_rate)
            except ValueError:
                bag_rate_value = self.config.bag_rate
                self.log(self.translator.get('log.invalid_bag_rate', '⚠️ Invalid bag rate, fallback to 0.5x'))
            
            bag_play_cmd = f"ros2 bag play {bag_path} --rate {bag_rate_value}"
            
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            if 'ROS_DOMAIN_ID' not in env:
                env['ROS_DOMAIN_ID'] = '0'
            
            self.log("=" * 60)
            self.log(self.translator.get('log.starting_bag_playback', '▶️ Starting Bag Playback'))
            self.log(self.translator.get('log.bag_name', '📁 Bag: {name}').replace('{name}', bag_path_obj.name))
            self.log(self.translator.get('log.bag_path', '📁 Bag path: {path}').replace('{path}', bag_path))
            self.log(self.translator.get('log.bag_rate', '⚡ Rate: {rate}x').replace('{rate}', str(bag_rate_value)))
            self.log("=" * 60)
            
            if use_drive_ws:
                bag_cmd = (
                    f"source {ros2_setup} && "
                    f"source {drive_ws_setup} && "
                    f"source {ws_setup} && "
                    f"{bag_play_cmd}"
                )
            else:
                bag_cmd = (
                    f"source {ros2_setup} && "
                    f"source {ws_setup} && "
                    f"{bag_play_cmd}"
                )
            
            self.log(self.translator.get('log.launching_bag_play', '▶️ Launching ros2 bag play...'))
            self.bag_process = subprocess.Popen(
                bag_cmd,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                env=env,
                preexec_fn=os.setsid if hasattr(os, 'setsid') else None
            )
            
            self.is_bag_playing = True
            self.log(self.translator.get('log.bag_playback_started', '✅ Bag playback started'))
            
            threading.Thread(target=self.monitor_bag_process, args=(root,), daemon=True).start()
            
            # Start QR code scanning subscriber if enabled
            if self.qr_scan_enabled and self.qr_scanner and 'start_qr_scanning' in ui_callbacks:
                ui_callbacks['start_qr_scanning']()
            
            return True
            
        except Exception as e:
            error_msg = f"Cannot start bag playback: {e}"
            self.log(f"❌ {error_msg}")
            messagebox.showerror(self.translator.get('dialog.error', 'Error'), error_msg)
            return False
    
    def monitor_mapping_process(self, root, ui_callbacks):
        """Theo dõi mapping process và phát hiện crash"""
        if not self.mapping_process:
            return
        
        try:
            for line in iter(self.mapping_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_mapping_running:
                    line_lower = line.lower()
                    if any(keyword in line_lower for keyword in ['error', 'warning', 'started', 'ready', 'failed', 'died', 'crash', 'segmentation', 'signal']):
                        self.log(f"[Mapping] {line.strip()}")
                    
                    # Phát hiện process crash
                    if 'process has died' in line_lower or 'process died' in line_lower:
                        self.log(f"❌ [Mapping] {line.strip()}")
                        time.sleep(0.5)
                        if self.mapping_process.poll() is not None:
                            exit_code = self.mapping_process.returncode
                            self._handle_mapping_crash(exit_code, line, root, ui_callbacks)
                            break
                else:
                    break
            
            # Kiểm tra lại sau khi đọc hết output
            if self.is_mapping_running and self.mapping_process:
                if self.mapping_process.poll() is not None:
                    exit_code = self.mapping_process.returncode
                    if exit_code != 0 and not (self.is_stopping or exit_code == -15):
                        self._handle_mapping_crash(exit_code, "Process exited unexpectedly", root, ui_callbacks)
                        
        except Exception as e:
            if self.is_mapping_running:
                self.log(f"⚠️ {self.translator.get('log.error_reading_mapping_output', 'Error reading mapping output')}: {e}")
                if self.mapping_process and self.mapping_process.poll() is not None:
                    self._handle_mapping_crash(self.mapping_process.returncode, str(e), root, ui_callbacks)
    
    def _handle_mapping_crash(self, exit_code, error_line, root, ui_callbacks):
        """Xử lý khi mapping process bị crash"""
        if not self.is_mapping_running:
            return
        
        if self.is_stopping or exit_code == -15:
            self.is_mapping_running = False
            self.log(self.translator.get('log.mapping_stopped_by_user', '✅ Mapping process stopped by user'))
            return
        
        self.is_mapping_running = False
        
        # Xác định loại lỗi
        error_type = "Unknown error"
        error_description = ""
        
        if exit_code == -11:
            error_type = "Segmentation Fault (SIGSEGV)"
            error_description = self.translator.get('log.crash_segmentation_fault', 
                'The mapping process crashed due to a segmentation fault.')
        elif exit_code == -6:
            error_type = "Abort (SIGABRT)"
            error_description = self.translator.get('log.crash_abort',
                'The mapping process was aborted.')
        elif exit_code < 0:
            error_type = f"Signal {abs(exit_code)}"
            signal_num = abs(exit_code)
            error_description = self.translator.get('log.crash_signal',
                'The mapping process was terminated by signal {signal}.').replace('{signal}', str(signal_num))
        else:
            error_type = f"Exit code {exit_code}"
            error_description = self.translator.get('log.crash_exit_code',
                'The mapping process exited with code {code}.').replace('{code}', str(exit_code))
        
        self.log("=" * 60)
        self.log(f"❌ {self.translator.get('log.mapping_process_crashed', 'MAPPING PROCESS CRASHED')}")
        self.log(f"   {self.translator.get('log.error_type', 'Error Type')}: {error_type}")
        self.log(f"   {self.translator.get('log.exit_code', 'Exit Code')}: {exit_code}")
        self.log("=" * 60)
        
        root.after(0, lambda: ui_callbacks['update_status_label'](
            self.translator.get('label.status_crashed', 'Status: ❌ Crashed'),
            "red"
        ))
        root.after(0, lambda: ui_callbacks.get('enable_start_button', lambda: None)())
        
        if self.is_bag_playing:
            self.log(self.translator.get('log.stopping_bag_due_to_crash', '⚠️ Stopping bag playback due to mapping crash...'))
            self.cleanup_processes(root, ui_callbacks)
        
        root.after(0, lambda: messagebox.showerror(
            self.translator.get('dialog.mapping_crashed_title', 'Mapping Process Crashed'),
            f"{self.translator.get('dialog.mapping_crashed_message', 'The mapping process has crashed!')}\n\n"
            f"{self.translator.get('dialog.error_type', 'Error Type')}: {error_type}\n"
            f"{self.translator.get('dialog.exit_code', 'Exit Code')}: {exit_code}\n\n"
            f"{error_description}"
        ))
    
    def monitor_bag_process(self, root):
        """Theo dõi output của ros2 bag play"""
        if not self.bag_process:
            return
        
        try:
            for line in iter(self.bag_process.stdout.readline, ''):
                if not line:
                    break
                if self.is_bag_playing:
                    if 'paused' in line.lower() or 'playing' in line.lower():
                        self.log(f"[Bag] {line.strip()}")
                else:
                    break
            
            if self.is_bag_playing:
                self.log(self.translator.get('log.bag_playback_finished', '✅ Bag playback finished'))
                self.is_bag_playing = False
                root.after(0, lambda: messagebox.showinfo(
                    self.translator.get('dialog.bag_playback_finished_title', 'Bag Playback Finished'),
                    self.translator.get('dialog.bag_playback_finished_message', 'Bag playback has completed successfully!')
                ))
        except Exception as e:
            if self.is_bag_playing:
                self.log(self.translator.get('log.error_reading_bag', '⚠️ Error reading bag output: {error}').replace('{error}', str(e)))
    
    def cleanup_processes(self, root, ui_callbacks):
        """Dọn dẹp tất cả processes"""
        self.is_stopping = True
        
        # Dừng bag play trước
        if self.bag_process:
            try:
                self.log(self.translator.get('log.stopping_bag_playback', 'Stopping bag playback...'))
                if hasattr(os, 'setsid'):
                    try:
                        os.killpg(os.getpgid(self.bag_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                else:
                    self.bag_process.terminate()
                
                try:
                    self.bag_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    if hasattr(os, 'setsid'):
                        try:
                            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        self.bag_process.kill()
                    self.bag_process.wait()
            except Exception as e:
                self.log(self.translator.get('log.error_stopping_bag', '⚠️ Error stopping bag process: {error}').replace('{error}', str(e)))
            finally:
                self.bag_process = None
                self.is_bag_playing = False
        
        # Sau đó dừng mapping node
        if self.mapping_process:
            try:
                self.log(self.translator.get('log.stopping_mapping_node', 'Stopping mapping node...'))
                if hasattr(os, 'setsid'):
                    try:
                        os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                else:
                    self.mapping_process.terminate()
                
                try:
                    self.mapping_process.wait(timeout=self.config.process_terminate_timeout)
                    self.log(self.translator.get('log.mapping_node_stopped', '✅ Mapping node stopped gracefully'))
                except subprocess.TimeoutExpired:
                    if hasattr(os, 'setsid'):
                        try:
                            os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    else:
                        self.mapping_process.kill()
                    self.mapping_process.wait()
            except Exception as e:
                self.log(f"⚠️ {self.translator.get('log.error_stopping_mapping', 'Error stopping mapping process')}: {e}")
            finally:
                self.mapping_process = None
                self.is_mapping_running = False
        
        # Cleanup ROS2 resources
        from .utils import cleanup_ros2_resources
        cleanup_ros2_resources(logger_callback=self.log)
        
        # Reset flag
        self.is_stopping = False
    
    def _wait_for_process_health(self, max_wait_time=10):
        """Đợi process khởi động và kiểm tra health"""
        check_interval = 0.5
        max_checks = int(max_wait_time / check_interval)
        
        for i in range(max_checks):
            time.sleep(check_interval)
            
            if not self.mapping_process:
                return False
            
            if self.mapping_process.poll() is not None:
                return False
            
            # Sau 3 giây đầu tiên, coi như process đã khởi động
            if i >= 6:  # 3 seconds
                return True
        
        if self.mapping_process and self.mapping_process.poll() is None:
            return True
        
        return False
