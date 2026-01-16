#!/usr/bin/env python3
"""
Livox Config Updater
Module to update IP address in Livox MID360 config files
"""

import json
import os
import subprocess
import sys
from pathlib import Path
from datetime import datetime
import ipaddress


class LivoxConfigUpdater:
    """Class to update Livox MID360 config files"""
    
    def __init__(self, project_root=None, translator=None):
        """
        Initialize config updater
        
        Args:
            project_root: Root path of livo2 project. If None, auto-detect from current file location.
            translator: Translator instance for multi-language support
        """
        if project_root is None:
            # Auto-detect project root (assuming this file is in gui/ directory)
            current_file = Path(__file__).resolve()
            project_root = current_file.parent.parent
        
        self.project_root = Path(project_root)
        self.translator = translator
        
        # Define config file paths
        self.config_files = {
            'drive_ws': self.project_root / "dependencies" / "drive_ws" / "src" / "livox_ros_driver2" / "config" / "MID360_config.json",
            'sdk2': self.project_root / "dependencies" / "Livox-SDK2" / "samples" / "livox_lidar_quick_start" / "mid360_config.json"
        }
        
    
    def validate_ip(self, ip):
        """
        Validate IP address format
        
        Args:
            ip: IP address string
            
        Returns:
            tuple: (is_valid, error_message)
        """
        try:
            ipaddress.ip_address(ip)
            return True, None
        except ValueError as e:
            return False, f"Invalid IP address format: {e}"
    
    def read_config(self, file_path):
        """
        Read config file
        
        Args:
            file_path: Path to config file
            
        Returns:
            dict: Config data or None if error
        """
        try:
            if not file_path.exists():
                return None
            
            with open(file_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"Error reading config {file_path}: {e}")
            return None
    
    def write_config(self, file_path, config):
        """
        Write config file
        
        Args:
            file_path: Path to config file
            config: Config data dict
            
        Returns:
            bool: True if success, False otherwise
        """
        try:
            # Ensure directory exists
            file_path.parent.mkdir(parents=True, exist_ok=True)
            
            # Write with pretty formatting
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2, ensure_ascii=False)
            return True
        except Exception as e:
            print(f"Error writing config {file_path}: {e}")
            return False
    
    def update_drive_ws_config(self, new_ip):
        """
        Update drive_ws MID360_config.json
        
        Args:
            new_ip: New IP address string
            
        Returns:
            tuple: (success, message)
        """
        config_path = self.config_files['drive_ws']
        
        # Read config
        config = self.read_config(config_path)
        if config is None:
            return False, f"Failed to read config {config_path}"
        
        updated = False
        
        # Update MID360.host_net_info[0].lidar_ip[0]
        if "MID360" in config and "host_net_info" in config["MID360"]:
            host_info = config["MID360"]["host_net_info"]
            
            if isinstance(host_info, list) and len(host_info) > 0:
                if "lidar_ip" in host_info[0]:
                    if isinstance(host_info[0]["lidar_ip"], list):
                        host_info[0]["lidar_ip"][0] = new_ip
                        updated = True
                    else:
                        host_info[0]["lidar_ip"] = [new_ip]
                        updated = True
        
        # Update lidar_configs[0].ip
        if "lidar_configs" in config and len(config["lidar_configs"]) > 0:
            if "ip" in config["lidar_configs"][0]:
                config["lidar_configs"][0]["ip"] = new_ip
                updated = True
        
        if not updated:
            return False, "No IP fields found to update in config"
        
        # Write config
        if self.write_config(config_path, config):
            return True, f"Successfully updated {config_path.name}"
        else:
            return False, f"Failed to write config {config_path}"
    
    def update_sdk2_config(self, new_ip):
        """
        Update Livox-SDK2 mid360_config.json
        
        Args:
            new_ip: New IP address string
            
        Returns:
            tuple: (success, message)
        """
        config_path = self.config_files['sdk2']
        
        # Read config
        config = self.read_config(config_path)
        if config is None:
            return False, f"Failed to read config {config_path}"
        
        updated = False
        
        # Update MID360.host_net_info[0].lidar_ip[0]
        if "MID360" in config and "host_net_info" in config["MID360"]:
            host_info = config["MID360"]["host_net_info"]
            
            if isinstance(host_info, list) and len(host_info) > 0:
                if "lidar_ip" in host_info[0]:
                    if isinstance(host_info[0]["lidar_ip"], list):
                        host_info[0]["lidar_ip"][0] = new_ip
                        updated = True
                    else:
                        host_info[0]["lidar_ip"] = [new_ip]
                        updated = True
        
        if not updated:
            return False, "No IP fields found to update in config"
        
        # Write config
        if self.write_config(config_path, config):
            return True, f"Successfully updated {config_path.name}"
        else:
            return False, f"Failed to write config {config_path}"
    
    def update_config_files(self, new_ip):
        """
        Update IP in both config files
        
        Args:
            new_ip: New IP address string
            
        Returns:
            dict: Results with success status and messages
        """
        # Validate IP
        is_valid, error_msg = self.validate_ip(new_ip)
        if not is_valid:
            return {
                'success': False,
                'message': error_msg,
                'drive_ws': {'success': False, 'message': error_msg},
                'sdk2': {'success': False, 'message': error_msg}
            }
        
        # Update both configs
        drive_ws_result = self.update_drive_ws_config(new_ip)
        sdk2_result = self.update_sdk2_config(new_ip)
        
        drive_ws_success, drive_ws_msg = drive_ws_result
        sdk2_success, sdk2_msg = sdk2_result
        
        overall_success = drive_ws_success and sdk2_success
        
        if overall_success:
            message = f"Successfully updated IP to {new_ip} in both config files"
        elif drive_ws_success or sdk2_success:
            message = f"Partially updated: {drive_ws_msg if drive_ws_success else ''} {sdk2_msg if sdk2_success else ''}"
        else:
            message = f"Failed to update configs: {drive_ws_msg}; {sdk2_msg}"
        
        return {
            'success': overall_success,
            'message': message,
            'drive_ws': {
                'success': drive_ws_success,
                'message': drive_ws_msg
            },
            'sdk2': {
                'success': sdk2_success,
                'message': sdk2_msg
            }
        }
    
    def get_current_ip(self):
        """
        Get current IP from config files
        
        Returns:
            dict: Current IPs from both config files
        """
        result = {
            'drive_ws': None,
            'sdk2': None
        }
        
        # Read drive_ws config
        config = self.read_config(self.config_files['drive_ws'])
        if config:
            # Try to get from MID360.host_net_info
            if "MID360" in config and "host_net_info" in config["MID360"]:
                host_info = config["MID360"]["host_net_info"]
                if isinstance(host_info, list) and len(host_info) > 0:
                    if "lidar_ip" in host_info[0]:
                        if isinstance(host_info[0]["lidar_ip"], list) and len(host_info[0]["lidar_ip"]) > 0:
                            result['drive_ws'] = host_info[0]["lidar_ip"][0]
            
            # Try to get from lidar_configs
            if result['drive_ws'] is None and "lidar_configs" in config and len(config["lidar_configs"]) > 0:
                if "ip" in config["lidar_configs"][0]:
                    result['drive_ws'] = config["lidar_configs"][0]["ip"]
        
        # Read sdk2 config
        config = self.read_config(self.config_files['sdk2'])
        if config:
            if "MID360" in config and "host_net_info" in config["MID360"]:
                host_info = config["MID360"]["host_net_info"]
                if isinstance(host_info, list) and len(host_info) > 0:
                    if "lidar_ip" in host_info[0]:
                        if isinstance(host_info[0]["lidar_ip"], list) and len(host_info[0]["lidar_ip"]) > 0:
                            result['sdk2'] = host_info[0]["lidar_ip"][0]
        
        return result
    
    def build_livox_driver(self, callback=None):
        """
        Build Livox driver after updating IP (giống recorder)
        
        Args:
            callback: Optional callback function to log progress
            
        Returns:
            tuple: (success, message)
        """
        # Path to build script
        build_script = self.project_root / "dependencies" / "drive_ws" / "build.sh"
        
        if not build_script.exists():
            # Try alternative path
            build_script = self.project_root / "dependencies" / "drive_ws" / "src" / "livox_ros_driver2" / "build.sh"
            if not build_script.exists():
                return False, f"Build script not found. Checked: {build_script}"
        
        try:
            if callback:
                callback(f"🔨 Building Livox driver with new IP...")
                callback(f"Build script: {build_script}")
            
            # Run build script với Popen để xử lý "Press Enter to exit..." và phát hiện build xong
            # Build script có trap cleanup EXIT nên sẽ hỏi "Press Enter to exit..."
            # Chúng ta cần tự động gửi Enter và phát hiện khi build xong
            process = subprocess.Popen(
                ["bash", str(build_script)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.PIPE,
                text=True,
                cwd=str(build_script.parent),
                bufsize=1
            )
            
            # Đọc output real-time và tự động phát hiện khi build xong
            output_lines = []
            build_completed = False
            
            while True:
                line = process.stdout.readline()
                if not line:
                    # Kiểm tra process đã kết thúc chưa
                    if process.poll() is not None:
                        break
                    continue
                
                output_lines.append(line)
                line_stripped = line.strip()
                
                if callback:
                    callback(line_stripped)
                
                # Phát hiện khi build xong (trước khi hỏi "Press Enter to exit")
                if "Build completed!" in line or ("Summary:" in line and "package finished" in line):
                    build_completed = True
                    if callback:
                        build_detected_msg = self.translator.get('log.build_detected_completed', '✅ Build detected as completed!') if self.translator else "✅ Build detected as completed!"
                        callback(build_detected_msg)
                
                # Nếu thấy "Press Enter to exit", tự động gửi Enter và kết thúc ngay
                if "Press Enter to exit" in line:
                    try:
                        process.stdin.write("\n")
                        process.stdin.flush()
                        # Đợi một chút để process xử lý Enter
                        import time
                        time.sleep(0.3)
                        # Kết thúc ngay sau khi gửi Enter
                        break
                    except:
                        pass
                
                # Nếu đã phát hiện build completed và thấy "Build completed!", kết thúc ngay
                if build_completed and "Build completed!" in line:
                    if callback:
                        build_completing_msg = self.translator.get('log.build_completing', '✅ Build completed, finishing...') if self.translator else "✅ Build completed, finishing..."
                        callback(build_completing_msg)
                    # Đọc thêm một vài dòng nếu có (non-blocking)
                    import time
                    import select
                    time.sleep(0.2)
                    try:
                        # Đọc thêm tối đa 5 dòng với timeout ngắn
                        for _ in range(5):
                            if process.poll() is not None:
                                break
                            # Check if data available (non-blocking)
                            if sys.platform != 'win32':
                                ready, _, _ = select.select([process.stdout], [], [], 0.05)
                                if not ready:
                                    break
                            extra_line = process.stdout.readline()
                            if extra_line:
                                output_lines.append(extra_line)
                                if callback:
                                    callback(extra_line.strip())
                                if "Press Enter to exit" in extra_line:
                                    try:
                                        process.stdin.write("\n")
                                        process.stdin.flush()
                                    except:
                                        pass
                                    break
                            else:
                                break
                    except:
                        pass
                    # Kết thúc ngay sau khi đã phát hiện build completed
                    break
            
            # Đợi process kết thúc (nếu chưa)
            if process.poll() is None:
                # Nếu đã build completed, chỉ đợi tối đa 2 giây để process tự kết thúc
                import time
                start_time = time.time()
                while process.poll() is None and (time.time() - start_time) < 2.0:
                    time.sleep(0.1)
                if process.poll() is None:
                    # Nếu vẫn chưa kết thúc, thử gửi Enter một lần nữa
                    try:
                        process.stdin.write("\n")
                        process.stdin.flush()
                        time.sleep(0.3)
                    except:
                        pass
                    # Đợi thêm một chút
                    if process.poll() is None:
                        time.sleep(0.5)
                    # Nếu vẫn chưa kết thúc, force terminate
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=0.5)
            
            # Combine output
            output = ''.join(output_lines)
            
            # Nếu đã phát hiện build completed, coi như thành công
            if build_completed:
                if callback:
                    build_success_msg = self.translator.get('log.build_driver_success', '✅ Livox driver built successfully!') if self.translator else "✅ Livox driver built successfully!"
                    callback(build_success_msg)
                return True, "Build successful"
            
            # Nếu chưa phát hiện build completed, kiểm tra returncode
            returncode = process.returncode if process.poll() is not None else 0
            
            if returncode == 0:
                if callback:
                    build_success_msg = self.translator.get('log.build_driver_success', '✅ Livox driver built successfully!') if self.translator else "✅ Livox driver built successfully!"
                    callback(build_success_msg)
                return True, "Build successful"
            else:
                error_msg = output
                if callback:
                    callback(f"❌ Build failed: {error_msg[:500]}")
                return False, f"Build failed: {error_msg[:200]}"
                
        except subprocess.TimeoutExpired:
            error_msg = "Build timeout - driver build took too long (over 10 minutes)"
            if callback:
                callback(f"❌ {error_msg}")
            return False, error_msg
        except Exception as e:
            error_msg = f"Build error: {str(e)}"
            if callback:
                callback(f"❌ {error_msg}")
            return False, error_msg