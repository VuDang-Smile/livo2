"""
Validation Module
Các function validation cho config, bag file, và system resources
"""
import subprocess
from pathlib import Path
from tkinter import messagebox


class ValidationError(Exception):
    """Exception cho validation errors"""
    pass


def validate_config_file(config_path: Path, logger_callback=None, translator=None) -> bool:
    """
    Validate YAML config file trước khi sử dụng
    
    Args:
        config_path: Path đến config file
        logger_callback: Function để log messages (optional)
        translator: Translator instance (optional)
        
    Returns:
        True nếu valid, False nếu không
    """
    def log(msg):
        if logger_callback:
            logger_callback(msg)
        else:
            print(msg)
    
    def get_translation(key, default):
        if translator:
            return translator.get(key, default)
        return default
    
    try:
        import yaml
    except ImportError:
        # Nếu không có yaml, chỉ kiểm tra file tồn tại
        log(get_translation('log.yaml_not_available', '⚠️ PyYAML not available, skipping config validation'))
        return True
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
        
        if config_data is None:
            error_msg = get_translation('message.config_file_empty', 'Config file is empty or invalid YAML format')
            log(f"❌ {error_msg}")
            if messagebox:
                messagebox.showerror(
                    get_translation('dialog.error', 'Error'),
                    f"{error_msg}: {config_path}"
                )
            return False
        
        # Kiểm tra các tham số quan trọng
        required_keys = ['common', 'lidar', 'imu']
        missing_keys = [key for key in required_keys if key not in config_data]
        
        if missing_keys:
            error_msg = get_translation('message.config_missing_keys', 
                'Config file is missing required sections: {keys}').replace('{keys}', ', '.join(missing_keys))
            log(f"⚠️ {error_msg}")
            # Không block, chỉ cảnh báo
            log(get_translation('log.config_validation_warning', 
                '⚠️ Config validation warning, but continuing...'))
        
        log(get_translation('log.config_validation_success', '✅ Config file validation passed'))
        return True
        
    except yaml.YAMLError as e:
        error_msg = get_translation('message.config_yaml_error', 
            'Config file has invalid YAML syntax: {error}').replace('{error}', str(e))
        log(f"❌ {error_msg}")
        if messagebox:
            messagebox.showerror(
                get_translation('dialog.error', 'Error'),
                f"{error_msg}\n\nFile: {config_path}"
            )
        return False
    except Exception as e:
        error_msg = get_translation('message.config_validation_error', 
            'Error validating config file: {error}').replace('{error}', str(e))
        log(f"⚠️ {error_msg}")
        # Không block, chỉ cảnh báo
        return True


def validate_config_parameters(config_path: Path, logger_callback=None, translator=None) -> bool:
    """
    Validate các tham số quan trọng trong config để tránh crash
    
    Args:
        config_path: Path đến config file
        logger_callback: Function để log messages (optional)
        translator: Translator instance (optional)
        
    Returns:
        True nếu valid, False nếu không
    """
    def log(msg):
        if logger_callback:
            logger_callback(msg)
        else:
            print(msg)
    
    def get_translation(key, default):
        if translator:
            return translator.get(key, default)
        return default
    
    try:
        import yaml
    except ImportError:
        return True
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = yaml.safe_load(f)
        
        if not config_data:
            return True
        
        errors = []
        warnings = []
        
        # Kiểm tra common section
        if 'common' in config_data:
            common = config_data['common']
            # Kiểm tra các tham số có thể gây crash nếu sai
            if 'lidar_topic' in common:
                topic = common['lidar_topic']
                if not topic or not isinstance(topic, str):
                    errors.append(get_translation('log.config_invalid_lidar_topic', 'Invalid lidar_topic in config'))
            
            if 'imu_topic' in common:
                topic = common['imu_topic']
                if not topic or not isinstance(topic, str):
                    errors.append(get_translation('log.config_invalid_imu_topic', 'Invalid imu_topic in config'))
        
        # Kiểm tra lidar section
        if 'lidar' in config_data:
            lidar = config_data['lidar']
            # Kiểm tra các tham số số học
            numeric_params = ['scan_line', 'blind', 'fov_degree', 'min_ring', 'max_ring']
            for param in numeric_params:
                if param in lidar:
                    try:
                        value = float(lidar[param])
                        if value < 0 and param in ['scan_line', 'min_ring', 'max_ring']:
                            warnings.append(get_translation('log.config_negative_value', 
                                f'Warning: {param} has negative value: {value}').replace('{param}', param).replace('{value}', str(value)))
                    except (ValueError, TypeError):
                        errors.append(get_translation('log.config_invalid_numeric', 
                            f'Invalid numeric value for {param}').replace('{param}', param))
        
        # Kiểm tra imu section
        if 'imu' in config_data:
            imu = config_data['imu']
            # Kiểm tra các tham số quan trọng
            if 'acc_n' in imu:
                try:
                    acc_n = float(imu['acc_n'])
                    if acc_n <= 0:
                        warnings.append(get_translation('log.config_invalid_acc_n', 
                            'Warning: acc_n should be positive'))
                except (ValueError, TypeError):
                    pass
        
        if errors:
            error_msg = '\n'.join(errors)
            log(f"❌ {get_translation('log.config_validation_errors', 'Config validation errors')}:")
            for err in errors:
                log(f"   - {err}")
            if messagebox:
                messagebox.showerror(
                    get_translation('dialog.error', 'Error'),
                    f"{get_translation('message.config_has_errors', 'Config file has errors that may cause crashes')}:\n\n{error_msg}"
                )
            return False
        
        if warnings:
            log(f"⚠️ {get_translation('log.config_validation_warnings', 'Config validation warnings')}:")
            for warn in warnings:
                log(f"   - {warn}")
        
        return True
        
    except Exception as e:
        log(f"⚠️ {get_translation('log.config_parameter_validation_error', 'Error validating config parameters: {error}').replace('{error}', str(e))}")
        # Không block, chỉ cảnh báo
        return True


def check_system_resources(logger_callback=None, translator=None, config=None) -> bool:
    """
    Kiểm tra system resources để tránh crash do thiếu tài nguyên
    
    Args:
        logger_callback: Function để log messages (optional)
        translator: Translator instance (optional)
        config: BagMappingConfig instance với thresholds (optional)
        
    Returns:
        True nếu OK, False nếu không đủ resources
    """
    def log(msg):
        if logger_callback:
            logger_callback(msg)
        else:
            print(msg)
    
    def get_translation(key, default):
        if translator:
            return translator.get(key, default)
        return default
    
    try:
        import psutil
    except ImportError:
        # Nếu không có psutil, bỏ qua check
        log(get_translation('log.psutil_not_available', '⚠️ psutil not available, skipping resource check'))
        return True
    
    # Get thresholds từ config hoặc dùng defaults
    memory_warning = config.memory_warning_threshold if config else 75
    memory_error = config.memory_error_threshold if config else 90
    disk_warning = config.disk_warning_threshold if config else 90
    disk_error = config.disk_error_threshold if config else 95
    
    try:
        # Kiểm tra memory
        memory = psutil.virtual_memory()
        memory_percent = memory.percent
        available_gb = memory.available / (1024**3)
        
        if memory_percent > memory_error:
            error_msg = get_translation('message.low_memory', 
                'System memory is very low ({percent}% used, {available:.1f}GB available). This may cause crashes.').replace('{percent}', str(memory_percent)).replace('{available:.1f}', f'{available_gb:.1f}')
            log(f"⚠️ {error_msg}")
            if messagebox:
                if not messagebox.askyesno(
                    get_translation('dialog.warning', 'Warning'),
                    f"{error_msg}\n\n{get_translation('message.continue_anyway', 'Continue anyway?')}"
                ):
                    return False
        elif memory_percent > memory_warning:
            log(get_translation('log.memory_usage_high', 
                f'⚠️ Memory usage is high: {memory_percent:.1f}%').replace('{memory_percent:.1f}', f'{memory_percent:.1f}'))
        
        # Kiểm tra disk space
        disk = psutil.disk_usage('/')
        disk_percent = disk.percent
        free_gb = disk.free / (1024**3)
        
        if disk_percent > disk_error:
            error_msg = get_translation('message.low_disk_space', 
                'Disk space is very low ({percent}% used, {free:.1f}GB free). This may cause crashes.').replace('{percent}', str(disk_percent)).replace('{free:.1f}', f'{free_gb:.1f}')
            log(f"❌ {error_msg}")
            if messagebox:
                messagebox.showerror(
                    get_translation('dialog.error', 'Error'),
                    error_msg
                )
            return False
        elif disk_percent > disk_warning:
            log(get_translation('log.disk_usage_high', 
                f'⚠️ Disk usage is high: {disk_percent:.1f}%').replace('{disk_percent:.1f}', f'{disk_percent:.1f}'))
        
        log(get_translation('log.resource_check_passed', 
            f'✅ Resource check passed (Memory: {memory_percent:.1f}%, Disk: {disk_percent:.1f}%)').replace('{memory_percent:.1f}', f'{memory_percent:.1f}').replace('{disk_percent:.1f}', f'{disk_percent:.1f}'))
        return True
        
    except Exception as e:
        log(f"⚠️ {get_translation('log.resource_check_error', 'Error checking system resources: {error}').replace('{error}', str(e))}")
        # Không block, chỉ cảnh báo
        return True


def validate_bag_file(bag_path: str, workspace_path: Path, logger_callback=None, translator=None) -> bool:
    """
    Kiểm tra bag file có chứa các topics cần thiết
    
    Args:
        bag_path: Đường dẫn đến bag file/folder
        workspace_path: Workspace path
        logger_callback: Function để log messages (optional)
        translator: Translator instance (optional)
        
    Returns:
        True nếu OK, False nếu có vấn đề
    """
    def log(msg):
        if logger_callback:
            logger_callback(msg)
        else:
            print(msg)
    
    def get_translation(key, default):
        if translator:
            return translator.get(key, default)
        return default
    
    if not bag_path:
        return True
    
    try:
        bag_path_obj = Path(bag_path)
        if not bag_path_obj.exists():
            return True
        
        log(get_translation('log.checking_bag_topics', '🔍 Checking bag file for required topics...'))
        
        # Kiểm tra bag info
        ros2_setup = "/opt/ros/jazzy/setup.bash"
        ws_setup = workspace_path / "install" / "setup.sh"
        
        if not ws_setup.exists():
            # Không thể check, bỏ qua
            return True
        
        cmd = f"source {ros2_setup} && source {ws_setup} && ros2 bag info {bag_path}"
        result = subprocess.run(
            cmd,
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True,
            timeout=10
        )
        
        if result.returncode != 0:
            log(get_translation('log.cannot_check_bag', '⚠️ Cannot check bag info, continuing...'))
            return True
        
        bag_output = result.stdout.lower()
        
        # Kiểm tra các topics cần thiết
        required_topic_groups = [
            ['/livox/lidar', '/record_sync/livox/lidar'],
            ['/livox/imu', '/record_sync/livox/imu'],
            ['/image_raw', '/record_sync/image_raw'],
        ]
        missing_topics = []
        
        for topic_group in required_topic_groups:
            group_found = False
            for topic in topic_group:
                topic_variants = [topic, topic[1:], topic.replace('/', '_')]
                if any(variant in bag_output for variant in topic_variants):
                    group_found = True
                    break
            if not group_found:
                missing_topics.append(" | ".join(topic_group))
        
        if missing_topics:
            error_msg = get_translation('message.bag_missing_topics', 
                'Bag file is missing required topics: {topics}\n\nThis may cause the mapping process to crash.').replace('{topics}', ', '.join(missing_topics))
            log(f"❌ {error_msg}")
            if messagebox:
                if not messagebox.askyesno(
                    get_translation('dialog.warning', 'Warning'),
                    f"{error_msg}\n\n{get_translation('message.continue_anyway', 'Continue anyway?')}"
                ):
                    return False
        else:
            log(get_translation('log.bag_topics_ok', '✅ All required topics found in bag file'))
        
        return True
        
    except Exception as e:
        log(f"⚠️ {get_translation('log.bag_validation_error', 'Error validating bag file: {error}').replace('{error}', str(e))}")
        # Không block, chỉ cảnh báo
        return True
