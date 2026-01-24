"""
Utilities Module
Các utility functions cho Bag Mapping Interface
"""
import uuid
import subprocess
import time
from pathlib import Path
from tkinter import messagebox


def get_local_mac_no_colon() -> str:
    """Lấy địa chỉ MAC (không dấu :) từ máy cục bộ."""
    try:
        mac_int = uuid.getnode()
        if mac_int == 0:
            return ""
        return f"{mac_int:012x}"
    except Exception as e:
        print(f"⚠️ Cannot read local MAC: {e}")
        return ""


def check_pcd_files(pcd_dir: Path) -> bool:
    """
    Kiểm tra xem có file PCD trong thư mục không
    
    Args:
        pcd_dir: Đường dẫn đến thư mục PCD
        
    Returns:
        True nếu có file PCD, False nếu không
    """
    if not pcd_dir.exists():
        return False
    
    # Tìm các file PCD (bỏ qua file merged nếu có)
    pcd_files = list(pcd_dir.glob("*.pcd"))
    return len(pcd_files) > 0


def cleanup_ros2_resources(logger_callback=None):
    """
    Dọn dẹp shared memory và các process ROS2 cũ để tránh lỗi FastRTPS SHM
    
    Args:
        logger_callback: Function để log messages (optional)
    """
    def log(msg):
        if logger_callback:
            logger_callback(msg)
        else:
            print(msg)
    
    try:
        log('🧹 Cleaning up old ROS2 resources...')
        
        # 1. Kill các ROS2 processes cũ (fastlivo, ros2 launch, etc.)
        try:
            # Tìm và kill các process liên quan đến fast_livo
            kill_cmd = "pkill -f 'fast_livo|fastlivo' 2>/dev/null || true"
            subprocess.run(kill_cmd, shell=True, timeout=3)
            
            # Kill các ros2 launch processes
            kill_cmd2 = "pkill -f 'ros2 launch.*fast_livo' 2>/dev/null || true"
            subprocess.run(kill_cmd2, shell=True, timeout=3)
            
            time.sleep(0.5)  # Đợi processes dừng
        except Exception as e:
            log(f'⚠️ Error killing processes: {e}')
        
        # 2. Cleanup shared memory segments (FastRTPS SHM)
        try:
            # Tìm và xóa shared memory segments của FastRTPS
            # FastRTPS thường tạo segments trong /dev/shm với prefix "fastrtps_"
            shm_cleanup_cmd = "find /dev/shm -name 'fastrtps_*' -type f -delete 2>/dev/null || true"
            result = subprocess.run(shm_cleanup_cmd, shell=True, capture_output=True, text=True, timeout=5)
            
            # Đếm số file đã xóa (nếu có)
            if result.returncode == 0:
                log('✅ Cleaned up shared memory segments')
        except Exception as e:
            log(f'⚠️ Error cleaning up shared memory: {e}')
        
        # 3. Cleanup semaphores liên quan (nếu có)
        try:
            # Xóa semaphores cũ của FastRTPS
            sem_cleanup_cmd = "ipcrm -a 2>/dev/null || true"
            subprocess.run(sem_cleanup_cmd, shell=True, timeout=3)
        except Exception as e:
            # ipcrm có thể không có trên một số hệ thống, bỏ qua lỗi
            pass
        
        log('✅ Finished cleaning up ROS2 resources')
        time.sleep(0.5)  # Đợi một chút trước khi tiếp tục
        
    except Exception as e:
        log(f'⚠️ Warning during cleanup: {e}')
        # Không dừng lại, tiếp tục chạy mapping


def launch_rviz_cmd(config_path: Path = None) -> str:
    """
    Tạo command để launch RViz2
    
    Args:
        config_path: Đường dẫn đến RViz config file (optional)
        
    Returns:
        Command string để launch RViz2
    """
    if config_path and config_path.exists():
        return f"rviz2 -d {config_path}"
    return "rviz2"


def set_default_config(workspace_path: Path, config_path_holder, logger_callback=None, translator=None):
    """
    Set default config file nếu có
    
    Args:
        workspace_path: Workspace path
        config_path_holder: Object với attribute 'value' để lưu config path (sẽ được set)
        logger_callback: Function để log (optional)
        translator: Translator instance (optional)
    """
    def log_success(msg):
        if logger_callback:
            logger_callback(f"✅ {msg}")
        else:
            print(f"✅ {msg}")
    
    def log_warning(msg):
        if logger_callback:
            logger_callback(f"⚠️ {msg}")
        else:
            print(f"⚠️ {msg}")
    
    default_config = workspace_path / "src" / "FAST-LIVO2" / "config" / "mid360_equirectangular_stable.yaml"
    if default_config.exists():
        config_path_holder.value = str(default_config)
        name = default_config.name
        if translator:
            log_success(translator.get('message.default_config_selected', f'Default config selected: {name}').replace('{name}', name))
        else:
            log_success(f'Default config selected: {name}')
    else:
        name = 'mid360_equirectangular_stable.yaml'
        if translator:
            log_warning(translator.get('message.default_config_not_found', f'Default config not found: {name}').replace('{name}', name))
        else:
            log_warning(f'Default config not found: {name}')
