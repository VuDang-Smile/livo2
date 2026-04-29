"""
Configuration module for Bag Mapping Interface
Tập trung tất cả constants, paths, và magic numbers để dễ quản lý và thay đổi
"""
import os
from pathlib import Path


class BagMappingConfig:
    """Configuration class chứa tất cả constants và paths"""
    
    def __init__(self, base_path: Path = None):
        """
        Initialize config với base path
        
        Args:
            base_path: Base path của project (default: parent của gui folder)
        """
        if base_path is None:
            base_path = Path(__file__).parent.parent.parent
        
        # Workspace paths
        self.workspace_path = base_path / "ws"
        self.drive_ws_path = base_path / "dependencies" / "drive_ws"
        
        # Fast-LIVO2 paths
        self.fast_livo2_path = self.workspace_path / "src" / "FAST-LIVO2"
        self.log_path = self.fast_livo2_path / "Log"
        self.config_path = self.fast_livo2_path / "config"
        self.pcd_dir = self.log_path / "PCD"
        self.merged_map_dir = self.log_path / "merged_map"
        self.hba_map_dir = self.log_path / "hba_map"
        self.obj_converted_dir = self.log_path / "obj_converted_pcd"
        
        # File paths
        self.design_config_path = Path(__file__).parent.parent / "design_map_config.json"
        self.qr_detect_json_path = self.log_path / "QR_detect.json"
        self.tunnel_entrance_config_path = Path(__file__).parent.parent / "tunnel_entrance_config.json"
        
        # Default config file
        self.default_config_file = "mid360_equirectangular_stable.yaml"
        self.default_config_path = self.config_path / self.default_config_file
        
        # Scripts paths
        self.scripts_path = base_path / "scripts"
        self.hba_script = self.scripts_path / "normalize_map_hba.py"
        self.compute_dir_script = self.scripts_path / "compute_tunnel_direction.py"
        self.rotate_pcd_script = self.scripts_path / "rotate_pcd_to_positive_x.py"
        self.sc_script = self.scripts_path / "generate_fast_localization_map.py"
        self.downscale_script = self.scripts_path / "downscale_pcd.py"
        self.pcd_to_floorplan_script = self.scripts_path / "pcd_to_floorplan.py"
        
        # Backend configuration
        self.backend_base_url = os.environ.get("LIVO_BACKEND_URL", "http://192.168.2.1:20128")
        
        # QR Scanner configuration
        self.qr_scanner_perspec_size = 600
        self.qr_scanner_max_workers = 4
        self.qr_scan_enabled = True
        self.qr_scan_frame_interval = 10  # Scan QR every N frames
        
        # Bag playback configuration
        self.bag_rate = 0.5  # Default bag playback rate
        
        # Design map comparison configuration
        self.design_file_type = "pcd"  # 'pcd' or 'obj'
        self.comparison_threshold = 90.0  # % match minimum recommendation
        
        # Logging configuration
        self.log_batch_size = 5  # Batch log updates để mượt hơn
        self.progress_animation_speed = 2  # Tốc độ animation progress bar (giá trị mỗi frame)
        
        # UI Configuration
        self.window_geometry = "1150x850"
        self.window_padding = {"padx": 10, "pady": 10}
        
        # Fonts
        self.font_primary = ("Arial", 9, "bold")
        self.font_secondary = ("Arial", 9)
        self.font_small = ("Arial", 7)
        self.font_large = ("Arial", 11, "bold")
        self.font_log = ("Consolas", 10)
        self.font_log_small = ("Consolas", 9)
        
        # Colors
        self.color_primary = "#0066cc"
        self.color_bg_light = "#e8e8e8"
        self.color_bg_active = "#d0d0d0"
        self.color_fg_active = "#0066cc"
        
        # Button sizes
        self.button_start_width = 20
        self.button_stop_width = 10
        
        # Listbox heights
        self.qr_listbox_height = 4
        self.log_panel_height = 8
        
        # Language configuration
        self.default_language = 'en'
        self.supported_languages = {'en': 'English', 'jp': '日本語'}
        
        # Process monitoring
        self.process_health_check_timeout = 10  # seconds
        self.process_terminate_timeout = 10  # seconds
        
        # File patterns
        self.pcd_file_pattern = "*.pcd"
        self.merged_pcd_name = "merged_all.pcd"
        self.hba_pcd_name = "merge_all_hba.pcd"
        self.scans_pos_file = "scans_pos.json"
        
        # Validation thresholds
        self.memory_warning_threshold = 75  # %
        self.memory_error_threshold = 90  # %
        self.disk_warning_threshold = 90  # %
        self.disk_error_threshold = 95  # %
        
        # Default file types for dialogs
        self.pcd_file_types = [("PCD files", "*.pcd"), ("All files", "*.*")]
        self.obj_file_types = [("OBJ files", "*.obj"), ("All files", "*.*")]
        self.bag_file_types = [("ROS2 Bag folders", "*"), ("All files", "*.*")]
        
    def get_pcd_dir(self) -> Path:
        """Get PCD directory path"""
        return self.pcd_dir
    
    def get_merged_map_dir(self) -> Path:
        """Get merged map directory path"""
        return self.merged_map_dir
    
    def get_hba_map_dir(self) -> Path:
        """Get HBA map directory path"""
        return self.hba_map_dir
    
    def get_obj_converted_dir(self) -> Path:
        """Get OBJ converted directory path"""
        return self.obj_converted_dir
    
    def get_qr_detect_json_path(self) -> Path:
        """Get QR detect JSON file path"""
        return self.qr_detect_json_path
    
    def get_design_config_path(self) -> Path:
        """Get design config file path"""
        return self.design_config_path
    
    def get_default_config_path(self) -> Path:
        """Get default config file path"""
        return self.default_config_path
    
    def get_workspace_setup_script(self) -> Path:
        """Get workspace setup script path"""
        return self.workspace_path / "install" / "setup.sh"
    
    def get_drive_ws_setup_script(self) -> Path:
        """Get drive workspace setup script path"""
        return self.drive_ws_path / "install" / "setup.sh"
