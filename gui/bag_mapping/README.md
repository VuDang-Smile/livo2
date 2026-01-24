# Bag Mapping Module Package

Package này chứa các module được tách ra từ `main_bag_mapping.py` để dễ quản lý và bảo trì.

## Cấu trúc

- `config.py` - Configuration class chứa tất cả constants, paths, và magic numbers
- `qr_code.py` - QR code scanning functionality (QRScanner, QRImageSubscriberNode)
- `logging.py` - Logging management (LoggingManager)
- `progress.py` - Progress bar management (ProgressManager)
- `utils.py` - Utility functions (MAC address, file checks, cleanup, etc.)
- `validation.py` - Validation functions (config, bag file, system resources)
- `language.py` - Language management (LanguageManager)

## Các module còn lại cần tách (trong tương lai)

- `ui.py` - UI components setup (setup_control_card, setup_preview_rviz_section, update_ui_texts)
- `core.py` - Core mapping logic (start_mapping, start_bag_playback, monitor processes, stop_process)
- `upload.py` - Upload functionality (start_upload, zip_map_folders, upload_to_backend)
- `design.py` - Design map comparison (browse_design_file, run_design_map_comparison)
- `pcd.py` - PCD processing (merge_pcd_files, HBA, SC, floorplan generation)

## Sử dụng

```python
from bag_mapping.config import BagMappingConfig
from bag_mapping.qr_code import QRScanner
from bag_mapping.logging import LoggingManager
from bag_mapping.progress import ProgressManager
from bag_mapping.utils import get_local_mac_no_colon, cleanup_ros2_resources
from bag_mapping.validation import validate_config_file, check_system_resources
from bag_mapping.language import LanguageManager

# Initialize config
config = BagMappingConfig()

# Use modules
qr_scanner = QRScanner(perspec_size=config.qr_scanner_perspec_size, max_workers=config.qr_scanner_max_workers)
```
