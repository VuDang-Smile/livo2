# Phương án GUI Design cho Theta Tab với 5 Camera Models

## Tổng quan

Thiết kế GUI mới cho `theta_tab.py` với **Notebook (sub-tabs)** để hiển thị 5 loại camera model khác nhau từ equirectangular conversion.

## Cấu trúc GUI

```
ThetaTab (Main Frame)
├── Control Panel (Top)
│   ├── Launch Theta Driver button
│   ├── Launch All Converters button (NEW)
│   ├── Start All Subscribers button (NEW)
│   └── Stop All button
│
└── Notebook (Sub-tabs)
    ├── Tab 1: Equirectangular (Original)
    │   ├── Launch button (disabled, auto-launch with theta_driver)
    │   ├── Image canvas
    │   └── Status info
    │
    ├── Tab 2: Pinhole
    │   ├── Launch Converter button
    │   ├── Start Subscriber button
    │   ├── Image canvas
    │   └── Status info
    │
    ├── Tab 3: EquidistantCamera (Fisheye)
    │   ├── Launch Converter button
    │   ├── Start Subscriber button
    │   ├── Image canvas
    │   └── Status info
    │
    ├── Tab 4: PolynomialCamera
    │   ├── Launch Converter button
    │   ├── Start Subscriber button
    │   ├── Image canvas
    │   └── Status info
    │
    ├── Tab 5: ATAN
    │   ├── Launch Converter button
    │   ├── Start Subscriber button
    │   ├── Image canvas
    │   └── Status info
    │
    └── Tab 6: Ocam (OmniCamera)
        ├── Launch Converter button
        ├── Start Subscriber button
        ├── Image canvas
        └── Status info
```

## Chi tiết Implementation

### 1. Class Structure

```python
class ThetaTab(ttk.Frame):
    """Main tab container với Notebook"""
    
    def __init__(self, parent):
        # Control panel (top)
        self.create_control_panel()
        
        # Notebook với sub-tabs
        self.notebook = ttk.Notebook(self)
        self.create_camera_tabs()
        
    def create_control_panel(self):
        """Tạo control panel chung ở trên"""
        # Launch Theta Driver
        # Launch All Converters
        # Start All Subscribers
        # Stop All
        
    def create_camera_tabs(self):
        """Tạo các sub-tabs cho camera models"""
        # Tab Equirectangular
        # Tab Pinhole
        # Tab EquidistantCamera
        # Tab PolynomialCamera
        # Tab ATAN
        # Tab Ocam


class CameraModelTab(ttk.Frame):
    """Tab cho một camera model cụ thể"""
    
    def __init__(self, parent, model_name, topic_name, camera_info_topic):
        self.model_name = model_name
        self.topic_name = topic_name
        self.camera_info_topic = camera_info_topic
        
        # UI elements
        self.create_widgets()
        
        # ROS components
        self.converter_process = None
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None
        self.is_running = False
        
    def create_widgets(self):
        """Tạo UI widgets"""
        # Control frame
        # Image canvas
        # Status label
        
    def launch_converter(self):
        """Launch converter node cho camera model này"""
        
    def start_subscriber(self):
        """Start ROS subscriber cho topic này"""
        
    def stop_all(self):
        """Stop converter và subscriber"""
```

### 2. UI Layout Details

#### Control Panel (Top Level)

```
┌─────────────────────────────────────────────────────────────┐
│ [Launch Theta Driver] [Launch All Converters] [Start All]   │
│ [Stop All]                    Status: Ready                 │
└─────────────────────────────────────────────────────────────┘
```

#### Individual Camera Tab

```
┌─────────────────────────────────────────────────────────────┐
│ [Launch Converter] [Start Subscriber] [Stop]              │
│                                                             │
│ ┌───────────────────────────────────────────────────────┐ │
│ │                                                       │ │
│ │              Image Canvas                            │ │
│ │                                                       │ │
│ └───────────────────────────────────────────────────────┘ │
│                                                             │
│ Status: Pinhole | 640x480 | Scale: 0.5 | Frames: 1234     │
└─────────────────────────────────────────────────────────────┘
```

### 3. Features

#### 3.1 Global Controls

- **Launch Theta Driver**: Launch theta_driver_node (chỉ một lần)
- **Launch All Converters**: Launch tất cả 5 converter nodes cùng lúc
- **Start All Subscribers**: Start tất cả ROS subscribers cho các tabs
- **Stop All**: Stop tất cả processes và subscribers

#### 3.2 Per-Tab Controls

- **Launch Converter**: Launch converter node cho camera model này
- **Start Subscriber**: Start ROS subscriber cho topic tương ứng
- **Stop**: Stop converter và subscriber cho tab này

#### 3.3 Status Display

Mỗi tab hiển thị:
- Camera model name
- Image resolution (width x height)
- Display scale
- Frame count
- Connection status

### 4. Topic Mapping

| Camera Model | Image Topic | Camera Info Topic |
|--------------|-------------|-------------------|
| Equirectangular | `/image_raw` | N/A |
| Pinhole | `/image_pinhole` | `/camera_info_pinhole` |
| EquidistantCamera | `/image_equidistant` | `/camera_info_equidistant` |
| PolynomialCamera | `/image_polynomial` | `/camera_info_polynomial` |
| ATAN | `/image_atan` | `/camera_info_atan` |
| Ocam | `/image_ocam` | `/camera_info_ocam` |

### 5. State Management

#### 5.1 Tab States

- **Disabled**: Chưa launch theta_driver
- **Ready**: Theta driver đã chạy, có thể launch converter
- **Running**: Converter đã chạy, có thể start subscriber
- **Active**: Subscriber đang chạy, đang nhận images
- **Error**: Có lỗi xảy ra

#### 5.2 Button States

```python
# Khi theta_driver chưa chạy
launch_theta_btn: ENABLED
launch_all_btn: DISABLED
start_all_btn: DISABLED

# Khi theta_driver đã chạy
launch_theta_btn: DISABLED
launch_all_btn: ENABLED
start_all_btn: DISABLED

# Khi converters đã chạy
launch_all_btn: DISABLED
start_all_btn: ENABLED

# Khi subscribers đang chạy
start_all_btn: DISABLED
stop_all_btn: ENABLED
```

### 6. Image Display

#### 6.1 Canvas Sizing

- Mỗi tab có canvas riêng
- Canvas tự động resize theo window size
- Image được scale để fit canvas (giữ aspect ratio)
- Background: black khi chưa có image

#### 6.2 Image Update

- Update image khi nhận được frame mới từ ROS
- Throttle update rate để tránh lag (max 30 FPS)
- Display resolution info và scale factor

### 7. ROS Integration

#### 7.1 Subscriber per Tab

Mỗi tab có ROS subscriber riêng:
- Subscribe topic tương ứng
- Callback update image display
- Thread-safe với main GUI thread

#### 7.2 Process Management

- Track converter processes
- Monitor process health
- Auto-restart on failure (optional)
- Clean shutdown on stop

### 8. Error Handling

#### 8.1 Launch Errors

- Check workspace setup.sh exists
- Check node executable exists
- Display error message in status label
- Show error dialog for critical errors

#### 8.2 ROS Errors

- Handle topic not found
- Handle subscriber errors
- Handle image conversion errors
- Log errors to console

### 9. Performance Considerations

#### 9.1 Image Processing

- Resize images trước khi display (không display full resolution)
- Use PIL ImageTk for efficient display
- Cache resized images

#### 9.2 Threading

- ROS spin trong separate thread
- GUI update trong main thread
- Use `after()` method để schedule GUI updates

#### 9.3 Memory Management

- Clear old images khi nhận image mới
- Limit image cache size
- Garbage collect unused images

### 10. Code Structure

```python
# gui/theta_tab.py

class ThetaTab(ttk.Frame):
    """Main tab với Notebook"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # State
        self.theta_driver_process = None
        self.converter_processes = {}  # {model_name: process}
        self.camera_tabs = {}  # {model_name: CameraModelTab}
        
        # UI
        self.create_control_panel()
        self.create_notebook()
        
    def create_control_panel(self):
        """Control panel ở top"""
        ...
        
    def create_notebook(self):
        """Tạo notebook với sub-tabs"""
        self.notebook = ttk.Notebook(self)
        
        # Tab Equirectangular (original)
        self.tab_equirect = EquirectangularTab(
            self.notebook, "Equirectangular", "image_raw"
        )
        self.notebook.add(self.tab_equirect, text="Equirectangular")
        
        # Tab Pinhole
        self.tab_pinhole = CameraModelTab(
            self.notebook, "Pinhole", "image_pinhole", 
            "camera_info_pinhole", "pinhole_converter_node"
        )
        self.notebook.add(self.tab_pinhole, text="Pinhole")
        
        # Tab EquidistantCamera
        self.tab_equidistant = CameraModelTab(
            self.notebook, "EquidistantCamera", "image_equidistant",
            "camera_info_equidistant", "equidistant_converter_node"
        )
        self.notebook.add(self.tab_equidistant, text="Equidistant")
        
        # Tab PolynomialCamera
        self.tab_polynomial = CameraModelTab(
            self.notebook, "PolynomialCamera", "image_polynomial",
            "camera_info_polynomial", "polynomial_converter_node"
        )
        self.notebook.add(self.tab_polynomial, text="Polynomial")
        
        # Tab ATAN
        self.tab_atan = CameraModelTab(
            self.notebook, "ATAN", "image_atan",
            "camera_info_atan", "atan_converter_node"
        )
        self.notebook.add(self.tab_atan, text="ATAN")
        
        # Tab Ocam
        self.tab_ocam = CameraModelTab(
            self.notebook, "Ocam", "image_ocam",
            "camera_info_ocam", "ocam_converter_node"
        )
        self.notebook.add(self.tab_ocam, text="Ocam")
        
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
    def launch_all_converters(self):
        """Launch tất cả converters"""
        for tab in self.camera_tabs.values():
            tab.launch_converter()
            
    def start_all_subscribers(self):
        """Start tất cả subscribers"""
        for tab in self.camera_tabs.values():
            tab.start_subscriber()
            
    def stop_all(self):
        """Stop tất cả"""
        # Stop theta driver
        # Stop all converters
        # Stop all subscribers


class CameraModelTab(ttk.Frame):
    """Tab cho một camera model"""
    
    def __init__(self, parent, model_name, topic_name, 
                 camera_info_topic, node_name):
        super().__init__(parent)
        self.model_name = model_name
        self.topic_name = topic_name
        self.camera_info_topic = camera_info_topic
        self.node_name = node_name
        
        self.create_widgets()
        
    def create_widgets(self):
        """Tạo UI widgets"""
        # Control frame
        control_frame = ttk.Frame(self, padding="5")
        control_frame.pack(fill=tk.X)
        
        self.launch_btn = ttk.Button(
            control_frame,
            text=f"Launch {self.model_name} Converter",
            command=self.launch_converter
        )
        self.launch_btn.pack(side=tk.LEFT, padx=5)
        
        self.start_btn = ttk.Button(
            control_frame,
            text="Start Subscriber",
            command=self.start_subscriber,
            state=tk.DISABLED
        )
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(
            control_frame,
            text="Stop",
            command=self.stop_all,
            state=tk.DISABLED
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Image canvas
        self.canvas = tk.Canvas(
            self,
            bg="black",
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Status label
        self.status_label = ttk.Label(
            self,
            text=f"{self.model_name}: Chưa có ảnh",
            font=("Arial", 9)
        )
        self.status_label.pack(fill=tk.X, padx=5, pady=5)
        
    def launch_converter(self):
        """Launch converter node"""
        ...
        
    def start_subscriber(self):
        """Start ROS subscriber"""
        ...
        
    def stop_all(self):
        """Stop converter và subscriber"""
        ...


class EquirectangularTab(ttk.Frame):
    """Tab cho equirectangular (original)"""
    # Tương tự CameraModelTab nhưng không có converter
    # Chỉ subscribe /image_raw từ theta_driver
```

### 11. Visual Design

#### 11.1 Colors

- Background: System default
- Canvas background: Black
- Status text: 
  - Green: Active/Running
  - Orange: Ready
  - Red: Error/Stopped

#### 11.2 Layout

- Control panel: Fixed height, top
- Notebook: Fill remaining space
- Each tab: Fill notebook space
- Canvas: Fill tab space (with padding)

### 12. Testing Checklist

- [ ] Launch theta_driver
- [ ] Launch individual converters
- [ ] Launch all converters
- [ ] Start individual subscribers
- [ ] Start all subscribers
- [ ] Display images in all tabs
- [ ] Stop individual tabs
- [ ] Stop all
- [ ] Error handling
- [ ] Window resize
- [ ] Multiple tabs simultaneously

---

## Summary

GUI mới sẽ có:
- **1 main control panel** ở top
- **6 sub-tabs** trong Notebook:
  1. Equirectangular (original)
  2. Pinhole
  3. EquidistantCamera
  4. PolynomialCamera
  5. ATAN
  6. Ocam
- **Mỗi tab** có controls riêng và image display riêng
- **Global controls** để launch/stop tất cả cùng lúc
- **Thread-safe** ROS integration
- **Error handling** và status display

