# Kế hoạch Convert Equirectangular sang 5 loại Camera Model

## Tổng quan

Kế hoạch này mô tả việc mở rộng `theta_driver` để convert ảnh equirectangular 360° từ Theta camera sang **5 loại camera model** được hỗ trợ bởi `rpg_vikit` và `FAST-LIVO2`:

1. **Ocam** (OmniCamera) - Camera toàn cảnh
2. **Pinhole** - Camera pinhole tiêu chuẩn
3. **EquidistantCamera** - Camera fisheye equidistant
4. **PolynomialCamera** - Camera đa thức
5. **ATAN** - Camera ATAN

## Mục tiêu

1. Tạo universal camera converter có thể convert sang tất cả 5 loại camera model
2. Mỗi loại camera model có topic riêng: `/image_ocam`, `/image_pinhole`, `/image_equidistant`, `/image_polynomial`, `/image_atan`
3. Mỗi loại camera model có camera_info riêng với distortion model tương ứng
4. Cập nhật GUI `theta_tab.py` với sub-tabs để hiển thị từng loại camera model
5. Tạo launch files cho từng loại camera converter

---

## 1. Kiến trúc Universal Camera Converter

### 1.1 Cấu trúc File

```
ws/src/theta_driver/
├── src/
│   ├── universal_camera_converter_lib.cpp    # Library chính
│   └── universal_camera_converter_node.cpp   # Node executable
├── include/theta_driver/
│   └── universal_camera_converter_lib.hpp     # Header file
└── launch/
    ├── ocam_converter.launch.py
    ├── pinhole_converter.launch.py
    ├── equidistant_converter.launch.py
    ├── polynomial_converter.launch.py
    └── atan_converter.launch.py
```

### 1.2 Class Design

```cpp
class UniversalCameraConverter : public rclcpp::Node {
public:
    enum CameraModel {
        OCAM,
        PINHOLE,
        EQUIDISTANT,
        POLYNOMIAL,
        ATAN
    };
    
private:
    // Convert equirectangular to specific camera model
    cv::Mat equirectangularToCameraModel(
        const cv::Mat& equirect,
        CameraModel model,
        int output_width,
        int output_height,
        const CameraParams& params);
    
    // Camera model specific conversion functions
    cv::Mat equirectangularToOcam(...);
    cv::Mat equirectangularToPinhole(...);
    cv::Mat equirectangularToEquidistant(...);
    cv::Mat equirectangularToPolynomial(...);
    cv::Mat equirectangularToATAN(...);
    
    // Calculate camera intrinsics for each model
    void calculateCameraIntrinsics(CameraModel model);
    
    // Publish camera info with correct distortion model
    void publishCameraInfo(CameraModel model, ...);
};
```

---

## 2. Chi tiết từng Camera Model

### 2.1 Ocam (OmniCamera)

**Đặc điểm:**
- Camera toàn cảnh (omnidirectional)
- Sử dụng calibration file từ OCamCalib
- Projection: `r = f * tan(θ)`

**Parameters:**
```yaml
camera_model: "Ocam"
calib_file: "/path/to/ocam_calib.txt"
output_width: 640
output_height: 480
camera_frame: "camera_link"
input_topic: "image_raw"
output_topic: "image_ocam"
camera_info_topic: "camera_info_ocam"
```

**Implementation:**
- Load calibration file từ OCamCalib format
- Convert equirectangular → spherical → Ocam projection
- Publish với `distortion_model = "omni"`

### 2.2 Pinhole

**Đặc điểm:**
- Camera pinhole tiêu chuẩn với distortion (plumb_bob)
- Projection: `x = fx * X/Z + cx`, `y = fy * Y/Z + cy`

**Parameters:**
```yaml
camera_model: "Pinhole"
output_width: 640
output_height: 480
fov_degrees: 75.0
fx: 417.0  # Optional, calculated from FOV if not provided
fy: 417.0
cx: 320.0
cy: 240.0
d0: 0.0  # k1
d1: 0.0  # k2
d2: 0.0  # p1
d3: 0.0  # p2
camera_frame: "camera_link"
input_topic: "image_raw"
output_topic: "image_pinhole"
camera_info_topic: "camera_info_pinhole"
```

**Implementation:**
- Sử dụng code hiện tại từ `perspective_converter_lib.cpp`
- Publish với `distortion_model = "plumb_bob"`

### 2.3 EquidistantCamera (Fisheye)

**Đặc điểm:**
- Camera fisheye với equidistant projection
- Projection: `r = f * θ` (r là khoảng cách từ center, θ là góc)
- Distortion: `k1, k2, k3, k4`

**Parameters:**
```yaml
camera_model: "EquidistantCamera"
output_width: 640
output_height: 480
fov_degrees: 180.0  # Full fisheye FOV
fx: 200.0  # Optional, calculated from FOV if not provided
fy: 200.0
cx: 320.0
cy: 240.0
k1: 0.0
k2: 0.0
k3: 0.0
k4: 0.0
camera_frame: "camera_link"
input_topic: "image_raw"
output_topic: "image_equidistant"
camera_info_topic: "camera_info_equidistant"
```

**Implementation:**
- Convert equirectangular → spherical → equidistant projection
- Formula: `θ = atan2(sqrt(x²+y²), 1)`, `r = fx * θ`
- Map pixel coordinates từ fisheye image → spherical → equirectangular
- Publish với `distortion_model = "equidistant"`

### 2.4 PolynomialCamera

**Đặc điểm:**
- Camera với polynomial distortion model
- Projection: `r = f * (θ + k2*θ³ + k3*θ⁵ + k4*θ⁷ + k5*θ⁹ + k6*θ¹¹ + k7*θ¹³)`
- Distortion: `k2, k3, k4, k5, k6, k7`

**Parameters:**
```yaml
camera_model: "PolynomialCamera"
output_width: 640
output_height: 480
fx: 200.0
fy: 200.0
cx: 320.0
cy: 240.0
skew: 0.0
k2: 0.0
k3: 0.0
k4: 0.0
k5: 0.0
k6: 0.0
k7: 0.0
camera_frame: "camera_link"
input_topic: "image_raw"
output_topic: "image_polynomial"
camera_info_topic: "camera_info_polynomial"
```

**Implementation:**
- Convert equirectangular → spherical → polynomial projection
- Sử dụng polynomial model để tính r từ θ
- Publish với `distortion_model = "polynomial"`

### 2.5 ATAN

**Đặc điểm:**
- Camera với ATAN projection model
- Projection: `r = f * atan(θ)`
- Distortion: `d0`

**Parameters:**
```yaml
camera_model: "ATAN"
output_width: 640
output_height: 480
fx: 200.0
fy: 200.0
cx: 320.0
cy: 240.0
d0: 0.0
camera_frame: "camera_link"
input_topic: "image_raw"
output_topic: "image_atan"
camera_info_topic: "camera_info_atan"
```

**Implementation:**
- Convert equirectangular → spherical → ATAN projection
- Formula: `r = fx * atan(θ)`
- Publish với `distortion_model = "atan"`

---

## 3. Conversion Algorithm

### 3.1 Tổng quan

Tất cả các camera model đều sử dụng quy trình chung:

1. **Input**: Equirectangular image (360°)
2. **For each pixel** trong output image:
   - Convert pixel coordinates → camera model coordinates (x, y)
   - Convert camera model coordinates → spherical coordinates (θ, φ)
   - Convert spherical coordinates → equirectangular pixel coordinates
   - Sample pixel từ equirectangular image
3. **Output**: Camera model image

### 3.2 Chi tiết từng Model

#### Pinhole
```
Pixel (i, j) → Normalized (x, y) → 3D direction → Spherical (θ, φ) → Equirectangular (px, py)
```

#### EquidistantCamera
```
Pixel (i, j) → Polar (r, φ) → Angle θ = r/fx → Spherical (θ, φ) → Equirectangular (px, py)
```

#### PolynomialCamera
```
Pixel (i, j) → Polar (r, φ) → Solve θ from r = f*(θ + k2*θ³ + ...) → Spherical (θ, φ) → Equirectangular (px, py)
```

#### ATAN
```
Pixel (i, j) → Polar (r, φ) → Angle θ = tan(r/fx) → Spherical (θ, φ) → Equirectangular (px, py)
```

#### Ocam
```
Pixel (i, j) → Ocam projection → Spherical (θ, φ) → Equirectangular (px, py)
```

---

## 4. GUI Integration (theta_tab.py)

### 4.1 Cấu trúc Sub-tabs

```python
class ThetaTab(ttk.Frame):
    def __init__(self, parent):
        # Tạo Notebook với sub-tabs
        self.notebook = ttk.Notebook(self)
        
        # Tab cho mỗi camera model
        self.tab_ocam = CameraModelTab(self.notebook, "Ocam", "image_ocam")
        self.tab_pinhole = CameraModelTab(self.notebook, "Pinhole", "image_pinhole")
        self.tab_equidistant = CameraModelTab(self.notebook, "Equidistant", "image_equidistant")
        self.tab_polynomial = CameraModelTab(self.notebook, "Polynomial", "image_polynomial")
        self.tab_atan = CameraModelTab(self.notebook, "ATAN", "image_atan")
        
        # Add tabs to notebook
        self.notebook.add(self.tab_ocam, text="Ocam")
        self.notebook.add(self.tab_pinhole, text="Pinhole")
        self.notebook.add(self.tab_equidistant, text="Equidistant")
        self.notebook.add(self.tab_polynomial, text="Polynomial")
        self.notebook.add(self.tab_atan, text="ATAN")
```

### 4.2 CameraModelTab Class

```python
class CameraModelTab(ttk.Frame):
    """Tab cho một camera model cụ thể"""
    def __init__(self, parent, model_name, topic_name):
        # Launch button
        # Image display canvas
        # ROS subscriber
        # Status label
```

### 4.3 Features

- Mỗi tab có nút Launch riêng cho converter tương ứng
- Mỗi tab có image display riêng
- Mỗi tab có ROS subscriber riêng cho topic tương ứng
- Status hiển thị frame count và resolution cho từng model

---

## 5. Launch Files

### 5.1 Universal Launch File

```python
# universal_camera_converter.launch.py
def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model', default='Pinhole')
    
    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='Pinhole'),
        Node(
            package='theta_driver',
            executable='universal_camera_converter_node',
            name='universal_camera_converter',
            parameters=[{
                'camera_model': camera_model,
                # ... other parameters
            }]
        )
    ])
```

### 5.2 Individual Launch Files

Mỗi camera model có launch file riêng với parameters mặc định tương ứng.

---

## 6. Implementation Steps

### Phase 1: Core Library
1. ✅ Tạo `universal_camera_converter_lib.hpp`
2. ✅ Implement base class và conversion functions
3. ✅ Implement từng camera model conversion
4. ✅ Test từng conversion function

### Phase 2: ROS Integration
1. ✅ Tạo `universal_camera_converter_node.cpp`
2. ✅ Implement ROS publishers/subscribers
3. ✅ Implement camera_info publishing
4. ✅ Update CMakeLists.txt

### Phase 3: Launch Files
1. ✅ Tạo universal launch file
2. ✅ Tạo individual launch files cho từng model
3. ✅ Test launch files

### Phase 4: GUI Integration
1. ✅ Refactor `theta_tab.py` với Notebook
2. ✅ Tạo `CameraModelTab` class
3. ✅ Implement sub-tabs cho 5 camera models
4. ✅ Test GUI với từng camera model

### Phase 5: Documentation
1. ✅ Update README.md
2. ✅ Tạo usage examples
3. ✅ Document parameters cho từng model

---

## 7. Testing Plan

### 7.1 Unit Tests
- Test conversion functions với sample equirectangular images
- Verify output image dimensions và quality
- Verify camera_info parameters

### 7.2 Integration Tests
- Test với theta_driver node
- Test với FAST-LIVO2
- Test với rpg_vikit camera loader

### 7.3 GUI Tests
- Test launch buttons cho từng model
- Test image display cho từng tab
- Test multiple tabs simultaneously

---

## 8. Dependencies

- OpenCV (đã có)
- rclcpp (đã có)
- sensor_msgs (đã có)
- rpg_vikit (để verify camera models)

---

## 9. Notes

- Tất cả converters chạy độc lập, có thể launch nhiều converters cùng lúc
- Mỗi converter có topic riêng để tránh conflict
- GUI có thể hiển thị nhiều tabs cùng lúc để so sánh các camera models
- Parameters có thể được override qua launch files hoặc parameter files

---

## 10. Future Enhancements

- Real-time parameter adjustment qua dynamic reconfigure
- Save/load camera parameters
- Visual comparison tool giữa các camera models
- Calibration tool integration

