# Kế hoạch tích hợp Fisheye Camera vào hệ thống LIVO

## Tổng quan

Kế hoạch này mô tả các bước để tích hợp fisheye camera (EquidistantCamera model) vào toàn bộ pipeline từ theta_driver → calibration → FAST-LIVO2.

## Mục tiêu

1. Convert equirectangular 360° từ Theta camera sang fisheye projection
2. Hỗ trợ EquidistantCamera model trong calibration pipeline
3. Tích hợp EquidistantCamera vào FAST-LIVO2
4. Cung cấp GUI để quản lý toàn bộ workflow

---

## 1. theta_driver: Tích hợp converter sang fisheye

### 1.1 Thêm Fisheye Converter

**File mới:** `ws/src/theta_driver/src/fisheye_converter_lib.cpp`
**Header:** `ws/src/theta_driver/include/theta_driver/fisheye_converter_lib.hpp`
**Node:** `ws/src/theta_driver/src/fisheye_converter_node.cpp`

#### Chức năng:
- Convert equirectangular → fisheye (equidistant projection)
- Publish `/image_fisheye` topic
- Publish `/camera_info` với `distortion_model = "equidistant"`

#### Parameters:
```cpp
- projection_type: "fisheye" (mặc định) hoặc "perspective"
- output_width: 640 (mặc định)
- output_height: 480 (mặc định)
- fov_degrees: 180.0 (mặc định cho fisheye 180°)
- k1, k2, k3, k4: fisheye distortion coefficients (mặc định 0.0)
- camera_frame: "camera_link"
- input_topic: "image_raw"
- output_topic: "image_fisheye"
- camera_info_topic: "camera_info"
```

#### Implementation:
- Function `equirectangularToFisheye()`:
  - Sử dụng equidistant projection: `r = f * θ`
  - Map pixel coordinates từ fisheye image → spherical coordinates → equirectangular
  - Formula: `θ = r / f`, `r = sqrt((x-cx)² + (y-cy)²)`

- Function `calculateFisheyeIntrinsics()`:
  - `fx = fy = output_width / (2 * π)` cho 180° FOV
  - `cx = output_width / 2`, `cy = output_height / 2`

- Function `publishCameraInfo()`:
  - `distortion_model = "equidistant"`
  - `d = [k1, k2, k3, k4]` (4 coefficients)

### 1.2 Update CMakeLists.txt

```cmake
# Add fisheye converter library
add_library(fisheye_converter_lib SHARED src/fisheye_converter_lib.cpp)
rclcpp_components_register_nodes(fisheye_converter_lib "theta_driver::FisheyeConverter")
ament_target_dependencies(fisheye_converter_lib
  ${dependencies}
  OpenCV
)

# Add fisheye converter executable
add_executable(fisheye_converter_node src/fisheye_converter_node.cpp)
target_link_libraries(fisheye_converter_node fisheye_converter_lib)
ament_target_dependencies(fisheye_converter_node ${dependencies})
```

### 1.3 Launch file

**File mới:** `ws/src/theta_driver/launch/theta_fisheye.launch.py`

```python
- Parameters cho fisheye conversion
- Launch fisheye_converter_node
- Tích hợp với theta_driver_node
```

---

## 2. gui/theta_tab.py: Thêm thông tin convert sang fisheye

### 2.1 UI Components

#### Thêm vào `create_widgets()`:
- **Frame mới:** "Fisheye Converter" (tương tự Perspective Converter)
  - Button: "Launch Fisheye Converter"
  - Button: "Stop Fisheye Converter"
  - Status label
  - Parameters input:
    - FOV degrees (default: 180.0)
    - Output width/height
    - k1, k2, k3, k4 coefficients

#### Canvas mới:
- **Fisheye view:** Hiển thị `/image_fisheye` topic
- Layout: 3 columns (Equirectangular | Perspective | Fisheye)

### 2.2 Methods

#### `launch_fisheye_converter()`:
```python
- Launch fisheye_converter_node với parameters
- Monitor process output
- Update UI status
```

#### `stop_fisheye_converter()`:
```python
- Stop fisheye_converter_node process
- Update UI
```

#### `on_image_received_fisheye()`:
```python
- Callback cho /image_fisheye topic
- Update fisheye canvas
- Display image info
```

### 2.3 ImageSubscriber update

- Thêm subscription cho `/image_fisheye` topic
- Callback `image_callback_fisheye()`

---

## 3. direct_visual_lidar_calibration: Tích hợp EquidistantCamera

### 3.1 Camera Model Support

#### Kiểm tra hiện tại:
- Package đã hỗ trợ camera models nào?
- Cần thêm EquidistantCamera support

#### Implementation:
- **File:** `ws/src/direct_visual_lidar_calibration/src/camera_loader.cpp` (nếu có)
- Hoặc tích hợp với `rpg_vikit` camera_loader
- Đảm bảo có thể load EquidistantCamera từ camera_info topic

### 3.2 Camera Info Processing

#### Update camera info parser:
- Parse `distortion_model = "equidistant"`
- Extract `k1, k2, k3, k4` từ distortion coefficients
- Tạo EquidistantCamera object với parameters đúng

### 3.3 Calibration Pipeline

#### Preprocessing:
- Detect camera model từ camera_info
- Nếu `distortion_model == "equidistant"`:
  - Sử dụng EquidistantCamera cho projection
  - Apply fisheye distortion model trong feature matching

#### Initial Guess:
- Hỗ trợ EquidistantCamera trong correspondence matching
- Update projection functions để dùng equidistant model

#### Fine Registration:
- Update optimization để hỗ trợ EquidistantCamera
- Jacobian calculation cho equidistant projection

### 3.4 Output Format

#### calib.json:
```json
{
  "camera": {
    "model": "EquidistantCamera",
    "width": 640,
    "height": 480,
    "fx": 101.86,
    "fy": 101.86,
    "cx": 320.0,
    "cy": 240.0,
    "k1": 0.0,
    "k2": 0.0,
    "k3": 0.0,
    "k4": 0.0
  },
  "extrinsic": {
    "Rcl": [...],
    "Pcl": [...]
  }
}
```

---

## 4. gui/calibration_tab.py: Update cho EquidistantCamera

### 4.1 Record Tab Updates

#### Perspective Converter Section:
- **Rename:** "Camera Converter" (bao gồm cả perspective và fisheye)
- **Mode selection:**
  - Radio button: "Perspective" (default)
  - Radio button: "Fisheye"
- **Parameters panel:**
  - Dynamic parameters dựa trên mode:
    - Perspective: FOV, width, height
    - Fisheye: FOV (180°), width, height, k1-k4

#### Topics to record:
- Update default: `/image_fisheye /camera_info /livox/points2` (nếu chọn fisheye)
- Hoặc: `/image_perspective /camera_info /livox/points2` (nếu chọn perspective)

### 4.2 Preprocessing Tab

#### Camera Parameters:
- **Auto-detect từ camera_info:**
  - Detect `distortion_model`
  - Nếu "equidistant" → hiển thị k1-k4
  - Nếu "plumb_bob" → hiển thị d0-d4

#### Options:
- Checkbox: "Use EquidistantCamera model" (auto-detect từ camera_info)

### 4.3 Initial Guess Tab

#### Mode selection:
- Thêm note: "EquidistantCamera được hỗ trợ trong cả manual và automatic mode"

### 4.4 Calibration Tab

#### Options:
- Display camera model được detect
- Warning nếu camera model không match

### 4.5 Export Tab

#### Results display:
- Hiển thị camera model (Pinhole/EquidistantCamera)
- Hiển thị distortion coefficients tương ứng
- Format output phù hợp với FAST-LIVO2

---

## 5. FAST-LIVO2: Config cho EquidistantCamera

### 5.1 Camera Config Files

#### File mới: `ws/src/FAST-LIVO2/config/camera_fisheye.yaml`

```yaml
/**:
  ros__parameters:
    camera:
      model: EquidistantCamera
      width: 640
      height: 480
      scale: 1.0
      fx: 101.86  # width / (2 * π) for 180° FOV
      fy: 101.86
      cx: 320.0
      cy: 240.0
      k1: 0.0
      k2: 0.0
      k3: 0.0
      k4: 0.0
```

#### File mới: `ws/src/FAST-LIVO2/config/avia_fisheye.yaml`
- Tương tự `avia_perspective.yaml` nhưng dùng:
  - `img_topic: "/image_fisheye"`
  - Camera config từ `camera_fisheye.yaml`

#### File mới: `ws/src/FAST-LIVO2/config/mid360_fisheye.yaml`
- Tương tự `mid360_perspective.yaml` nhưng dùng fisheye

### 5.2 Launch Files

#### File mới: `ws/src/FAST-LIVO2/launch/mapping_avia_fisheye.launch.py`
- Sử dụng `camera_fisheye.yaml`
- Sử dụng `avia_fisheye.yaml`

#### File mới: `ws/src/FAST-LIVO2/launch/mapping_mid360_fisheye.launch.py`
- Sử dụng `camera_fisheye.yaml`
- Sử dụng `mid360_fisheye.yaml`

### 5.3 Conversion Script Update

#### `convert_calib_to_fast_livo2.py`:
- Detect camera model từ calib.json
- Nếu `model == "EquidistantCamera"`:
  - Generate config với `model: EquidistantCamera`
  - Map k1-k4 coefficients đúng format
- Output YAML phù hợp với EquidistantCamera

---

## 6. gui/livox_tab.py: Tích hợp config từ calibration

### 6.1 Config Management

#### Thêm vào UI:
- **Frame:** "Camera Configuration"
  - Dropdown: "Camera Model" (Pinhole/EquidistantCamera)
  - Button: "Load from Calibration"
  - Display: Current config file path

### 6.2 Load Calibration Config

#### Method `load_calibration_config()`:
```python
- Browse calib.json file
- Parse camera model
- Convert sang FAST-LIVO2 format
- Save vào config directory
- Update dropdown và display
```

### 6.3 Launch Integration

#### Update `start_mapping()`:
- Detect camera model từ config
- Chọn launch file phù hợp:
  - `mapping_avia_perspective.launch.py` → Pinhole
  - `mapping_avia_fisheye.launch.py` → EquidistantCamera
- Pass config file parameter

### 6.4 Auto-detect

#### Method `detect_camera_model()`:
- Check `/camera_info` topic
- Parse `distortion_model`
- Auto-select camera model và config

---

## 7. Dependencies và Requirements

### 7.1 Code Dependencies
- `rpg_vikit`: Đã có EquidistantCamera support ✓
- OpenCV: `cv::fisheye` module (có sẵn)
- ROS2: sensor_msgs/CameraInfo với equidistant support

### 7.2 Testing Requirements
- Test fisheye conversion với Theta camera
- Test calibration với fisheye images
- Test FAST-LIVO2 với EquidistantCamera
- Validate extrinsic calibration accuracy

---

## 8. Implementation Order

### Phase 1: Core Conversion (Week 1)
1. ✅ Implement `fisheye_converter_lib.cpp`
2. ✅ Test fisheye conversion từ equirectangular
3. ✅ Verify camera_info output format

### Phase 2: GUI Integration (Week 1-2)
4. ✅ Update `theta_tab.py` với fisheye UI
5. ✅ Test fisheye display trong GUI
6. ✅ Update `calibration_tab.py` với fisheye options

### Phase 3: Calibration Pipeline (Week 2)
7. ✅ Update `direct_visual_lidar_calibration` để hỗ trợ EquidistantCamera
8. ✅ Test preprocessing với fisheye images
9. ✅ Test calibration với EquidistantCamera

### Phase 4: FAST-LIVO2 Integration (Week 2-3)
10. ✅ Create fisheye config files
11. ✅ Create fisheye launch files
12. ✅ Update conversion script
13. ✅ Test FAST-LIVO2 với EquidistantCamera

### Phase 5: End-to-End Testing (Week 3)
14. ✅ Full pipeline test: Theta → Fisheye → Calibration → LIVO2
15. ✅ Validate accuracy và performance
16. ✅ Documentation và examples

---

## 9. Testing Checklist

### 9.1 Fisheye Conversion
- [ ] Equirectangular → Fisheye conversion accuracy
- [ ] Camera info format đúng (equidistant)
- [ ] FOV và distortion coefficients đúng
- [ ] Performance (FPS)

### 9.2 Calibration
- [ ] Preprocessing với fisheye images
- [ ] Initial guess với EquidistantCamera
- [ ] Fine registration accuracy
- [ ] Output calib.json format

### 9.3 FAST-LIVO2
- [ ] Load EquidistantCamera config
- [ ] Mapping với fisheye camera
- [ ] Accuracy so với perspective camera
- [ ] Performance comparison

### 9.4 GUI
- [ ] Fisheye display trong theta_tab
- [ ] Calibration workflow với fisheye
- [ ] Config loading trong livox_tab
- [ ] Error handling và user feedback

---

## 10. Documentation

### 10.1 User Guide
- Hướng dẫn sử dụng fisheye converter
- Hướng dẫn calibration với fisheye camera
- Hướng dẫn config FAST-LIVO2 với EquidistantCamera

### 10.2 Developer Notes
- Fisheye projection mathematics
- EquidistantCamera integration details
- API changes và migration guide

---

## 11. Potential Issues và Solutions

### 11.1 Fisheye Conversion
- **Issue:** Distortion ở edges
- **Solution:** Tune k1-k4 coefficients từ calibration

### 11.2 Calibration Accuracy
- **Issue:** Lower accuracy với fisheye vs perspective
- **Solution:** Tăng số lượng correspondences, optimize parameters

### 11.3 Performance
- **Issue:** Slower processing với fisheye
- **Solution:** Optimize projection code, consider GPU acceleration

---

## 12. Success Criteria

1. ✅ Fisheye conversion hoạt động với Theta camera
2. ✅ Calibration pipeline hỗ trợ EquidistantCamera
3. ✅ FAST-LIVO2 mapping hoạt động với fisheye camera
4. ✅ GUI hỗ trợ đầy đủ workflow
5. ✅ Accuracy tương đương hoặc tốt hơn perspective camera
6. ✅ Documentation đầy đủ

---

## Notes

- Tất cả các thay đổi cần backward compatible với perspective camera
- Giữ nguyên các config và launch files hiện tại
- Thêm mới thay vì thay thế để tránh breaking changes

