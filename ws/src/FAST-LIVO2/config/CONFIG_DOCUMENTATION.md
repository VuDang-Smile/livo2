# FAST-LIVO2 Configuration Documentation

Tài liệu đầy đủ về tất cả các tham số cấu hình của hệ thống FAST-LIVO2.

## Mục lục

1. [Common Parameters](#1-common-parameters)
2. [Extrinsic Calibration](#2-extrinsic-calibration)
3. [Time Offset](#3-time-offset)
4. [Preprocessing](#4-preprocessing)
5. [VIO (Visual-Inertial Odometry)](#5-vio-visual-inertial-odometry)
6. [IMU](#6-imu)
7. [LIO (LiDAR-Inertial Odometry)](#7-lio-lidar-inertial-odometry)
8. [Local Map](#8-local-map)
9. [Memory Management](#9-memory-management)
10. [UAV](#10-uav)
11. [Publish](#11-publish)
12. [EVO](#12-evo)
13. [PCD Save](#13-pcd-save)
14. [Camera](#14-camera)
15. [Debug](#15-debug)

---

## 1. Common Parameters

Các tham số chung cho hệ thống.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `common.lid_topic` | string | `/livox/lidar` | `/livox/lidar` | ROS topic name | Topic nhận dữ liệu LiDAR (CustomMsg từ Livox driver) |
| `common.imu_topic` | string | `/livox/imu` | `/livox/imu` | ROS topic name | Topic nhận dữ liệu IMU từ Livox driver |
| `common.img_topic` | string | `/left_camera/image` | `/image_raw` | ROS topic name | Topic nhận dữ liệu hình ảnh từ camera |
| `common.img_en` | int | 1 | 1 | 0/1 | Bật/tắt xử lý hình ảnh (1=bật, 0=tắt) |
| `common.lidar_en` | int | 1 | 1 | 0/1 | Bật/tắt xử lý LiDAR (1=bật, 0=tắt) |
| `common.ros_driver_bug_fix` | bool | false | true | true/false | Sửa lỗi driver ROS (nên bật khi dùng Livox driver) |

### Giải thích chi tiết:

- **img_en/lidar_en**: Điều khiển việc sử dụng sensor. Nếu tắt, hệ thống sẽ chỉ dùng sensor còn lại.
- **ros_driver_bug_fix**: Sửa lỗi đồng bộ thời gian trong Livox ROS driver. Nên bật khi dùng Livox.

---

## 2. Extrinsic Calibration

Calibration ngoại tại giữa các sensor.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `extrin_calib.extrinsic_T` | vector[3] | `[]` | `[0.04165, 0.02326, -0.0284]` | meters | Vector dịch chuyển từ IMU sang LiDAR [x, y, z] (m) |
| `extrin_calib.extrinsic_R` | vector[9] | `[]` | Identity matrix | row-major | Ma trận quay từ IMU sang LiDAR (9 giá trị, row-major) |
| `extrin_calib.Rcl` | vector[9] | `[]` | Calibrated | row-major | Ma trận quay từ LiDAR sang Camera (9 giá trị, row-major) |
| `extrin_calib.Pcl` | vector[3] | `[]` | Calibrated | meters | Vector dịch chuyển từ LiDAR sang Camera [x, y, z] (m) |

### Giải thích chi tiết:

- **extrinsic_T/R**: Transformation từ IMU frame sang LiDAR frame. Thường được calibrate sẵn từ nhà sản xuất.
- **Rcl/Pcl**: Transformation từ LiDAR frame sang Camera frame. **QUAN TRỌNG**: Phải được calibrate lại khi thay đổi camera hoặc vị trí lắp đặt.
- **Calibration**: Sử dụng `direct_visual_lidar_calibration` package để calibrate.

### Hiệu quả:

- Sai số calibration ảnh hưởng trực tiếp đến độ chính xác của odometry.
- Rcl/Pcl sai sẽ làm VIO không hoạt động đúng, dẫn đến drift lớn.

---

## 3. Time Offset

Đồng bộ thời gian giữa các sensor.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `time_offset.imu_time_offset` | double | 0.0 | 0.0 | seconds | Offset thời gian IMU (IMU đi trước: âm, đi sau: dương) |
| `time_offset.img_time_offset` | double | 0.0 | 0.0 | seconds | Offset thời gian Image |
| `time_offset.exposure_time_init` | double | 0.0 | 0.0 | seconds | Thời gian phơi sáng ban đầu (cho exposure estimation) |

### Giải thích chi tiết:

- **imu_time_offset**: Nếu IMU đi trước LiDAR, dùng giá trị âm. Nếu đi sau, dùng giá trị dương.
- **img_time_offset**: Offset thời gian cho camera (thường 0 nếu camera đã sync tốt).
- **exposure_time_init**: Giá trị khởi tạo cho thuật toán ước lượng exposure time.

### Hiệu quả:

- Offset sai sẽ gây drift trong odometry.
- Nên calibrate bằng cách so sánh timestamp giữa các sensor.

---

## 4. Preprocessing

Xử lý trước dữ liệu LiDAR.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `preprocess.point_filter_num` | int | 3 | 1 | 1-10 | Số điểm bỏ qua khi downsampling (1=giữ tất cả) |
| `preprocess.filter_size_surf` | double | 0.5 | 0.12 | 0.05-1.0 | Kích thước filter cho surface points (m) |
| `preprocess.lidar_type` | int | AVIA | 8 | 0-10 | Loại LiDAR (8=MID360, 0=AVIA, ...) |
| `preprocess.scan_line` | int | 6 | 6 | 1-128 | Số scan lines của LiDAR (MID360=6) |
| `preprocess.scan_rate` | int | 10 | - | 1-100 | Tần số quét của LiDAR (Hz) |
| `preprocess.blind` | double | 0.01 | 0.8 | 0.0-2.0 | Khoảng cách blind zone (m) - bỏ điểm gần |
| `preprocess.feature_extract_enabled` | bool | false | - | true/false | Bật/tắt feature extraction |

### Giải thích chi tiết:

#### `preprocess.point_filter_num` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Bỏ qua N điểm liên tiếp, chỉ giữ điểm thứ (N+1). Đây là downsampling đơn giản nhất.

**Giá trị và tác dụng**:
- **1**: Giữ **100% điểm** → Độ chính xác cao nhất, nhưng:
  - MID360: ~100,000 điểm/scan → xử lý chậm (~2-3x)
  - Memory: ~3x cao hơn
  - CPU: ~2.5x cao hơn
- **2**: Giữ **50% điểm** → Cân bằng tốt:
  - MID360: ~50,000 điểm/scan
  - Độ chính xác: ~95% so với point_filter_num=1
  - Tốc độ: ~1.8x nhanh hơn
- **3** (default): Giữ **33% điểm** → Tốc độ ưu tiên:
  - MID360: ~33,000 điểm/scan
  - Độ chính xác: ~90% so với point_filter_num=1
  - Tốc độ: ~2.5x nhanh hơn
- **5+**: Giữ <20% điểm → Mất nhiều chi tiết, không khuyến nghị

**Khuyến nghị**: 
- Mapping chất lượng cao: **1**
- Real-time mapping: **2-3**
- Tốc độ tối đa: **3**

#### `preprocess.filter_size_surf` ⭐ **RẤT QUAN TRỌNG**

**Cơ chế hoạt động**: Voxel grid filter - chia không gian thành các voxel có kích thước này, mỗi voxel chỉ giữ 1 điểm (thường là trung bình).

**Giá trị và tác dụng**:
- **0.05-0.08m**: Rất chi tiết
  - Phù hợp: Mapping indoor, chi tiết cao
  - Điểm density: ~800-1200 điểm/m²
  - Memory: Rất cao (~5-8GB cho map 100m)
  - Tốc độ: Chậm (~0.5x real-time)
- **0.10-0.12m** (current): Cân bằng tốt ⭐
  - Phù hợp: Hầu hết trường hợp
  - Điểm density: ~400-600 điểm/m²
  - Memory: Vừa phải (~2-3GB cho map 100m)
  - Tốc độ: Real-time được
- **0.15-0.20m**: Chi tiết vừa
  - Phù hợp: Outdoor, tốc độ cao
  - Điểm density: ~200-300 điểm/m²
  - Memory: Thấp (~1GB cho map 100m)
  - Tốc độ: Nhanh (~1.5x real-time)
- **0.30-0.50m**: Ít chi tiết
  - Phù hợp: Large-scale mapping
  - Điểm density: ~50-100 điểm/m²
  - Memory: Rất thấp
  - Tốc độ: Rất nhanh nhưng mất nhiều chi tiết

**Công thức ước tính**: 
- Số điểm ≈ (Diện tích) / (filter_size_surf)²
- Memory (GB) ≈ (Số điểm) × 32 bytes / 1e9

**Khuyến nghị**:
- Indoor/detail: **0.08-0.10m**
- General: **0.10-0.12m** ⭐
- Outdoor/speed: **0.15-0.20m**

#### `preprocess.blind`

**Cơ chế hoạt động**: Bỏ tất cả điểm có khoảng cách < blind từ LiDAR origin.

**Giá trị và tác dụng**:
- **0.0-0.3m**: Không bỏ hoặc bỏ ít
  - MID360: Có thể có noise ở gần (do multi-return)
  - Risk: Nhận noise làm feature
- **0.5-0.8m** (current): Cân bằng ⭐
  - MID360: Bỏ noise gần, giữ thông tin hữu ích
  - Phù hợp: Hầu hết trường hợp
- **1.0-2.0m**: Bỏ nhiều
  - Phù hợp: Khi có vật cản gần (như xe, tay cầm)
  - Risk: Mất thông tin quan trọng

**Lưu ý**: MID360 có blind zone tự nhiên ~0.3-0.5m, nên blind=0.8m là hợp lý.

#### `preprocess.lidar_type`

**Giá trị**:
- **0**: AVIA (Livox Avia)
- **1**: HORIZON (Livox Horizon)  
- **8**: MID360 (Livox MID360) ⭐ current
- **9**: HAP (Livox HAP)

**Quan trọng**: Phải đúng loại LiDAR để xử lý đúng scan pattern.

#### `preprocess.scan_line`

**Giá trị**:
- **MID360**: 6 lines (current)
- **AVIA**: 6 lines
- **HORIZON**: 1 line (spinning)

**Quan trọng**: Sai giá trị này sẽ làm scan pattern không đúng → drift lớn.

### Mối quan hệ giữa các tham số:

```
point_filter_num ↓ → filter_size_surf có thể ↑ (vì đã giảm điểm rồi)
filter_size_surf ↓ → blind có thể ↓ (vì đã filter rồi)
blind ↑ → point_filter_num có thể ↑ (vì đã bỏ điểm gần rồi)
```

### Hiệu quả tổng hợp:

**Cấu hình tốc độ** (point_filter_num=3, filter_size_surf=0.20, blind=0.8):
- Điểm/scan: ~20,000
- Tốc độ: ~2x real-time
- Memory: ~1GB/100m
- Độ chính xác: ~85%

**Cấu hình cân bằng** (point_filter_num=1, filter_size_surf=0.12, blind=0.8): ⭐
- Điểm/scan: ~80,000
- Tốc độ: ~1x real-time
- Memory: ~2.5GB/100m
- Độ chính xác: ~95%

**Cấu hình chất lượng** (point_filter_num=1, filter_size_surf=0.08, blind=0.5):
- Điểm/scan: ~120,000
- Tốc độ: ~0.6x real-time
- Memory: ~5GB/100m
- Độ chính xác: ~98%

---

## 5. VIO (Visual-Inertial Odometry)

Tham số cho Visual-Inertial Odometry.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `vio.max_iterations` | int | 5 | 3 | 1-20 | Số lần lặp tối đa cho optimization |
| `vio.outlier_threshold` | int | 100 | 1000 | 10-10000 | Ngưỡng loại bỏ outlier (pixel) |
| `vio.img_point_cov` | int | 100 | 100 | 10-1000 | Covariance của điểm ảnh |
| `vio.patch_size` | int | 8 | 7 | 3-15 | Kích thước patch (pixel) |
| `vio.patch_pyrimid_level` | int | 4 | 3 | 1-5 | Số cấp pyramid cho patch |
| `vio.normal_en` | bool | true | true | true/false | Bật/tắt sử dụng normal vector |
| `vio.raycast_en` | bool | false | false | true/false | Bật/tắt raycast (thường tắt) |
| `vio.inverse_composition_en` | bool | false | false | true/false | Bật/tắt inverse composition |
| `vio.exposure_estimate_en` | bool | true | true | true/false | Bật/tắt ước lượng exposure time |
| `vio.inv_expo_cov` | double | 0.1 | 0.1 | 0.01-1.0 | Covariance của inverse exposure |
| `vio.grid_size` | int | 5 | - | 3-10 | Kích thước grid cho feature detection |
| `vio.grid_n_height` | int | 17 | - | 10-30 | Chiều cao grid cho feature detection |

### Giải thích chi tiết:

#### `vio.max_iterations` ⭐ **RẤT QUAN TRỌNG**

**Cơ chế hoạt động**: Số lần lặp tối đa trong Gauss-Newton optimization để tìm pose tốt nhất.

**Giá trị và tác dụng**:
- **1-2**: Rất nhanh nhưng không đủ
  - Convergence: ~60-70%
  - Drift: Cao (~2-5cm/m)
  - Thời gian: ~5-10ms/frame
  - **Không khuyến nghị** trừ khi cần tốc độ cực cao
- **3** (current): Cân bằng tốt ⭐
  - Convergence: ~85-90%
  - Drift: Vừa phải (~1-2cm/m)
  - Thời gian: ~15-25ms/frame
  - **Khuyến nghị cho real-time**
- **4-5**: Chất lượng tốt
  - Convergence: ~92-95%
  - Drift: Thấp (~0.5-1cm/m)
  - Thời gian: ~30-50ms/frame
  - **Khuyến nghị cho mapping chất lượng**
- **6-10**: Rất chính xác nhưng chậm
  - Convergence: ~96-98%
  - Drift: Rất thấp (~0.2-0.5cm/m)
  - Thời gian: ~60-150ms/frame
  - **Chỉ dùng cho offline processing**

**Quy tắc**: Mỗi iteration cải thiện ~5-10% accuracy, nhưng thời gian tăng tuyến tính.

**Khuyến nghị**:
- Real-time: **3**
- Quality mapping: **4-5**
- Offline: **5-7**

#### `vio.outlier_threshold` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Ngưỡng reprojection error (pixel) để loại bỏ điểm không khớp.

**Giá trị và tác dụng**:
- **10-50**: Rất chặt chẽ
  - Loại bỏ: ~30-40% điểm
  - Risk: Loại bỏ điểm tốt khi motion lớn
  - Phù hợp: Static scene, camera ổn định
- **100** (default): Cân bằng
  - Loại bỏ: ~10-15% điểm
  - Phù hợp: Hầu hết trường hợp
- **500-1000** (current): Lỏng lẻo
  - Loại bỏ: ~3-5% điểm
  - Risk: Giữ nhiều outlier → drift
  - Phù hợp: Fast motion, nhiều noise
- **2000+**: Rất lỏng
  - Loại bỏ: ~1-2% điểm
  - **Không khuyến nghị** (quá nhiều outlier)

**Công thức**: Reprojection error = ||projected_point - observed_point|| (pixel)

**Khuyến nghị**:
- Static/slow: **50-100**
- Normal: **100-500** ⭐
- Fast motion: **500-1000**

#### `vio.patch_size`

**Cơ chế hoạt động**: Kích thước patch (N×N pixel) để track feature giữa các frame.

**Giá trị và tác dụng**:
- **3-5**: Nhỏ, nhanh
  - Robust: Thấp (dễ mất track)
  - Thời gian: ~5-10ms/patch
  - Phù hợp: Texture phong phú
- **7-8** (current: 7): Cân bằng ⭐
  - Robust: Tốt
  - Thời gian: ~15-25ms/patch
  - Phù hợp: Hầu hết trường hợp
- **10-15**: Lớn, chậm
  - Robust: Rất tốt
  - Thời gian: ~40-80ms/patch
  - Phù hợp: Texture nghèo, motion lớn

**Trade-off**: Patch lớn hơn → robust hơn nhưng chậm hơn và cần texture tốt hơn.

#### `vio.patch_pyrimid_level`

**Cơ chế hoạt động**: Số cấp image pyramid (multi-scale) để track feature.

**Giá trị và tác dụng**:
- **1-2**: Ít level
  - Track range: Nhỏ (~10-20 pixel motion)
  - Thời gian: ~1x
  - Phù hợp: Slow motion
- **3** (current): Cân bằng ⭐
  - Track range: Vừa (~30-50 pixel motion)
  - Thời gian: ~1.5x
  - Phù hợp: Normal motion
- **4-5**: Nhiều level
  - Track range: Lớn (~80-150 pixel motion)
  - Thời gian: ~2-3x
  - Phù hợp: Fast motion, aggressive movement

**Cơ chế**: Level 0 = full resolution, level N = downsampled 2^N lần.

**Khuyến nghị**:
- Slow/indoor: **2-3**
- Normal: **3** ⭐
- Fast/outdoor: **4**

#### `vio.img_point_cov`

**Cơ chế hoạt động**: Covariance (uncertainty) của điểm ảnh trong optimization.

**Giá trị và tác dụng**:
- **10-50**: Tin tưởng cao vào điểm ảnh
  - Phù hợp: Camera chất lượng cao, ít noise
  - Risk: Quá tin tưởng → drift khi có error
- **100** (default, current): Cân bằng ⭐
  - Phù hợp: Hầu hết trường hợp
- **200-500**: Ít tin tưởng
  - Phù hợp: Camera chất lượng thấp, nhiều noise
  - Risk: Quá ít tin tưởng → chậm convergence

**Khuyến nghị**: **100** cho hầu hết trường hợp.

#### `vio.normal_en`

**Cơ chế hoạt động**: Sử dụng normal vector của surface để constraint optimization.

**Tác dụng**:
- **true** (current): Sử dụng normal
  - Accuracy: +10-15%
  - Thời gian: +20-30%
  - **Khuyến nghị** ⭐
- **false**: Không dùng normal
  - Accuracy: Thấp hơn
  - Thời gian: Nhanh hơn
  - Chỉ dùng khi cần tốc độ cực cao

#### `vio.exposure_estimate_en`

**Cơ chế hoạt động**: Tự động ước lượng exposure time của camera.

**Tác dụng**:
- **true** (current): Ước lượng exposure
  - Phù hợp: Camera auto-exposure
  - Cải thiện: Track tốt hơn khi brightness thay đổi
  - **Khuyến nghị** ⭐
- **false**: Không ước lượng
  - Phù hợp: Camera fixed exposure
  - Nhanh hơn một chút

### Mối quan hệ:

```
max_iterations ↑ → outlier_threshold có thể ↑ (vì có nhiều lần để sửa)
patch_size ↑ → patch_pyrimid_level có thể ↓ (vì patch lớn đã robust)
outlier_threshold ↑ → img_point_cov có thể ↑ (để balance)
```

### Hiệu quả tổng hợp:

**Cấu hình tốc độ** (max_iterations=2, patch_size=5, patch_pyrimid_level=2):
- Thời gian: ~10-15ms/frame
- Accuracy: ~75-80%
- Drift: ~3-5cm/m

**Cấu hình cân bằng** (max_iterations=3, patch_size=7, patch_pyrimid_level=3): ⭐
- Thời gian: ~20-30ms/frame
- Accuracy: ~85-90%
- Drift: ~1-2cm/m

**Cấu hình chất lượng** (max_iterations=5, patch_size=8, patch_pyrimid_level=4):
- Thời gian: ~50-80ms/frame
- Accuracy: ~92-95%
- Drift: ~0.5-1cm/m

---

## 6. IMU

Tham số cho IMU processing.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `imu.imu_en` | bool | true | true | true/false | Bật/tắt IMU |
| `imu.imu_int_frame` | int | 30 | 30 | 10-100 | Số frame tích phân IMU |
| `imu.acc_cov` | double | 1.0 | 0.8 | 0.1-10.0 | Covariance của accelerometer |
| `imu.gyr_cov` | double | 1.0 | 0.5 | 0.1-10.0 | Covariance của gyroscope |
| `imu.gravity_est_en` | bool | true | - | true/false | Bật/tắt ước lượng gravity |
| `imu.ba_bg_est_en` | bool | true | - | true/false | Bật/tắt ước lượng bias (acc/gyr) |
| `imu.b_acc_cov` | double | - | 0.0002 | 0.00001-0.001 | Covariance của accelerometer bias |
| `imu.b_gyr_cov` | double | - | 0.0002 | 0.00001-0.001 | Covariance của gyroscope bias |

### Giải thích chi tiết:

#### `imu.acc_cov` và `imu.gyr_cov` ⭐⭐⭐ **RẤT QUAN TRỌNG**

**Cơ chế hoạt động**: Covariance (uncertainty) của IMU noise trong EKF filter. Giá trị lớn = filter ít tin tưởng IMU hơn.

**acc_cov** (accelerometer covariance):
- **0.1-0.5**: Tin tưởng cao vào IMU
  - Phù hợp: IMU chất lượng cao (tactical grade)
  - Risk: Quá tin tưởng → drift khi IMU có noise
  - Drift: Thấp nếu IMU tốt, cao nếu IMU xấu
- **0.5-1.0** (current: 0.8): Cân bằng ⭐
  - Phù hợp: IMU consumer grade (như Livox)
  - Drift: Vừa phải (~1-2cm/m)
  - **Khuyến nghị cho MID360**
- **1.0-2.0**: Ít tin tưởng
  - Phù hợp: IMU chất lượng thấp, nhiều noise
  - Drift: Cao hơn (~2-3cm/m) nhưng robust với noise
- **2.0+**: Rất ít tin tưởng
  - Phù hợp: IMU rất xấu
  - Drift: Rất cao
  - **Không khuyến nghị**

**gyr_cov** (gyroscope covariance):
- **0.1-0.3**: Tin tưởng cao
  - Phù hợp: Gyro chất lượng cao
  - Risk: Quá tin tưởng → rotation drift
- **0.3-0.6** (current: 0.5): Cân bằng ⭐
  - Phù hợp: Gyro consumer grade
  - **Khuyến nghị cho MID360**
- **0.6-1.0**: Ít tin tưởng
  - Phù hợp: Gyro chất lượng thấp
- **1.0+**: Rất ít tin tưởng
  - **Không khuyến nghị**

**Quy tắc**: 
- Nếu IMU có nhiều noise → tăng covariance
- Nếu IMU tốt → giảm covariance
- Thường: `acc_cov > gyr_cov` (vì acc thường noise hơn gyro)

**Khuyến nghị cho MID360**:
- acc_cov: **0.8** ⭐
- gyr_cov: **0.5** ⭐

#### `imu.imu_int_frame`

**Cơ chế hoạt động**: Số frame LiDAR để tích phân IMU trước khi update.

**Giá trị và tác dụng**:
- **10-20**: Phản ứng nhanh
  - Smooth: Thấp (có thể jitter)
  - Phù hợp: Fast motion
  - Risk: Không đủ data để estimate tốt
- **20-40** (current: 30): Cân bằng ⭐
  - Smooth: Tốt
  - Phù hợp: Hầu hết trường hợp
  - **Khuyến nghị**
- **40-60**: Rất smooth
  - Smooth: Rất tốt
  - Phù hợp: Slow motion
  - Risk: Phản ứng chậm với motion nhanh
- **60+**: Quá smooth
  - Phản ứng: Rất chậm
  - **Không khuyến nghị**

**Khuyến nghị**: **30** cho hầu hết trường hợp.

#### `imu.gravity_est_en`

**Cơ chế hoạt động**: Tự động ước lượng gravity vector thay vì dùng giá trị cố định.

**Tác dụng**:
- **true** (default): Ước lượng gravity
  - Phù hợp: Khi không biết hướng gravity (như UAV)
  - Accuracy: Tốt hơn khi gravity không thẳng đứng
  - **Khuyến nghị** ⭐
- **false**: Dùng gravity cố định (9.81 m/s², hướng xuống)
  - Phù hợp: Ground robot, biết chắc gravity
  - Nhanh hơn một chút

**Khuyến nghị**: **true** cho hầu hết trường hợp.

#### `imu.ba_bg_est_en`

**Cơ chế hoạt động**: Tự động ước lượng bias của accelerometer và gyroscope.

**Tác dụng**:
- **true** (default): Ước lượng bias
  - Phù hợp: IMU consumer grade (có bias)
  - Accuracy: Tốt hơn nhiều
  - **Khuyến nghị** ⭐
- **false**: Không ước lượng bias
  - Phù hợp: IMU tactical grade (bias đã được calibrate)
  - Nhanh hơn một chút

**Quan trọng**: MID360 IMU có bias → **phải bật** (true).

**Khuyến nghị**: **true** cho hầu hết trường hợp.

#### `imu.b_acc_cov` và `imu.b_gyr_cov`

**Lưu ý**: Các tham số này có trong yaml nhưng **KHÔNG ĐƯỢC ĐỌC** trong code (có thể là bug).

**Cơ chế hoạt động** (nếu được implement): Covariance của bias estimation.

**Giá trị đề xuất** (nếu được dùng):
- **0.0001-0.0002** (current: 0.0002): Bias thay đổi chậm
  - Phù hợp: IMU ổn định
- **0.0002-0.0005**: Bias thay đổi vừa phải
  - Phù hợp: IMU có bias drift
- **0.0005+**: Bias thay đổi nhanh
  - Phù hợp: IMU rất không ổn định

### Mối quan hệ:

```
acc_cov ↑ → gyr_cov có thể ↑ (để balance)
imu_int_frame ↑ → acc_cov/gyr_cov có thể ↓ (vì đã smooth rồi)
gravity_est_en = true → ba_bg_est_en nên = true
```

### Hiệu quả tổng hợp:

**Cấu hình IMU tốt** (acc_cov=0.8, gyr_cov=0.5, imu_int_frame=30):
- Drift: ~1-2cm/m
- Robust: Tốt với noise
- Phù hợp: MID360 ⭐

**Cấu hình IMU chất lượng cao** (acc_cov=0.3, gyr_cov=0.2, imu_int_frame=20):
- Drift: ~0.5-1cm/m
- Robust: Kém với noise
- Phù hợp: Tactical grade IMU

**Cấu hình IMU chất lượng thấp** (acc_cov=1.5, gyr_cov=1.0, imu_int_frame=40):
- Drift: ~2-3cm/m
- Robust: Rất tốt với noise
- Phù hợp: Low-cost IMU

### Lưu ý:

- `gravity_est_en` và `ba_bg_est_en` có trong code nhưng không có trong yaml (dùng default=true).
- `b_acc_cov` và `b_gyr_cov` có trong yaml nhưng không được đọc trong code (có thể là bug hoặc feature chưa implement).

---

## 7. LIO (LiDAR-Inertial Odometry)

Tham số cho LiDAR-Inertial Odometry.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `lio.max_iterations` | int | - | 3 | 1-20 | Số lần lặp tối đa cho LIO optimization |
| `lio.min_iterations` | int | 5 | - | 1-20 | Số lần lặp tối thiểu (default, không có trong yaml) |
| `lio.dept_err` | double | 0.05 | 0.02 | 0.001-0.1 | Lỗi depth (m) |
| `lio.beam_err` | double | 0.02 | 0.05 | 0.001-0.1 | Lỗi beam (m) |
| `lio.min_eigen_value` | double | 0.01 | 0.0025 | 0.0001-0.1 | Giá trị eigen nhỏ nhất (để xác định plane) |
| `lio.voxel_size` | double | 0.5 | 0.55 | 0.1-2.0 | Kích thước voxel (m) |
| `lio.max_layer` | int | 1 | 2 | 1-5 | Số lớp voxel tối đa |
| `lio.max_points_num` | int | 50 | 45 | 10-100 | Số điểm tối đa trong mỗi voxel |
| `lio.layer_init_num` | vector[5] | [5,5,5,5,5] | [5,5,5,5,5] | int array | Số điểm khởi tạo cho mỗi layer |
| `lio.sigma_num` | double | 3 | - | 1.0-10.0 | Số sigma cho outlier rejection (default, không có trong yaml) |

### Giải thích chi tiết:

#### `lio.voxel_size` ⭐⭐⭐ **QUAN TRỌNG NHẤT**

**Cơ chế hoạt động**: Kích thước voxel trong voxel map - mỗi voxel chứa một plane model.

**Giá trị và tác dụng**:
- **0.2-0.3m**: Rất chi tiết
  - Voxel density: ~37-125 voxels/m³
  - Memory: Rất cao (~8-15GB cho map 100m)
  - Accuracy: Rất cao (~0.5-1cm)
  - Thời gian: Chậm (~0.3-0.5x real-time)
  - Phù hợp: Indoor, chi tiết cao
- **0.4-0.5m**: Chi tiết tốt
  - Voxel density: ~8-15 voxels/m³
  - Memory: Cao (~3-5GB cho map 100m)
  - Accuracy: Cao (~1-2cm)
  - Thời gian: Vừa phải (~0.7-0.9x real-time)
  - Phù hợp: General mapping
- **0.5-0.6m** (current: 0.55): Cân bằng ⭐
  - Voxel density: ~5-8 voxels/m³
  - Memory: Vừa phải (~2-3GB cho map 100m)
  - Accuracy: Tốt (~2-3cm)
  - Thời gian: Real-time được
  - Phù hợp: Hầu hết trường hợp
- **0.7-1.0m**: Ít chi tiết
  - Voxel density: ~1-3 voxels/m³
  - Memory: Thấp (~0.5-1GB cho map 100m)
  - Accuracy: Vừa phải (~3-5cm)
  - Thời gian: Nhanh (~1.5-2x real-time)
  - Phù hợp: Large-scale, tốc độ cao

**Công thức ước tính**:
- Số voxels ≈ (Volume) / (voxel_size)³
- Memory (GB) ≈ (Số voxels) × 200 bytes / 1e9

**Quan hệ với filter_size_surf**:
- Nên: `voxel_size ≈ 4-5 × filter_size_surf`
- Ví dụ: filter_size_surf=0.12 → voxel_size=0.5-0.6 là hợp lý

**Khuyến nghị**:
- Indoor/detail: **0.4-0.5m**
- General: **0.5-0.6m** ⭐
- Outdoor/speed: **0.7-0.9m**

#### `lio.max_iterations` ⭐ **RẤT QUAN TRỌNG**

**Cơ chế hoạt động**: Số lần lặp tối đa trong LIO optimization (ICP-like).

**Giá trị và tác dụng**:
- **1-2**: Rất nhanh nhưng không đủ
  - Convergence: ~60-70%
  - Drift: Cao (~3-5cm/m)
  - Thời gian: ~10-15ms/frame
  - **Không khuyến nghị**
- **3** (current): Cân bằng tốt ⭐
  - Convergence: ~85-90%
  - Drift: Vừa phải (~1-2cm/m)
  - Thời gian: ~20-30ms/frame
  - **Khuyến nghị cho real-time**
- **4-5**: Chất lượng tốt
  - Convergence: ~92-95%
  - Drift: Thấp (~0.5-1cm/m)
  - Thời gian: ~40-60ms/frame
  - **Khuyến nghị cho mapping chất lượng**
- **6-10**: Rất chính xác nhưng chậm
  - Convergence: ~96-98%
  - Drift: Rất thấp (~0.2-0.5cm/m)
  - Thời gian: ~80-200ms/frame
  - **Chỉ dùng cho offline**

**Khuyến nghị**:
- Real-time: **3**
- Quality mapping: **4-5**
- Offline: **5-7**

#### `lio.min_eigen_value` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Ngưỡng giá trị eigen nhỏ nhất của covariance matrix để xác định plane.

**Giá trị và tác dụng**:
- **0.001-0.002**: Rất nhạy
  - Nhận diện: Nhiều plane hơn (kể cả plane yếu)
  - Risk: Nhận nhầm noise làm plane → drift
  - Phù hợp: Scene có nhiều plane rõ ràng
- **0.002-0.005** (current: 0.0025): Cân bằng ⭐
  - Nhận diện: Plane vừa phải
  - Risk: Vừa phải
  - Phù hợp: Hầu hết trường hợp
- **0.005-0.01**: Ít nhạy
  - Nhận diện: Chỉ plane rõ ràng
  - Risk: Bỏ sót plane yếu → kém chính xác
  - Phù hợp: Scene có nhiều noise
- **0.01-0.05**: Rất ít nhạy
  - Nhận diện: Chỉ plane rất rõ ràng
  - Risk: Bỏ sót nhiều plane
  - **Không khuyến nghị**

**Cơ chế**: Eigen value nhỏ → plane yếu (như edge, corner). Eigen value lớn → plane mạnh (như wall, floor).

**Khuyến nghị**: **0.002-0.005** cho hầu hết trường hợp.

#### `lio.dept_err` và `lio.beam_err` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Ngưỡng lỗi để loại bỏ điểm không khớp với plane model.

**dept_err** (lỗi depth - khoảng cách đến plane):
- **0.01-0.02m** (current: 0.02): Chặt chẽ
  - Loại bỏ: ~15-20% điểm
  - Accuracy: Cao
  - Risk: Loại bỏ điểm tốt khi plane không hoàn hảo
- **0.02-0.05m**: Cân bằng ⭐
  - Loại bỏ: ~8-12% điểm
  - Accuracy: Tốt
  - Phù hợp: Hầu hết trường hợp
- **0.05-0.10m**: Lỏng lẻo
  - Loại bỏ: ~3-5% điểm
  - Risk: Giữ nhiều outlier

**beam_err** (lỗi beam - góc giữa beam và plane normal):
- **0.02-0.05m** (current: 0.05): Cân bằng ⭐
  - Phù hợp: Hầu hết trường hợp
- **0.05-0.10m**: Lỏng lẻo
  - Risk: Chấp nhận điểm không vuông góc với plane

**Khuyến nghị**:
- dept_err: **0.02-0.05m**
- beam_err: **0.05m**

#### `lio.max_layer` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Số lớp multi-resolution trong octree - mỗi layer có resolution khác nhau.

**Giá trị và tác dụng**:
- **1**: Single resolution
  - Thời gian: Nhanh nhất (~1x)
  - Accuracy: Vừa phải
  - Phù hợp: Scene đơn giản
- **2** (current): Cân bằng ⭐
  - Thời gian: ~1.3x
  - Accuracy: Tốt
  - Phù hợp: Hầu hết trường hợp
- **3-4**: Multi-resolution tốt
  - Thời gian: ~1.8-2.5x
  - Accuracy: Rất tốt
  - Phù hợp: Scene phức tạp, scale khác nhau
- **5+**: Quá nhiều
  - Thời gian: Rất chậm
  - Accuracy: Không cải thiện nhiều
  - **Không khuyến nghị**

**Cơ chế**: Layer 0 = full resolution, layer N = downsampled 2^N lần.

**Khuyến nghị**: **2** cho hầu hết trường hợp.

#### `lio.max_points_num`

**Cơ chế hoạt động**: Giới hạn số điểm tối đa trong mỗi voxel để giảm computation.

**Giá trị và tác dụng**:
- **30-40**: Ít điểm
  - Memory: Thấp
  - Accuracy: Vừa phải
  - Phù hợp: Scene đơn giản
- **45-55** (current: 45): Cân bằng ⭐
  - Memory: Vừa phải
  - Accuracy: Tốt
  - Phù hợp: Hầu hết trường hợp
- **60-80**: Nhiều điểm
  - Memory: Cao
  - Accuracy: Rất tốt
  - Phù hợp: Scene phức tạp, nhiều điểm

**Khuyến nghị**: **45-55** cho hầu hết trường hợp.

#### `lio.sigma_num`

**Cơ chế hoạt động**: Số sigma (standard deviation) cho outlier rejection (3-sigma rule).

**Giá trị và tác dụng**:
- **2.0-2.5**: Chặt chẽ
  - Loại bỏ: ~10-15% điểm
  - Risk: Loại bỏ điểm tốt
- **3.0** (default): Cân bằng ⭐
  - Loại bỏ: ~5-7% điểm
  - Phù hợp: Hầu hết trường hợp
- **3.5-4.0**: Lỏng lẻo
  - Loại bỏ: ~2-3% điểm
  - Risk: Giữ nhiều outlier

**Khuyến nghị**: **3.0** cho hầu hết trường hợp.

### Mối quan hệ:

```
voxel_size ↑ → max_points_num có thể ↑ (vì voxel lớn có nhiều điểm)
voxel_size ↑ → min_eigen_value có thể ↑ (vì voxel lớn có plane mạnh hơn)
max_layer ↑ → voxel_size có thể ↑ (vì đã có multi-resolution)
dept_err ↑ → beam_err có thể ↑ (để balance)
```

### Hiệu quả tổng hợp:

**Cấu hình tốc độ** (voxel_size=0.7, max_iterations=2, max_layer=1):
- Thời gian: ~15-20ms/frame
- Accuracy: ~80-85%
- Drift: ~3-4cm/m
- Memory: ~1GB/100m

**Cấu hình cân bằng** (voxel_size=0.55, max_iterations=3, max_layer=2): ⭐
- Thời gian: ~25-35ms/frame
- Accuracy: ~88-92%
- Drift: ~1-2cm/m
- Memory: ~2.5GB/100m

**Cấu hình chất lượng** (voxel_size=0.4, max_iterations=5, max_layer=3):
- Thời gian: ~60-100ms/frame
- Accuracy: ~93-96%
- Drift: ~0.5-1cm/m
- Memory: ~5GB/100m

---

## 8. Local Map

Tham số cho local map management.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `local_map.map_sliding_en` | bool | false | false | true/false | Bật/tắt sliding window cho map |
| `local_map.half_map_size` | int | 100 | 90 | 50-200 | Kích thước nửa map (m) |
| `local_map.sliding_thresh` | double | 8.0 | 8.0 | 1.0-20.0 | Ngưỡng để slide map (m) |

### Giải thích chi tiết:

- **map_sliding_en**: Sliding window → giữ map trong vùng local → giảm memory.
- **half_map_size**: Bán kính map (m). Lớn hơn → map lớn hơn → tốt cho loop closure nhưng nhiều memory hơn.
- **sliding_thresh**: Khoảng cách để slide map. Nhỏ hơn → slide thường xuyên hơn → ít memory hơn.

### Hiệu quả:

- **map_sliding_en=false**: Giữ toàn bộ map → tốt cho mapping nhưng nhiều memory.
- **half_map_size=90**: Cân bằng tốt cho hầu hết ứng dụng.

---

## 9. Memory Management

Quản lý memory và buffer.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `memory.max_lidar_buffer_size` | int | 5 | 5 | 1-20 | Số frame LiDAR tối đa trong buffer |
| `memory.max_imu_buffer_size` | int | 100 | 100 | 10-500 | Số message IMU tối đa trong buffer |
| `memory.max_img_buffer_size` | int | 2 | 2 | 1-10 | Số frame image tối đa trong buffer |
| `memory.forced_sliding_interval` | int | 10 | 5 | 1-50 | Force slide map sau N frames |
| `memory.vio_feat_map_cleanup_interval` | int | 30 | 20 | 5-100 | Cleanup VIO feature map sau N frames |
| `memory.vio_feat_map_cleanup_radius` | double | 15.0 | 15.0 | 5.0-50.0 | Bán kính cleanup VIO feature map (m) |

### Giải thích chi tiết:

- **max_*_buffer_size**: Giới hạn buffer để tránh memory leak. Nhỏ hơn → ít memory hơn nhưng có thể mất dữ liệu khi xử lý chậm.
- **forced_sliding_interval**: Force cleanup map định kỳ → giảm memory tích tụ.
- **vio_feat_map_cleanup**: Cleanup feature map của VIO → quan trọng để tránh memory leak trong VIO.

### Hiệu quả:

- **Buffer nhỏ**: Giảm memory nhưng có thể mất sync khi xử lý chậm.
- **Cleanup thường xuyên**: Giảm memory leak nhưng có thể mất feature cũ.

---

## 10. UAV

Tham số đặc biệt cho UAV.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `uav.imu_rate_odom` | bool | false | false | true/false | Sử dụng IMU rate cho odometry |
| `uav.gravity_align_en` | bool | false | false | true/false | Căn chỉnh gravity |

### Giải thích chi tiết:

- **imu_rate_odom**: Publish odometry ở tần số IMU (thường cao hơn) → tốt cho control loop nhanh.
- **gravity_align_en**: Tự động căn chỉnh gravity → tốt khi không biết hướng gravity.

---

## 11. Publish

Tham số cho publishing point clouds và visualization.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `publish.dense_map_en` | bool | false | false | true/false | Publish dense map (toàn bộ điểm, chưa downsampling) |
| `publish.pub_effect_point_en` | bool | false | false | true/false | Publish effective points (điểm được dùng trong optimization) |
| `publish.pub_plane_en` | bool | false | false | true/false | Publish plane map (voxel planes) |
| `publish.pub_scan_num` | int | 1 | 3 | 1-10 | Publish mỗi N scans (tăng = giảm publish rate) |
| `publish.blind_rgb_points` | double | 0.01 | 0.0 | 0.0-1.0 | Bỏ RGB points trong blind zone (m) |

### Giải thích chi tiết:

#### `publish.dense_map_en` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Quyết định publish point cloud nào:
- **true**: Publish `feats_undistort` - toàn bộ điểm sau khi undistort, **chưa downsampling**
- **false** (current): Publish `feats_down_body` - điểm đã được downsampling bằng `filter_size_surf`

**Giá trị và tác dụng**:

**false** (current, khuyến nghị) ⭐:
- **Point cloud**: Đã downsampling
- **Số điểm**: ~10-50% so với dense (tùy `filter_size_surf`)
- **Bandwidth**: Thấp
  - MID360: ~20,000-50,000 điểm/scan
  - Data rate: ~5-15 MB/s
  - Network: ~40-120 Mbps
- **Memory**: Thấp
  - Buffer: ~50-200 MB
- **Tốc độ**: Nhanh
  - Publish time: ~5-15ms
- **Phù hợp**: Real-time, visualization, network yếu

**true** (không khuyến nghị cho real-time):
- **Point cloud**: Toàn bộ điểm, chưa downsampling
- **Số điểm**: 100% điểm gốc
- **Bandwidth**: Rất cao
  - MID360: ~100,000-200,000 điểm/scan
  - Data rate: ~30-60 MB/s
  - Network: ~240-480 Mbps
- **Memory**: Rất cao
  - Buffer: ~200-800 MB
- **Tốc độ**: Chậm
  - Publish time: ~20-50ms
- **Phù hợp**: Offline analysis, debugging, cần toàn bộ điểm

**Công thức ước tính**:
- Dense points ≈ (Raw points) × 100%
- Downsampled points ≈ (Raw points) / (filter_size_surf / 0.1)²
- Bandwidth (MB/s) ≈ (Points/scan) × 32 bytes × (Publish rate Hz) / 1e6

**Khuyến nghị**: 
- Real-time: **false** ⭐
- Offline/debugging: **true** (nếu cần)

#### `publish.pub_effect_point_en` ⭐

**Cơ chế hoạt động**: Publish các điểm "effective" - những điểm được sử dụng trong LIO optimization (từ `ptpl_list` - point-to-plane correspondences).

**Giá trị và tác dụng**:

**false** (current, default):
- Không publish effective points
- Tiết kiệm bandwidth
- Phù hợp: Hầu hết trường hợp

**true**:
- Publish effective points
- **Số điểm**: ~500-2000 điểm/scan (chỉ điểm được match với plane)
- **Bandwidth**: Thấp (~1-5 MB/s)
- **Mục đích**: 
  - Debug: Xem điểm nào được dùng trong optimization
  - Visualization: Hiển thị matching points
  - Analysis: Phân tích quality của matching
- **Topic**: `/cloud_effected`

**Khuyến nghị**:
- Production: **false**
- Debugging: **true** (để xem matching quality)

#### `publish.pub_plane_en` ⭐

**Cơ chế hoạt động**: Publish plane map - các plane được detect trong voxel map (dưới dạng visualization markers).

**Giá trị và tác dụng**:

**false** (current, default):
- Không publish plane map
- Tiết kiệm bandwidth
- Phù hợp: Hầu hết trường hợp

**true**:
- Publish plane map
- **Format**: Visualization markers (planes, normals)
- **Bandwidth**: Thấp (~0.1-1 MB/s)
- **Mục đích**:
  - Debug: Xem plane detection
  - Visualization: Hiển thị plane structure
  - Analysis: Phân tích plane quality
- **Topics**: 
  - `/planner_normal` (plane normals)
  - `/voxels` (voxel structure)

**Lưu ý**: Tham số này được đọc từ `voxelmap_manager->config_setting_.is_pub_plane_map_` (từ `lio.pub_plane_en` trong voxel config).

**Khuyến nghị**:
- Production: **false**
- Debugging: **true** (để xem plane detection)

#### `publish.pub_scan_num` ⭐ **QUAN TRỌNG**

**Cơ chế hoạt động**: Chỉ publish mỗi N scans, bỏ qua (N-1) scans giữa các lần publish.

**Giá trị và tác dụng**:

- **1** (default): Publish mỗi scan
  - Publish rate: 100% (mỗi scan)
  - Bandwidth: Tối đa
  - Phù hợp: Cần real-time visualization
  - **Không khuyến nghị** cho network yếu

- **2-3** (current: 3): Publish mỗi 2-3 scans ⭐
  - Publish rate: 33-50%
  - Bandwidth: Giảm 50-67%
  - Phù hợp: Hầu hết trường hợp
  - **Khuyến nghị** cho real-time

- **4-5**: Publish mỗi 4-5 scans
  - Publish rate: 20-25%
  - Bandwidth: Giảm 75-80%
  - Phù hợp: Network yếu, không cần real-time
  - Visualization: Có thể lag

- **6-10**: Publish rất ít
  - Publish rate: 10-17%
  - Bandwidth: Giảm 83-90%
  - Phù hợp: Network rất yếu
  - **Không khuyến nghị** (visualization quá lag)

**Công thức**:
- Publish rate = 1 / pub_scan_num
- Bandwidth reduction = (pub_scan_num - 1) / pub_scan_num × 100%

**Ví dụ**:
- pub_scan_num=1: Publish 100% scans → 100% bandwidth
- pub_scan_num=3: Publish 33% scans → 33% bandwidth (giảm 67%)
- pub_scan_num=5: Publish 20% scans → 20% bandwidth (giảm 80%)

**Khuyến nghị**:
- Real-time visualization: **1-2**
- General use: **3** ⭐
- Network yếu: **4-5**
- Offline: **1** (không cần giảm)

#### `publish.blind_rgb_points` ⭐

**Cơ chế hoạt động**: Bỏ các điểm RGB có khoảng cách từ origin < `blind_rgb_points` khi publish RGB point cloud.

**Giá trị và tác dụng**:

- **0.0** (current): Không bỏ điểm nào
  - Publish: Tất cả điểm RGB
  - Risk: Có thể có noise ở gần
  - Phù hợp: Khi đã filter tốt ở preprocessing

- **0.01-0.05m**: Bỏ điểm rất gần
  - Publish: Bỏ điểm < 1-5cm
  - Phù hợp: Loại bỏ noise ở rất gần
  - **Khuyến nghị** nếu có noise

- **0.1-0.3m**: Bỏ điểm gần
  - Publish: Bỏ điểm < 10-30cm
  - Phù hợp: Khi có vật cản gần (như tay cầm, mount)
  - Risk: Mất thông tin quan trọng

- **0.5-1.0m**: Bỏ nhiều điểm
  - Publish: Bỏ điểm < 50-100cm
  - **Không khuyến nghị** (mất quá nhiều thông tin)

**Cơ chế**: 
```cpp
if (point_distance > blind_rgb_points) {
    publish_point();
}
```

**Quan hệ với `preprocess.blind`**:
- `blind_rgb_points` chỉ ảnh hưởng đến **publishing**, không ảnh hưởng đến processing
- `preprocess.blind` ảnh hưởng đến **processing** (đã bỏ điểm từ đầu)
- Nên: `blind_rgb_points` ≤ `preprocess.blind` (vì đã filter rồi)

**Khuyến nghị**:
- Đã có `preprocess.blind`: **0.0** (không cần filter lại)
- Có noise trong RGB: **0.01-0.05m**
- Có vật cản gần: **0.1-0.3m**

### Mối quan hệ giữa các tham số:

```
dense_map_en = true → pub_scan_num nên ↑ (vì bandwidth cao)
pub_scan_num ↑ → blind_rgb_points có thể ↑ (vì ít publish hơn, có thể filter kỹ hơn)
pub_effect_point_en = true → pub_scan_num có thể ↑ (vì thêm 1 topic)
pub_plane_en = true → pub_scan_num có thể ↑ (vì thêm markers)
```

### Hiệu quả tổng hợp:

**Cấu hình tối thiểu bandwidth** (dense_map_en=false, pub_scan_num=5, pub_effect_point_en=false, pub_plane_en=false):
- Bandwidth: ~2-5 MB/s
- Publish rate: 20% scans
- Memory: ~50-100 MB
- Phù hợp: Network yếu, không cần real-time visualization

**Cấu hình cân bằng** (dense_map_en=false, pub_scan_num=3, pub_effect_point_en=false, pub_plane_en=false): ⭐
- Bandwidth: ~5-10 MB/s
- Publish rate: 33% scans
- Memory: ~100-200 MB
- Phù hợp: Hầu hết trường hợp, real-time visualization

**Cấu hình real-time** (dense_map_en=false, pub_scan_num=1, pub_effect_point_en=false, pub_plane_en=false):
- Bandwidth: ~15-30 MB/s
- Publish rate: 100% scans
- Memory: ~200-400 MB
- Phù hợp: Cần real-time visualization, network tốt

**Cấu hình debugging** (dense_map_en=true, pub_scan_num=1, pub_effect_point_en=true, pub_plane_en=true):
- Bandwidth: ~50-100 MB/s
- Publish rate: 100% scans
- Memory: ~500-1000 MB
- Phù hợp: Offline debugging, analysis

### Khuyến nghị sử dụng:

**Production/Real-time**:
```yaml
publish:
  dense_map_en: false          # Tiết kiệm bandwidth
  pub_effect_point_en: false   # Không cần debug
  pub_plane_en: false          # Không cần debug
  pub_scan_num: 3              # Cân bằng
  blind_rgb_points: 0.0        # Đã filter ở preprocessing
```

**Debugging/Analysis**:
```yaml
publish:
  dense_map_en: true           # Cần toàn bộ điểm
  pub_effect_point_en: true    # Xem matching
  pub_plane_en: true           # Xem plane detection
  pub_scan_num: 1              # Real-time
  blind_rgb_points: 0.01       # Filter noise
```

---

## 12. EVO

Tham số cho evaluation (EVO).

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `evo.seq_name` | string | "01" | "MID360_Equirectangular" | string | Tên sequence cho evaluation |
| `evo.pose_output_en` | bool | false | false | true/false | Xuất pose cho EVO evaluation |

### Giải thích chi tiết:

- **seq_name**: Tên sequence → dùng để đặt tên file output.
- **pose_output_en**: Xuất pose → để đánh giá bằng EVO tool.

---

## 13. PCD Save

Tham số cho lưu PCD files.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `pcd_save.pcd_save_en` | bool | false | true | true/false | Bật/tắt lưu PCD |
| `pcd_save.interval` | int | -1 | -1 | -1 hoặc >0 | Interval lưu PCD (-1=lưu khi dừng, >0=lưu mỗi N scans) |
| `pcd_save.filter_size_pcd` | double | 0.5 | 0.12 | 0.05-1.0 | Kích thước filter cho PCD (m) |
| `pcd_save.colmap_output_en` | bool | false | false | true/false | Xuất format COLMAP |

### Giải thích chi tiết:

- **interval=-1**: Chỉ lưu khi dừng (force save) → tiết kiệm disk.
- **interval>0**: Lưu mỗi N scans → có thể merge sau thành `all_raw_points.pcd`.
- **filter_size_pcd**: Downsample khi lưu → giảm kích thước file.

### Hiệu quả:

- **interval=-1**: Tiết kiệm disk, chỉ lưu khi cần.
- **filter_size_pcd nhỏ**: Giữ nhiều chi tiết nhưng file lớn hơn.

---

## 14. Camera

Tham số cho camera model.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `camera.model` | string | - | "Equirectangular" | "Pinhole"/"Equirectangular" | Loại camera model |
| `camera.width` | int | - | 2880 | 100-10000 | Độ rộng ảnh (pixel) |
| `camera.height` | int | - | 1440 | 100-10000 | Độ cao ảnh (pixel) |
| `camera.scale` | double | - | 1.0 | 0.1-1.0 | Scale factor (1.0=full resolution) |

### Giải thích chi tiết:

- **model**: Loại camera. "Equirectangular" cho 360° camera, "Pinhole" cho camera thường.
- **width/height**: Resolution của camera. Phải khớp với image từ camera.
- **scale**: Scale down resolution. <1.0 → nhanh hơn nhưng kém chi tiết.

### Hiệu quả:

- **Equirectangular**: Tốt cho 360° camera nhưng tính toán phức tạp hơn.
- **scale=1.0**: Full resolution → tốt nhất nhưng chậm nhất.

---

## 15. Debug

Tham số cho debugging.

| Parameter | Type | Default | Current | Range | Mô tả |
|-----------|------|---------|---------|-------|-------|
| `debug.plot_time` | double | -10 | - | seconds | Thời gian để plot debug info |
| `debug.frame_cnt` | int | 6 | - | 1-100 | Số frame để debug |

### Lưu ý:

- Các tham số này có trong code nhưng không có trong yaml (dùng default).

---

## Tổng kết các tham số thiếu trong YAML

Các tham số sau có trong code nhưng **KHÔNG CÓ** trong file yaml hiện tại (sẽ dùng default):

1. `imu.gravity_est_en` (default: true)
2. `imu.ba_bg_est_en` (default: true)
3. `preprocess.scan_rate` (default: 10)
4. `preprocess.feature_extract_enabled` (default: false)
5. `vio.grid_size` (default: 5)
6. `vio.grid_n_height` (default: 17)
7. `lio.min_iterations` (default: 5)
8. `lio.sigma_num` (default: 3)
9. `debug.plot_time` (default: -10)
10. `debug.frame_cnt` (default: 6)

**Lưu ý**: `imu.b_acc_cov` và `imu.b_gyr_cov` có trong yaml nhưng **KHÔNG ĐƯỢC ĐỌC** trong code (có thể là bug hoặc feature chưa implement).

---

## Top 10 Tham Số Quan Trọng Nhất

Dựa trên ảnh hưởng đến chất lượng và hiệu suất, đây là top 10 tham số quan trọng nhất:

### 1. `lio.voxel_size` ⭐⭐⭐
**Ảnh hưởng**: Rất lớn đến accuracy, memory, và tốc độ  
**Giá trị khuyến nghị**: 0.5-0.6m  
**Tác dụng**: Quyết định độ chi tiết của map và tốc độ xử lý

### 2. `preprocess.filter_size_surf` ⭐⭐⭐
**Ảnh hưởng**: Rất lớn đến số điểm và memory  
**Giá trị khuyến nghị**: 0.10-0.12m  
**Tác dụng**: Quyết định density của point cloud

### 3. `vio.max_iterations` ⭐⭐
**Ảnh hưởng**: Lớn đến accuracy và tốc độ VIO  
**Giá trị khuyến nghị**: 3-5  
**Tác dụng**: Quyết định độ chính xác của visual odometry

### 4. `lio.max_iterations` ⭐⭐
**Ảnh hưởng**: Lớn đến accuracy và tốc độ LIO  
**Giá trị khuyến nghị**: 3-5  
**Tác dụng**: Quyết định độ chính xác của LiDAR odometry

### 5. `imu.acc_cov` và `imu.gyr_cov` ⭐⭐
**Ảnh hưởng**: Lớn đến drift và robustness  
**Giá trị khuyến nghị**: acc_cov=0.8, gyr_cov=0.5  
**Tác dụng**: Quyết định mức độ tin tưởng vào IMU

### 6. `preprocess.point_filter_num` ⭐
**Ảnh hưởng**: Vừa phải đến số điểm và tốc độ  
**Giá trị khuyến nghị**: 1-3  
**Tác dụng**: Quyết định số điểm được xử lý

### 7. `lio.min_eigen_value` ⭐
**Ảnh hưởng**: Vừa phải đến plane detection  
**Giá trị khuyến nghị**: 0.002-0.005  
**Tác dụng**: Quyết định độ nhạy của plane detection

### 8. `vio.outlier_threshold` ⭐
**Ảnh hưởng**: Vừa phải đến quality của VIO  
**Giá trị khuyến nghị**: 100-1000  
**Tác dụng**: Quyết định số outlier được loại bỏ

### 9. `lio.dept_err` và `lio.beam_err` ⭐
**Ảnh hưởng**: Vừa phải đến accuracy của LIO  
**Giá trị khuyến nghị**: dept_err=0.02-0.05, beam_err=0.05  
**Tác dụng**: Quyết định ngưỡng loại bỏ điểm không khớp

### 10. `lio.max_layer` ⭐
**Ảnh hưởng**: Vừa phải đến accuracy và tốc độ  
**Giá trị khuyến nghị**: 2  
**Tác dụng**: Quyết định multi-resolution capability

---

## Khuyến nghị tuning theo mục đích

### 🎯 Cho độ chính xác cao (Offline/Quality Mapping):

**Mục tiêu**: Accuracy tối đa, không quan tâm tốc độ

```yaml
preprocess:
  point_filter_num: 1          # Giữ tất cả điểm
  filter_size_surf: 0.08        # Rất chi tiết
  blind: 0.5

vio:
  max_iterations: 5             # Nhiều iterations
  outlier_threshold: 500
  patch_size: 8
  patch_pyrimid_level: 4

lio:
  max_iterations: 5             # Nhiều iterations
  voxel_size: 0.4               # Chi tiết
  max_layer: 3                  # Multi-resolution
  min_eigen_value: 0.002        # Nhạy
  dept_err: 0.015               # Chặt chẽ
  beam_err: 0.04
  max_points_num: 60

imu:
  acc_cov: 0.6                  # Tin tưởng hơn (nếu IMU tốt)
  gyr_cov: 0.4
```

**Kết quả dự kiến**:
- Accuracy: ~95-98%
- Drift: ~0.3-0.5cm/m
- Tốc độ: ~0.4-0.6x real-time
- Memory: ~5-8GB/100m

### ⚡ Cho tốc độ cao (Real-time):

**Mục tiêu**: Real-time hoặc nhanh hơn, chấp nhận accuracy thấp hơn

```yaml
preprocess:
  point_filter_num: 3           # Bỏ nhiều điểm
  filter_size_surf: 0.20        # Ít chi tiết
  blind: 0.8

vio:
  max_iterations: 2             # Ít iterations
  outlier_threshold: 1000
  patch_size: 5
  patch_pyrimid_level: 2

lio:
  max_iterations: 2             # Ít iterations
  voxel_size: 0.7               # Ít chi tiết
  max_layer: 1                  # Single resolution
  min_eigen_value: 0.005        # Ít nhạy
  dept_err: 0.05                # Lỏng lẻo
  beam_err: 0.08
  max_points_num: 40

imu:
  acc_cov: 1.0                  # Ít tin tưởng (robust)
  gyr_cov: 0.6
```

**Kết quả dự kiến**:
- Accuracy: ~80-85%
- Drift: ~2-3cm/m
- Tốc độ: ~1.5-2x real-time
- Memory: ~1GB/100m

### ⚖️ Cho cân bằng (General Use) ⭐ **KHUYẾN NGHỊ**:

**Mục tiêu**: Cân bằng tốt giữa accuracy và tốc độ

```yaml
preprocess:
  point_filter_num: 1           # Giữ tất cả
  filter_size_surf: 0.12        # Chi tiết vừa phải
  blind: 0.8

vio:
  max_iterations: 3             # Cân bằng
  outlier_threshold: 1000
  patch_size: 7
  patch_pyrimid_level: 3

lio:
  max_iterations: 3             # Cân bằng
  voxel_size: 0.55              # Cân bằng
  max_layer: 2                  # Multi-resolution
  min_eigen_value: 0.0025       # Cân bằng
  dept_err: 0.02                # Cân bằng
  beam_err: 0.05
  max_points_num: 45

imu:
  acc_cov: 0.8                  # Cân bằng
  gyr_cov: 0.5
```

**Kết quả dự kiến**:
- Accuracy: ~88-92%
- Drift: ~1-2cm/m
- Tốc độ: ~1x real-time
- Memory: ~2.5GB/100m

### 💾 Cho memory thấp:

**Mục tiêu**: Giảm memory usage, chấp nhận accuracy thấp hơn

```yaml
preprocess:
  point_filter_num: 3
  filter_size_surf: 0.20
  blind: 0.8

lio:
  voxel_size: 0.7               # Lớn hơn
  max_points_num: 35            # Ít điểm hơn
  max_layer: 1

local_map:
  half_map_size: 50             # Map nhỏ hơn
  map_sliding_en: true          # Bật sliding

memory:
  max_lidar_buffer_size: 3
  max_imu_buffer_size: 50
  max_img_buffer_size: 1
  forced_sliding_interval: 5
  vio_feat_map_cleanup_interval: 15

publish:
  dense_map_en: false
```

**Kết quả dự kiến**:
- Memory: ~0.5-1GB/100m
- Accuracy: ~80-85%
- Tốc độ: ~1.2-1.5x real-time

---

## Tài liệu tham khảo

- Source code: `ws/src/FAST-LIVO2/src/LIVMapper.cpp` (readParameters function)
- Source code: `ws/src/FAST-LIVO2/src/voxel_map.cpp` (loadVoxelConfig function)
- Config file: `ws/src/FAST-LIVO2/config/mid360_equirectangular.yaml`

