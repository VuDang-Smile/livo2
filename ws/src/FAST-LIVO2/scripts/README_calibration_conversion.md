# Hướng dẫn chuyển đổi kết quả Calibration

## Tổng quan

Script này giúp chuyển đổi kết quả calibration từ `direct_visual_lidar_calibration` sang format của `FAST-LIVO2`.

## Quy trình

### 1. Calibration với direct_visual_lidar_calibration

Sau khi chạy calibration với `direct_visual_lidar_calibration`, bạn sẽ có file `calib.json` trong thư mục data.

File `calib.json` chứa:
```json
{
  "results": {
    "T_lidar_camera": [x, y, z, qx, qy, qz, qw]
  }
}
```

Trong đó:
- `T_lidar_camera` là transformation từ camera frame sang lidar frame
- Format: `[translation_x, translation_y, translation_z, quaternion_x, quaternion_y, quaternion_z, quaternion_w]`

### 2. Convert sang format FAST-LIVO2

Chạy script conversion:

```bash
cd /home/smile/Documents/code/livo/ws/src/FAST-LIVO2/scripts
python3 convert_calib_to_fast_livo2.py <path_to_calib.json>
```

Ví dụ:
```bash
python3 convert_calib_to_fast_livo2.py /path/to/calibration/data/calib.json
```

Hoặc lưu kết quả vào file:
```bash
python3 convert_calib_to_fast_livo2.py /path/to/calibration/data/calib.json --output calib_result.yaml
```

### 3. Sử dụng kết quả trong FAST-LIVO2

Copy kết quả `Rcl` và `Pcl` vào file config của FAST-LIVO2 (ví dụ: `config/avia.yaml`):

```yaml
extrin_calib:
  extrinsic_T: [0.04165, 0.02326, -0.0284]  # IMU to LiDAR (giữ nguyên)
  extrinsic_R: [1.0, 0.0, 0.0,              # IMU to LiDAR (giữ nguyên)
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
  # Kết quả từ direct_visual_lidar_calibration
  Rcl: [r11, r12, r13,                       # Rotation matrix từ LiDAR sang Camera
        r21, r22, r23,
        r31, r32, r33]
  Pcl: [x, y, z]                             # Translation vector từ LiDAR sang Camera
```

## Giải thích

### Transformation Convention

- **direct_visual_lidar_calibration** output: `T_lidar_camera`
  - Transform từ camera frame sang lidar frame
  - `p_lidar = T_lidar_camera * p_camera`

- **FAST-LIVO2** cần: `Rcl` và `Pcl`
  - Transform từ lidar frame sang camera frame
  - `p_camera = Rcl * p_lidar + Pcl`

Vì vậy script sẽ:
1. Đọc `T_lidar_camera` từ `calib.json`
2. Tính inverse: `T_camera_lidar = T_lidar_camera.inverse()`
3. Extract rotation matrix `Rcl` và translation `Pcl` từ `T_camera_lidar`

### Format

- **Rcl**: Rotation matrix 3x3 được flatten thành 9 values (row-major)
  - `[r11, r12, r13, r21, r22, r23, r31, r32, r33]`

- **Pcl**: Translation vector 3 values
  - `[x, y, z]`

## Dependencies

Script cần các thư viện Python:
- `numpy`
- `scipy`

Cài đặt:
```bash
pip3 install numpy scipy
```

## Ví dụ đầy đủ

```bash
# 1. Chạy calibration với direct_visual_lidar_calibration
ros2 run direct_visual_lidar_calibration calibrate /path/to/data

# 2. Convert kết quả
python3 convert_calib_to_fast_livo2.py /path/to/data/calib.json

# 3. Copy Rcl và Pcl vào config file của FAST-LIVO2
# Edit config/avia.yaml hoặc config/MARS_LVIG.yaml

# 4. Chạy FAST-LIVO2 với config đã update
ros2 launch fast_livo mapping_avia.launch.py
```

## Lưu ý

1. **Camera intrinsics**: Script này chỉ convert extrinsic (Rcl, Pcl). Camera intrinsics cần được cấu hình riêng trong `camera_pinhole.yaml` hoặc `camera_MARS_LVIG.yaml`.

2. **IMU to LiDAR**: `extrinsic_T` và `extrinsic_R` (IMU to LiDAR) không thay đổi, giữ nguyên giá trị cũ.

3. **Verification**: Sau khi update config, nên kiểm tra lại bằng cách visualize point cloud với RGB từ camera để đảm bảo calibration chính xác.

