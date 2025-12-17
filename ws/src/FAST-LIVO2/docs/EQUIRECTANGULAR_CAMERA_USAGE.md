# Hướng dẫn sử dụng EquirectangularCamera trong FAST-LIVO2

## Tổng quan

EquirectangularCamera model đã được tích hợp vào `vikit_common` và có thể sử dụng trong FAST-LIVO2 cho:
- **Theta X**: Full 360x180 degree FOV (3840x1920 resolution)
- **Livox MID 360**: 360 degree horizontal, 59 degree vertical FOV

## Cấu hình Camera

### 1. Tạo config file cho camera

Tạo file config trong `ws/src/FAST-LIVO2/config/` với format:

```yaml
/**:
  ros__parameters:
    camera:
      model: Equirectangular
      width: 3840    # Image width
      height: 1920   # Image height
      scale: 1.0     # Scale factor (có thể giảm để tăng tốc)
```

**Lưu ý**: Equirectangular camera không cần `fx`, `fy`, `cx`, `cy`, hoặc distortion parameters. Intrinsics được tự động tính từ width/height.

### 2. Config files mẫu

#### Theta X (Full 360x180):
- File: `camera_equirectangular_theta_x.yaml`
- Resolution: 3840x1920
- FOV: 360° ngang x 180° dọc

#### Livox MID 360 (360x59):
- File: `camera_equirectangular_livox_mid360.yaml`
- Resolution: 3840x629 (tính từ 59/180 * 1920)
- FOV: 360° ngang x 59° dọc

## Sử dụng với Launch File

### Launch file mẫu

Đã tạo sẵn launch file: `mapping_mid360_equirectangular.launch.py`

```bash
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py
```

### Tùy chỉnh launch file

1. Chỉnh sửa `mid360_equirectangular.yaml` cho các tham số VIO/LIO
2. Chỉnh sửa `camera_equirectangular_theta_x.yaml` cho camera config
3. Đảm bảo các topics sau đang publish:
   - `/livox/lidar` (CustomMsg)
   - `/livox/imu`
   - `/image_raw` (equirectangular image)

## Calibration

**Quan trọng**: Cần calibrate lại extrinsic giữa LiDAR và camera khi sử dụng equirectangular camera:

1. Sử dụng `direct_visual_lidar_calibration` package
2. Record rosbag với equirectangular images và LiDAR points
3. Chạy calibration pipeline
4. Cập nhật `Rcl` và `Pcl` trong config file

## Tính toán Resolution cho FOV giới hạn

Với Livox MID 360 có FOV 59° dọc:

```
height = base_height * (vertical_fov / 180)
height = 1920 * (59 / 180) ≈ 629 pixels
```

Hoặc có thể dùng resolution khác tùy nhu cầu, miễn là aspect ratio phù hợp với FOV.

## Scale Factor

Scale factor có thể được điều chỉnh để tăng tốc xử lý:
- `scale: 1.0` - Full resolution
- `scale: 0.5` - Half resolution (nhanh hơn 4x)
- `scale: 0.25` - Quarter resolution (nhanh hơn 16x)

## Troubleshooting

### Camera model không load được
- Kiểm tra model name phải là `Equirectangular` (chính xác)
- Kiểm tra width/height phải là số nguyên
- Kiểm tra camera_loader.h đã include EquirectangularCamera

### Image size không khớp
- Đảm bảo image resolution từ camera khớp với config
- Kiểm tra scale factor

### Performance issues
- Giảm scale factor
- Giảm patch_size, patch_pyrimid_level trong VIO config
- Giảm max_iterations


