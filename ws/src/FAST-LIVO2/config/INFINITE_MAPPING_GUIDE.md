# Hướng dẫn Infinite Mapping với FAST-LIVO2

## Tổng quan

Config `mid360_equirectangular_infinite_mapping.yaml` được thiết kế để mapping quy mô lớn (vô hạn) với khả năng lưu map định kỳ lên disk, tránh tràn RAM.

## Các tính năng chính

### 1. **Lưu map định kỳ lên disk**
- `pcd_save.interval: 50` - Lưu mỗi 50 scans vào file riêng biệt (1.pcd, 2.pcd, ...)
- Map được lưu vào: `ws/src/FAST-LIVO2/Log/PCD/`
- Mỗi file PCD chứa ~50 scans, giúp giảm memory footprint

### 2. **Memory management tích cực**
- `half_map_size: 50` - Map nhỏ hơn (giảm 33% so với stable)
- `forced_sliding_interval: 2` - Cleanup mỗi 2 frames (rất tích cực)
- `vio_feat_map_cleanup_interval: 8` - Cleanup VIO feat_map mỗi 8 frames
- `sliding_thresh: 4.0` - Slide thường xuyên hơn

### 3. **Giảm memory khi publish**
- `dense_map_en: false` - Publish downsampled points
- `pub_scan_num: 5` - Giảm publish rate

## Cách sử dụng

### Bước 1: Chạy mapping với infinite config

```bash
# Terminal 1: Chạy FAST-LIVO2
ros2 launch fast_livo mapping_mid360_equirectangular.launch.py \
  params_file:=ws/src/FAST-LIVO2/config/mid360_equirectangular_infinite_mapping.yaml
```

### Bước 2: (Tùy chọn) Monitor memory và auto-save

```bash
# Terminal 2: Monitor memory và tự động save khi memory cao
cd ws/src/FAST-LIVO2/scripts
./monitor_memory_and_save.sh 8000 30
# 8000 = threshold 8GB, 30 = check mỗi 30 giây
```

### Bước 3: Play bag file

```bash
# Terminal 3: Play bag với rate chậm để xử lý kỹ
ros2 bag play /path/to/bag --rate 0.5
```

## Merge PCD files sau khi mapping

Sau khi mapping xong, merge tất cả các file PCD thành 1 file:

```bash
cd ws/src/FAST-LIVO2/scripts
./merge_pcd_files.sh

# Hoặc chỉ định thư mục và output file
./merge_pcd_files.sh /path/to/pcd/dir /path/to/output.pcd
```

**Yêu cầu:**
- Cần có `pcl-tools` hoặc Python với `open3d`:
  ```bash
  # Option 1: Install PCL tools
  sudo apt-get install pcl-tools
  
  # Option 2: Install Python library
  pip install open3d
  ```

## Tùy chỉnh interval lưu

Nếu muốn lưu thường xuyên hơn (ít memory hơn nhưng nhiều file hơn):

```yaml
pcd_save:
  interval: 30  # Lưu mỗi 30 scans thay vì 50
```

Nếu muốn lưu ít hơn (ít file hơn nhưng memory cao hơn):

```yaml
pcd_save:
  interval: 100  # Lưu mỗi 100 scans
```

## Kiểm tra memory usage

```bash
# Xem memory của process FAST-LIVO2
ps aux | grep fast_livo | grep -v grep

# Hoặc dùng htop
htop

# Hoặc monitor liên tục
watch -n 1 'ps aux | grep fast_livo | grep -v grep'
```

## Manual save (nếu cần)

Nếu muốn save ngay lập tức mà không đợi interval:

```bash
# Gọi save service
ros2 service call /laserMapping/save_results std_srvs/srv/Trigger
```

## Xử lý khi memory quá cao

Nếu memory vẫn quá cao dù đã dùng infinite config:

1. **Giảm `half_map_size` xuống 40:**
   ```yaml
   local_map:
     half_map_size: 40
   ```

2. **Tăng `forced_sliding_interval` lên 1:**
   ```yaml
   memory:
     forced_sliding_interval: 1  # Cleanup mỗi frame
   ```

3. **Giảm `voxel_size` lên 0.5:**
   ```yaml
   lio:
     voxel_size: 0.5  # Lớn hơn = ít memory hơn
   ```

4. **Tăng `pcd_save.interval` lên 30:**
   ```yaml
   pcd_save:
     interval: 30  # Lưu thường xuyên hơn
   ```

## Cấu trúc file sau khi mapping

```
ws/src/FAST-LIVO2/Log/PCD/
├── 1.pcd          # Scans 1-50
├── 2.pcd          # Scans 51-100
├── 3.pcd          # Scans 101-150
├── ...
├── scans_pos.json # Vị trí của mỗi scan
└── merged_map.pcd # File merged (sau khi chạy merge script)
```

## So sánh với các config khác

| Tham số | Stable | Infinite Mapping |
|---------|--------|------------------|
| `half_map_size` | 60 | **50** (-17%) |
| `forced_sliding_interval` | 3 | **2** (+50% frequency) |
| `pcd_save.interval` | -1 (chỉ khi dừng) | **50** (định kỳ) |
| `sliding_thresh` | 5.0 | **4.0** (slide thường xuyên hơn) |
| **Memory/100m** | ~1.5-2.5GB | **~1-2GB** |
| **Disk usage** | Chỉ khi dừng | **Định kỳ** |

## Lưu ý quan trọng

1. **Disk space**: Đảm bảo có đủ disk space. Mỗi file PCD có thể ~50-200MB tùy vào `filter_size_pcd`.

2. **Performance**: Lưu định kỳ có thể làm chậm một chút (~5-10% overhead).

3. **Merge files**: Sau khi mapping xong, nhớ merge các file PCD để có map hoàn chỉnh.

4. **Backup**: Nên backup các file PCD định kỳ trong quá trình mapping dài.

## Troubleshooting

### Memory vẫn tăng dù đã dùng infinite config
- Kiểm tra xem `map_sliding_en` có = true không
- Giảm `half_map_size` xuống 40
- Tăng frequency của cleanup

### PCD files không được tạo
- Kiểm tra `pcd_save_en: true`
- Kiểm tra `interval > 0`
- Kiểm tra quyền ghi vào thư mục `Log/PCD/`

### Merge script không chạy
- Cài đặt `pcl-tools` hoặc `open3d`
- Kiểm tra đường dẫn thư mục PCD
- Kiểm tra quyền thực thi của script

## Kết luận

Config infinite mapping cho phép mapping quy mô lớn mà không lo tràn RAM bằng cách:
- Lưu map định kỳ lên disk
- Memory management tích cực
- Map size nhỏ hơn
- Cleanup thường xuyên

Phù hợp cho:
- Mapping quy mô lớn (>500m)
- Mapping dài hạn (nhiều giờ)
- Hệ thống có RAM hạn chế
- Cần backup map định kỳ

