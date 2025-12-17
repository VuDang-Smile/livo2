# Create Calibration Bag Scripts

Scripts để tạo calibration bag từ MCAP recording với các topics cần thiết cho calibration.

## Yêu cầu

### Option 1: Sử dụng Python script (Khuyến nghị)

Cần cài đặt `rosbags` library:
```bash
pip install rosbags
# hoặc
python3 -m pip install rosbags --user
```

Nếu pip chưa được cài đặt:
```bash
sudo apt install python3-pip
pip install rosbags
```

### Option 2: Sử dụng Bash script

Không cần cài đặt thêm, chỉ cần:
- ROS2 Jazzy đã được cài đặt
- Workspace đã được build với các packages:
  - `theta_driver` (camera_info_publisher_node)
  - `livox_msg_converter` (livox_msg_converter_node)

## Cách sử dụng

### Option 1: Python Script (create_calibration_bag.py)

Script này đọc MCAP recording, convert messages và tạo bag mới. Nhanh và hiệu quả hơn.

```bash
cd /home/ubuntu/Documents/code/livo2
python3 ws/src/FAST-LIVO2/scripts/create_calibration_bag.py \
    /home/ubuntu/Documents/code/livo2/recordings/recording_20251210_165248 \
    --output /home/ubuntu/Documents/code/livo2/recordings/recording_20251210_165248_calibration \
    --duration 5
```

**Arguments:**
- `input`: Đường dẫn đến thư mục recording (bắt buộc)
- `--output, -o`: Đường dẫn output bag (mặc định: `<input>_calibration`)
- `--duration, -d`: Thời gian extract (giây, mặc định: 5.0)

**Output topics:**
- `/image_raw` (Image)
- `/camera_info` (CameraInfo) - được tạo từ Image messages
- `/livox/point2` (PointCloud2) - được convert từ CustomMsg

### Option 2: Bash Script (create_calibration_bag_ros2.sh)

Script này sử dụng ROS2 nodes để convert messages trong thời gian thực.

```bash
cd /home/ubuntu/Documents/code/livo2
bash ws/src/FAST-LIVO2/scripts/create_calibration_bag_ros2.sh \
    --input /home/ubuntu/Documents/code/livo2/recordings/recording_20251210_165248 \
    --output /home/ubuntu/Documents/code/livo2/recordings/recording_20251210_165248_calibration \
    --duration 5
```

**Arguments:**
- `--input, -i`: Đường dẫn đến thư mục recording (bắt buộc)
- `--output, -o`: Đường dẫn output bag (mặc định: `<input>_calibration`)
- `--duration, -d`: Thời gian extract (giây, mặc định: 5.0)

**Lưu ý:**
- Script này cần source ROS2 và workspace trước khi chạy
- Script sẽ tự động source `/opt/ros/jazzy/setup.bash` và workspace nếu có
- Các node converter sẽ chạy trong background và tự động dừng khi xong

## So sánh 2 phương pháp

| Tiêu chí | Python Script | Bash Script |
|----------|---------------|-------------|
| Tốc độ | Nhanh (xử lý trực tiếp) | Chậm hơn (chạy nodes) |
| Dependencies | Cần `rosbags` | Chỉ cần ROS2 |
| Độ chính xác | Cao (convert trực tiếp) | Cao (dùng nodes thật) |
| Khuyến nghị | ✅ Nên dùng | Dùng khi không có pip |

## Kiểm tra kết quả

Sau khi tạo bag, kiểm tra bằng:

```bash
ros2 bag info <output_bag_path>
```

Bag nên có các topics:
- `/image_raw` (sensor_msgs/msg/Image)
- `/camera_info` (sensor_msgs/msg/CameraInfo)
- `/livox/point2` (sensor_msgs/msg/PointCloud2)

## Troubleshooting

### Python script: "rosbags library not found"
```bash
pip install rosbags
# hoặc
python3 -m pip install rosbags --user
```

### Bash script: "node not found"
Đảm bảo workspace đã được build:
```bash
cd ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select theta_driver livox_msg_converter --symlink-install
source install/setup.bash
```

### Bash script: "No messages recorded"
- Kiểm tra recording có đúng format không
- Kiểm tra các topics có tồn tại trong recording không
- Tăng thời gian `sleep` trong script nếu cần


