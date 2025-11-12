# Theta Driver GUI Viewer

GUI để mở theta_driver và hiển thị ảnh từ topic `/image_raw` trong ROS2.

## Yêu cầu

### ROS2 Dependencies
```bash
# Cài đặt ROS2 Jazzy (nếu chưa có)
sudo apt install ros-jazzy-desktop

# Cài đặt các packages cần thiết
sudo apt install ros-jazzy-cv-bridge ros-jazzy-sensor-msgs python3-rclpy python3-venv
```

### Python Dependencies

Có 2 cách cài đặt:

#### Cách 1: Sử dụng Virtual Environment (Khuyến nghị - cô lập dependencies)
```bash
cd gui/
./setup_venv.sh
```

Script này sẽ:
- Tạo virtual environment trong `venv/`
- Cài đặt tất cả dependencies từ `requirements.txt`
- Sẵn sàng để sử dụng

#### Cách 2: Cài đặt trực tiếp (không dùng venv) - ⚠️ Không khuyến nghị

**Lưu ý**: Trên Ubuntu 24.04 với Python 3.12, hệ thống Python được bảo vệ và có thể yêu cầu `--break-system-packages` flag.

```bash
cd gui/
./install_dependencies.sh
```

Script sẽ:
- Thử cài với `--user` flag trước
- Nếu không được, sẽ hỏi xác nhận để dùng `--break-system-packages`
- Cảnh báo về rủi ro khi cài vào system Python

**Khuyến nghị**: Nên dùng venv (Cách 1) để tránh xung đột với system packages.

## Cách sử dụng

### 1. Cài đặt Dependencies

**Chọn một trong hai cách:**

#### Cách A: Dùng Virtual Environment (khuyến nghị)
```bash
cd /home/smile/Documents/code/livo/gui
./setup_venv.sh
```

#### Cách B: Không dùng venv (cài trực tiếp)
```bash
cd /home/smile/Documents/code/livo/gui
./install_dependencies.sh
```

### 2. Build workspace (nếu chưa build)
```bash
cd /home/smile/Documents/code/livo/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select theta_driver --symlink-install
source install/setup.sh
```

### 3. Chạy GUI

#### Cách đơn giản nhất (dùng script):
```bash
cd /home/smile/Documents/code/livo/gui
./run_viewer.sh
```

Script này sẽ tự động:
- Source ROS2 environment
- Source workspace setup.sh
- Activate virtual environment (nếu có)
- Chạy GUI

#### Cách thủ công:
```bash
cd /home/smile/Documents/code/livo/gui
source /opt/ros/jazzy/setup.bash
source ../ws/install/setup.sh

# Nếu dùng venv:
source venv/bin/activate

# Nếu không dùng venv, đảm bảo đã cài dependencies:
# ./install_dependencies.sh

python3 theta_viewer.py
```

### 4. Sử dụng GUI

1. **Launch Theta Driver**: Nhấn nút "Launch Theta Driver" để tự động khởi chạy theta_driver node
   - Hoặc chạy thủ công trong terminal:
     ```bash
     source /opt/ros/jazzy/setup.bash
     source ws/install/setup.sh
     ros2 run theta_driver theta_driver_node
     ```

2. **Start Subscriber**: Sau khi theta_driver đã chạy, nhấn "Start Subscriber" để bắt đầu subscribe topic `/image_raw`

3. **Xem ảnh**: Ảnh sẽ tự động hiển thị trên canvas khi nhận được từ topic

4. **Stop**: Nhấn "Stop" để dừng subscriber và theta_driver

## Tính năng

- ✅ Tự động launch theta_driver node
- ✅ Subscribe topic `/image_raw` 
- ✅ Hiển thị ảnh real-time với auto-resize
- ✅ Hiển thị thông tin ảnh (kích thước, scale)
- ✅ Xử lý encoding RGB8 từ theta_driver
- ✅ Giao diện đơn giản, dễ sử dụng

## Lưu ý

- Đảm bảo theta camera đã được kết nối trước khi launch
- Nếu theta_driver không chạy được, kiểm tra quyền USB:
  ```bash
  sudo usermod -a -G dialout $USER
  # Sau đó logout và login lại
  ```
- GUI sẽ tự động resize ảnh để vừa với cửa sổ
- Encoding mặc định từ theta_driver là `rgb8`

## Troubleshooting

### Lỗi: "Không tìm thấy setup.sh"
- Đảm bảo đã build workspace: `colcon build --packages-select theta_driver`
- Kiểm tra đường dẫn: `ws/install/setup.sh` phải tồn tại

### Lỗi: "Topic /image_raw không có dữ liệu"
- Kiểm tra xem theta_driver đã chạy chưa: `ros2 topic list`
- Kiểm tra topic: `ros2 topic echo /image_raw --once`
- Đảm bảo camera đã kết nối và được nhận diện

### Lỗi import module
- Cài đặt dependencies: `pip3 install -r requirements.txt`
- Kiểm tra ROS2 Python packages: `python3 -c "import rclpy"`

