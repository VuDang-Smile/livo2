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

### System Dependencies

Script `install_dependencies.sh` sẽ tự động cài đặt tất cả dependencies cần thiết:

#### System Packages (apt)
- `libomp-dev` - OpenMP support
- `libboost-all-dev` - Boost libraries
- `libglm-dev` - OpenGL Mathematics library
- `libglfw3-dev` - OpenGL framework
- `libpng-dev` - PNG library
- `libjpeg-dev` - JPEG library

#### Libraries từ source
- **GTSAM** (version 4.2a9) - Geometry and optimization library
- **Ceres Solver** - Nonlinear optimization library
- **Iridescence** - Visualization library

#### Python Dependencies
- opencv-python
- Pillow

**Cài đặt tất cả dependencies:**
```bash
cd /home/smile/Documents/code/livo
./install_dependencies.sh
```

Script sẽ:
- Cài đặt system packages qua apt
- Clone và build GTSAM, Ceres Solver, Iridescence từ source
- Cài đặt Python dependencies (thử `--user` flag trước, nếu không được sẽ hỏi xác nhận để dùng `--break-system-packages`)

## Cách sử dụng

### 1. Cài đặt Dependencies

```bash
cd /home/smile/Documents/code/livo
./install_dependencies.sh
```

Script này sẽ cài đặt:
- System dependencies (apt packages)
- GTSAM, Ceres Solver, Iridescence (build từ source)
- Python dependencies

**Lưu ý**: Script cần quyền sudo để cài đặt system packages và build libraries.

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
cd /home/smile/Documents/code/livo
./run_viewer.sh
```

Script này sẽ tự động:
- Source ROS2 environment
- Source workspace setup.sh
- Chạy GUI

#### Cách thủ công:
```bash
cd /home/smile/Documents/code/livo/gui
source /opt/ros/jazzy/setup.bash
source ../ws/install/setup.sh

# Đảm bảo đã cài dependencies:
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
- Chạy script cài đặt: `./install_dependencies.sh` (từ root project)
- Kiểm tra ROS2 Python packages: `python3 -c "import rclpy"`

### Lỗi khi link libraries
- Chạy `sudo ldconfig` để cập nhật library cache sau khi cài đặt GTSAM, Ceres, Iridescence


