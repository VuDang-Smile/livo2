# Livox Message Converter

Package ROS2 để convert Livox CustomMsg sang PointCloud2 format.

## Mô tả

Node này subscribe topic `/livox/lidar` (CustomMsg format) và convert sang PointCloud2, publish ra topic `/livox/points2` (hoặc `/livox/points` nếu được cấu hình).

## Cài đặt

1. Build package:
```bash
cd ws
source install/setup.sh
colcon build --packages-select livox_msg_converter --symlink-install
source install/setup.sh
```

## Sử dụng

### Chạy bằng launch file (khuyến nghị):
```bash
ros2 launch livox_msg_converter livox_msg_converter.launch.py
```

### Chạy trực tiếp:
```bash
ros2 run livox_msg_converter livox_msg_converter_node
```

### Với parameters tùy chỉnh:
```bash
ros2 run livox_msg_converter livox_msg_converter_node \
  --ros-args \
  -p input_topic:=/livox/lidar \
  -p output_topic:=/livox/points2 \
  -p frame_id:=livox_frame
```

## Parameters

- `input_topic` (string, default: `/livox/lidar`): Topic input nhận CustomMsg
- `output_topic` (string, default: `/livox/points`): Topic output publish PointCloud2
- `frame_id` (string, default: `livox_frame`): Frame ID cho PointCloud2 message
- `use_points2_if_exists` (bool, default: `true`): Nếu true và output_topic là `/livox/points`, sẽ tự động dùng `/livox/points2` để tránh conflict

## Topics

### Subscribed
- `/livox/lidar` (livox_ros_driver2::msg::CustomMsg): Input topic từ Livox driver

### Published
- `/livox/points2` (sensor_msgs::msg::PointCloud2): Output topic với PointCloud2 format

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `livox_ros_driver2`

## Lưu ý

- Node này yêu cầu Livox driver đang chạy và publish topic `/livox/lidar` với CustomMsg format
- Mặc định output topic là `/livox/points2` để tránh conflict với topic `/livox/points` có thể đã tồn tại từ driver khác
- PointCloud2 message được format theo chuẩn của Livox với các fields: x, y, z, intensity, tag, line, timestamp
