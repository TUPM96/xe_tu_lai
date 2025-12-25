# Xe Tự Lái - Tránh Vật Cản Tự Động

Hệ thống xe tự lái sử dụng **Camera** và **LiDAR** để phát hiện và tránh vật cản tự động.

## Tính năng

- ✅ **Camera**: Phát hiện vạch kẻ đường và điều chỉnh để đi giữa đường
- ✅ **LiDAR**: Phát hiện và tránh vật cản tự động
- ✅ **Kết hợp thông minh**: Ưu tiên tránh vật cản, sau đó đi theo vạch kẻ đường
- ✅ Điều khiển mượt mà với DWB controller
- ✅ Không cần map - hoạt động hoàn toàn tự động

## Cài đặt

### Dependencies

```bash
# Dependencies cơ bản
sudo aptitude install -y ros-jazzy-cv-bridge \
                    ros-jazzy-v4l2-camera \
                    python3-opencv \
                    python3-numpy

# Dependencies cho simulation (nếu chạy mô phỏng)
sudo aptitude install -y ros-jazzy-gazebo-ros-pkgs \
                    ros-jazzy-gazebo-ros \
                    gazebo
```

### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Sử dụng

### 1. Chạy mô phỏng trên Gazebo (Khuyến nghị để test)

```bash
source ~/ros2_ws/install/setup.bash

# Chạy simulation với world đường phố 2 làn (mặc định) ⭐
ros2 launch xe_lidar simulation.launch.py

# Chạy với world test map (nhiều vật cản hơn)
ros2 launch xe_lidar simulation.launch.py world:=test_map.world

# Chạy với world mê cung (thử thách cao)
ros2 launch xe_lidar simulation.launch.py world:=maze_map.world

# Hoặc chạy với world trống
ros2 launch xe_lidar simulation.launch.py world:=empty.world
```

Robot sẽ tự động di chuyển và tránh vật cản trong môi trường mô phỏng!

### 2. Chạy trên robot thật

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar autonomous_drive.launch.py
```

### 3. Chạy với tham số tùy chỉnh (robot thật)

```bash
ros2 launch xe_lidar autonomous_drive.launch.py \
    serial_port:=/dev/ttyUSB0 \
    video_device:=/dev/video0
```

### 4. Chạy từng thành phần

#### Robot cơ bản
```bash
ros2 launch xe_lidar launch_robot.launch.py
```

#### LiDAR
```bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

#### Camera
```bash
sudo chmod 777 /dev/video*
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

#### Obstacle Avoidance Node
```bash
ros2 run xe_lidar obstacle_avoidance.py
```

## Cấu hình

Các tham số có thể điều chỉnh trong launch file:

- `min_distance`: Khoảng cách tối thiểu (m) - mặc định: 0.5
- `safe_distance`: Khoảng cách an toàn (m) - mặc định: 0.8
- `max_linear_speed`: Tốc độ tối đa (m/s) - mặc định: 0.3
- `max_angular_speed`: Tốc độ quay tối đa (rad/s) - mặc định: 1.0
- `front_angle_range`: Góc phía trước (degrees) - mặc định: 60
- `use_camera`: Bật/tắt camera - mặc định: true

## Kiểm tra

### Xem dữ liệu LiDAR
```bash
ros2 topic echo /scan
```

### Xem ảnh camera
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Xem lệnh điều khiển
```bash
ros2 topic echo /cmd_vel
```

## Xử lý sự cố

### LiDAR không hoạt động
```bash
# Kiểm tra port
ls -l /dev/serial/by-path/

# Cấp quyền
sudo chmod 666 /dev/ttyUSB0
```

### Camera không hoạt động
```bash
# Kiểm tra device
ls -l /dev/video*

# Cấp quyền
sudo chmod 777 /dev/video0
```

### Robot không di chuyển
```bash
# Kiểm tra controller
ros2 control list_controllers

# Kiểm tra topic
ros2 topic echo /cmd_vel
```

## Nguyên lý hoạt động

### Camera - Lane Detection:
- Phát hiện vạch kẻ đường bằng Canny edge detection và HoughLinesP
- Tính toán offset từ giữa đường và điều chỉnh góc quay

### LiDAR - Obstacle Avoidance:
- Quét 360° và phát hiện vật cản trong vùng phía trước (60°)
- Xác định hướng vật cản và thực hiện tránh

### Logic điều khiển:
- **Ưu tiên 1**: Tránh vật cản (LiDAR)
- **Ưu tiên 2**: Đi theo vạch kẻ đường (Camera)
