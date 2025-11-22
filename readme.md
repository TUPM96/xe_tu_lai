# Hệ thống Xe Tự Lái - Tránh Vật Cản Tự Động

Hệ thống xe tự lái sử dụng **Camera** và **LiDAR** để phát hiện và tránh vật cản tự động, không cần map hay navigation.

**Repository**: [https://github.com/TUPM96/xe_tu_lai](https://github.com/TUPM96/xe_tu_lai)

## Tính năng

- ✅ **Camera**: Phát hiện vạch kẻ đường và điều chỉnh để đi giữa đường
- ✅ **LiDAR**: Phát hiện và tránh vật cản tự động
- ✅ **Kết hợp thông minh**: Ưu tiên tránh vật cản, sau đó đi theo vạch kẻ đường
- ✅ **Hỗ trợ 2 loại điều khiển**: Ackermann Steering (4 bánh) và Differential Drive (2 bánh)
- ✅ **Điều khiển mượt mà** với ROS2 control
- ✅ **Không cần map** - hoạt động hoàn toàn tự động

## Cấu trúc hệ thống

```
xe_lidar/
├── scripts/
│   └── obstacle_avoidance.py    # Node xử lý tránh vật cản
├── launch/
│   ├── simulation.launch.py        # Launch file mô phỏng Gazebo ⭐
│   ├── autonomous_drive.launch.py  # Launch file robot thật
│   ├── launch_robot.launch.py      # Launch robot cơ bản
│   ├── rplidar.launch.py           # Launch LiDAR riêng
│   └── camera.launch.py            # Launch Camera riêng
├── worlds/
│   ├── road_map.world              # World đường phố 2 làn với vật cản 2 bên ⭐ (mặc định)
│   ├── obstacles.world             # World có vật cản cơ bản
│   ├── test_map.world              # World test với nhiều vật cản
│   ├── maze_map.world              # World mê cung thử thách
│   └── empty.world                 # World trống
└── config/
    └── my_controllers.yaml         # Cấu hình controller
```

## Yêu cầu hệ thống

### Hệ điều hành
- **Ubuntu 22.04 LTS** (khuyến nghị)
- Hoặc Ubuntu 20.04 với ROS2 Foxy (cần điều chỉnh)

### Phần mềm
- **ROS2 Humble Hawksbill** (bắt buộc)
- Python 3.8+
- OpenCV (python3-opencv)
- NumPy
- Gazebo (cho simulation)
- Git

### Phần cứng (Cho robot thật)
- Raspberry Pi 4 hoặc máy tính có Ubuntu 22.04
- RPLIDAR A1 hoặc tương đương
- USB Camera
- Hardware:
  - **Ackermann steering**: 4 bánh với bánh lái (khuyến nghị)
  - **Differential Drive**: 2 bánh (đơn giản hơn)

### Yêu cầu hệ thống tối thiểu
- CPU: 2 cores trở lên
- RAM: 4GB trở lên (8GB khuyến nghị)
- Dung lượng: 20GB trống trở lên
- GPU: Không bắt buộc (nhưng khuyến nghị cho Gazebo)

## Cài đặt từ đầu

### Bước 1: Cài đặt Ubuntu 22.04 và ROS2 Humble

#### 1.1. Cài đặt Ubuntu 22.04
- Tải Ubuntu 22.04 Desktop từ [ubuntu.com](https://ubuntu.com/download/desktop)
- Cài đặt trên máy ảo (VMware/VirtualBox) hoặc máy thật
- Đảm bảo có ít nhất 20GB dung lượng và 4GB RAM

#### 1.2. Cài đặt ROS2 Humble

```bash
# Cập nhật hệ thống
sudo apt update && sudo apt upgrade -y

# Cài đặt locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Thêm ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Thêm ROS2 GPG key
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Thêm ROS2 repository
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Cài đặt ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop

# Cài đặt development tools
sudo apt install -y python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool

# Khởi tạo rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Bước 2: Tải source code từ GitHub

```bash
# Tạo workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/TUPM96/xe_tu_lai.git

# Kiểm tra file Python script có trong repository
ls -la xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py
```

### Bước 3: Cài đặt dependencies

```bash
# Cài đặt dependencies cơ bản
sudo apt update
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ackermann-msgs \
    python3-opencv \
    python3-numpy \
    python3-pip \
    gazebo

# Cài đặt thêm Python packages nếu cần
# QUAN TRỌNG: Cài NumPy 1.x để tương thích với cv_bridge (NumPy 2.x chưa được hỗ trợ)
pip3 install "numpy<2.0" opencv-python

# Cài đặt dependencies cho các package trong workspace
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Bước 4: Build workspace

```bash
# Di chuyển về thư mục workspace
cd ~/ros2_ws

# QUAN TRỌNG: Đảm bảo file Python có quyền thực thi
chmod +x src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py

# Build workspace
# File sẽ tự động được copy vào: install/xe_lidar/lib/xe_lidar/obstacle_avoidance.py
colcon build --symlink-install

# Kiểm tra file đã được cài đặt đúng vị trí
ls -la install/xe_lidar/lib/xe_lidar/obstacle_avoidance.py

# Source workspace
source install/setup.bash

# Thêm vào .bashrc để tự động source mỗi khi mở terminal
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Bước 5: Kiểm tra cài đặt

```bash
# Kiểm tra ROS2 environment
printenv | grep ROS

# Kiểm tra package đã được build
ros2 pkg list | grep xe_lidar

# Kiểm tra file Python script đã được cài đặt và có thể chạy
ros2 run xe_lidar obstacle_avoidance.py --help

# Nếu chạy được lệnh trên (hiển thị help) là OK! ✅
```

### Bước 6: Cài đặt ackermann_steering_controller (Tùy chọn - nếu dùng Ackermann)

```bash
# Clone ros2_controllers repository
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_controllers.git -b humble

# Cài đặt dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build ackermann_steering_controller
colcon build --packages-select ackermann_steering_controller

# Source lại
source install/setup.bash
```

### Bước 7: Cấu hình quyền truy cập thiết bị (Cho robot thật)

```bash
# Thêm user vào dialout group (cho serial port)
sudo usermod -a -G dialout $USER

# Cấp quyền cho camera
sudo usermod -a -G video $USER

# Logout và login lại để áp dụng thay đổi
```

## Cài đặt nhanh (Nếu đã có ROS2 Humble)

Nếu bạn đã có ROS2 Humble được cài đặt, chỉ cần:

```bash
# 1. Clone repository
cd ~/ros2_ws/src
git clone https://github.com/TUPM96/xe_tu_lai.git

# 2. Cài đặt dependencies
cd ~/ros2_ws
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ackermann-msgs \
    python3-opencv \
    python3-numpy \
    gazebo

rosdep install --from-paths src --ignore-src -r -y

# QUAN TRỌNG: Cài NumPy 1.x để tương thích với cv_bridge (NumPy 2.x chưa được hỗ trợ)
pip3 install "numpy<2.0"

# 3. Đảm bảo file Python có quyền thực thi
chmod +x src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py

# 4. Build
colcon build --symlink-install
source install/setup.bash
```

## Sử dụng

### 1. Chạy mô phỏng trên Gazebo (Khuyến nghị để test)

```bash
source ~/ros2_ws/install/setup.bash

# Chạy simulation với Ackermann steering (4 bánh, bánh lái) ⭐
ros2 launch xe_lidar simulation_ackermann.launch.py

# Hoặc chạy với differential drive (2 bánh) - cũ
ros2 launch xe_lidar simulation.launch.py

# Chạy với world test map (nhiều vật cản hơn)
ros2 launch xe_lidar simulation.launch.py world:=test_map.world

# Chạy với world mê cung (thử thách cao)
ros2 launch xe_lidar simulation.launch.py world:=maze_map.world

# Hoặc chạy với world trống
ros2 launch xe_lidar simulation.launch.py world:=empty.world

# Chạy với RViz2 tự động mở
ros2 launch xe_lidar simulation.launch.py
```

Robot sẽ tự động di chuyển và tránh vật cản trong môi trường mô phỏng!

### 2. Chạy trên robot thật

```bash
source ~/ros2_ws/install/setup.bash

# Chạy với Ackermann steering (4 bánh, bánh lái) ⭐
ros2 launch xe_lidar launch_robot_ackermann.launch.py
ros2 launch xe_lidar autonomous_drive.launch.py

# Hoặc chạy với differential drive (2 bánh) - cũ
ros2 launch xe_lidar launch_robot.launch.py
ros2 launch xe_lidar autonomous_drive.launch.py
```

### 3. Chạy từng thành phần riêng lẻ

#### Chạy robot cơ bản (động cơ + controller)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar launch_robot.launch.py
```

#### Chạy LiDAR

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

#### Chạy Camera

```bash
sudo chmod 777 /dev/video*
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

#### Chạy Obstacle Avoidance Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run xe_lidar obstacle_avoidance.py
```

### 4. Điều khiển thủ công (tùy chọn)

Nếu muốn điều khiển thủ công thay vì tự động:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Cấu hình

### Tham số Obstacle Avoidance

Có thể điều chỉnh các tham số trong launch file hoặc khi chạy node:

```bash
ros2 run xe_lidar obstacle_avoidance.py --ros-args \
    -p min_distance:=0.5 \
    -p safe_distance:=0.8 \
    -p max_linear_speed:=0.3 \
    -p max_angular_speed:=1.0 \
    -p front_angle_range:=60 \
    -p use_camera:=true
```

**Các tham số:**
- `min_distance`: Khoảng cách tối thiểu để dừng (m) - mặc định: 0.5
- `safe_distance`: Khoảng cách an toàn để bắt đầu tránh (m) - mặc định: 0.8
- `max_linear_speed`: Tốc độ tối đa tiến/lùi (m/s) - mặc định: 0.3
- `max_angular_speed`: Tốc độ quay tối đa (rad/s) - mặc định: 1.0
- `front_angle_range`: Góc phía trước để kiểm tra (degrees) - mặc định: 60
- `use_camera`: Bật/tắt sử dụng camera - mặc định: true

## Kiểm tra hệ thống

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

### Xem RViz2

```bash
rviz2
```

Trong RViz2, thêm:
- **LaserScan** topic: `/scan`
- **Image** topic: `/camera/image_raw`
- **TF** để xem robot model

## Xử lý sự cố

### LiDAR không hoạt động

1. Kiểm tra port:
```bash
ls -l /dev/serial/by-path/
```

2. Kiểm tra quyền truy cập:
```bash
sudo chmod 666 /dev/ttyUSB0
```

3. Test LiDAR:
```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200
```

### Camera không hoạt động

1. Kiểm tra device:
```bash
ls -l /dev/video*
```

2. Cấp quyền:
```bash
sudo chmod 777 /dev/video0
```

3. Test camera:
```bash
ros2 run v4l2_camera v4l2_camera_node
```

### Robot không di chuyển

1. Kiểm tra controller:
```bash
ros2 control list_controllers
```

2. Kiểm tra topic cmd_vel:
```bash
ros2 topic echo /cmd_vel
ros2 topic echo /diff_cont/cmd_vel_unstamped
```

3. Kiểm tra log của obstacle_avoidance node:
```bash
ros2 topic echo /rosout | grep obstacle_avoidance
```

## Nguyên lý hoạt động

### Hệ thống kết hợp Camera + LiDAR:

1. **Camera - Phát hiện vạch kẻ đường (Lane Detection)**:
   - Sử dụng Canny edge detection và HoughLinesP để phát hiện vạch kẻ đường
   - Tính toán offset từ giữa đường
   - Điều chỉnh góc quay để đi giữa đường

2. **LiDAR - Phát hiện và tránh vật cản**:
   - Quét 360° và phát hiện vật cản trong vùng phía trước (60°)
   - Xác định hướng vật cản (trái/phải/giữa)
   - Ưu tiên cao nhất: tránh vật cản

3. **Logic điều khiển (2 mức ưu tiên)**:
   - **Ưu tiên 1 (Cao)**: Nếu có vật cản (LiDAR) → Tránh vật cản
     - Vật cản bên trái: quay phải
     - Vật cản bên phải: quay trái
     - Vật cản ở giữa: lùi lại và quay
   - **Ưu tiên 2 (Thấp)**: Nếu không có vật cản → Đi theo vạch kẻ đường (Camera)
     - Phát hiện vạch kẻ đường: điều chỉnh để đi giữa
     - Không phát hiện vạch: đi thẳng

4. **Controller** nhận lệnh và điều khiển động cơ

## Tùy chỉnh

### Chọn loại điều khiển

**Ackermann Steering (4 bánh, bánh lái)** - Khuyến nghị cho xe tự lái:
- Giống xe ô tô thật
- Điều khiển mượt mà hơn
- Phù hợp với lane following

**Differential Drive (2 bánh)** - Đơn giản hơn:
- Dễ build và điều khiển
- Quay tại chỗ được
- Phù hợp với robot nhỏ

### Tắt camera

Chỉnh trong launch file hoặc khi chạy:
```bash
ros2 launch xe_lidar autonomous_drive.launch.py --ros-args \
    -p use_camera:=false
```

### Điều chỉnh tốc độ

Chỉnh các tham số `max_linear_speed` và `max_angular_speed` trong launch file:
- `max_linear_speed`: Tốc độ tiến/lùi (m/s)
- `max_angular_speed`: Tốc độ quay (rad/s)

### Điều chỉnh tham số Ackermann

Trong file `my_controllers_ackermann.yaml`:
- `wheelbase`: Khoảng cách giữa bánh trước và sau (m)
- `track_width`: Khoảng cách giữa 2 bánh trái/phải (m)
- `max_steer_angle`: Góc quay tối đa của bánh lái (rad)

## Lưu ý

- **ROS2 Humble (Ubuntu 22.04)** được khuyến nghị
- Đảm bảo LiDAR và Camera được kết nối đúng trước khi chạy
- Kiểm tra quyền truy cập thiết bị (USB)
- Điều chỉnh tham số phù hợp với môi trường thực tế
- Luôn giám sát robot khi chạy tự động
- Với Ackermann: Đảm bảo `ackermann_steering_controller` đã được cài đặt
- Nếu không có `ackermann_steering_controller`, có thể dùng `twist_to_ackermann_drive` để convert cmd_vel

## Troubleshooting

### Lỗi khi build

**Lỗi: Package không tìm thấy**
```bash
# Đảm bảo đã source ROS2
source /opt/ros/humble/setup.bash

# Kiểm tra package có trong workspace
ros2 pkg list | grep xe_lidar
```

**Lỗi: Dependencies thiếu**
```bash
# Cài đặt lại dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Lỗi: Permission denied khi build**
```bash
# Kiểm tra quyền thư mục
sudo chown -R $USER:$USER ~/ros2_ws
```

**Lỗi: rosdep init failed**
```bash
# Nếu đã init rồi, bỏ qua lỗi này
# Hoặc xóa và init lại:
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
```

**Lỗi: executable 'obstacle_avoidance.py' not found**

Đảm bảo đã làm đúng các bước theo thứ tự:
1. File có quyền thực thi: `chmod +x src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py`
2. Build workspace: `cd ~/ros2_ws && colcon build --symlink-install`
3. Kiểm tra file đã được cài: `ls -la install/xe_lidar/lib/xe_lidar/obstacle_avoidance.py`
4. Source workspace: `source install/setup.bash`
5. Test: `ros2 run xe_lidar obstacle_avoidance.py --help`

### Lỗi khi chạy simulation

**Gazebo không khởi động**
```bash
# Kiểm tra Gazebo đã cài đặt
gazebo --version

# Cài đặt lại nếu cần
sudo apt install --reinstall gazebo ros-humble-gazebo-ros-pkgs
```

**Robot không spawn trong Gazebo**
```bash
# Kiểm tra URDF
cd ~/ros2_ws
source install/setup.bash
xacro src/xe_tu_lai/xe_lidar/description/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

**Lỗi: Could not find a package configuration file**
```bash
# Đảm bảo đã build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Lỗi với Ackermann controller

**Controller không tìm thấy**
```bash
# Kiểm tra controller đã build
ros2 pkg list | grep ackermann

# Nếu không có, build từ source (xem Bước 6 trong phần Cài đặt)
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_controllers.git -b humble
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ackermann_steering_controller
source install/setup.bash
```
