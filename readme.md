# Hệ Thống Xe Tự Lái 4 Bánh - Ackermann Steering

> **Hệ thống xe tự lái thông minh sử dụng Ackermann Steering (4 bánh lái như ô tô thật)**
> Kết hợp **Camera** (lane following) và **LiDAR** (obstacle avoidance) với ROS2 Humble

**Repository**: [https://github.com/TUPM96/xe_tu_lai](https://github.com/TUPM96/xe_tu_lai)

---

## 🎯 Tính năng chính

### ✅ Điều khiển Ackermann Steering (4 bánh)
- **4 bánh xe**: 2 bánh trước có khả năng lái, 2 bánh sau cố định
- **Giống xe ô tô thật**: Bánh trái và phải có góc lái khác nhau (Ackermann geometry)
- **Điều khiển mượt mà**: Phù hợp cho lane following và tự lái
- **Giới hạn góc lái**: max ±30° (0.5236 rad) để an toàn

### 🎥 Camera - Lane Following (Đi theo làn đường)
- **Phát hiện vạch kẻ đường**: Sử dụng OpenCV (HSV + Canny + HoughLinesP)
- **Tính toán offset**: Xác định vị trí xe so với giữa đường
- **Điều khiển chính xác**:
  - Xe lệch phải → quay trái tự động
  - Xe lệch trái → quay phải tự động
- **Xử lý đúng hệ tọa độ ảnh**: Slope classification chính xác

### 🛑 LiDAR - Obstacle Avoidance (Tránh vật cản)
- **Quét 360°**: RPLIDAR A1 phát hiện vật cản xung quanh
- **Vùng phát hiện**: 60° phía trước (±30° từ trục xe)
- **Quyết định thông minh**:
  - Vật cản bên trái → quay phải
  - Vật cản bên phải → quay trái
  - Vật cản chặn đường → lùi lại và quay
- **Khoảng cách an toàn**: 0.8m (có thể điều chỉnh)

### 🧠 Logic điều khiển 2 mức độ ưu tiên
```
┌─────────────────────────────────────────┐
│  PRIORITY 1 (CAO) - SAFETY              │
│  LiDAR Obstacle Avoidance               │
│  → Có vật cản? Tránh ngay lập tức!     │
└──────────────┬──────────────────────────┘
               │ Không có vật cản
               ↓
┌─────────────────────────────────────────┐
│  PRIORITY 2 (THẤP) - NAVIGATION         │
│  Camera Lane Following                  │
│  → Phát hiện làn đường? Đi giữa làn!   │
│  → Không thấy làn? Đi thẳng!           │
└─────────────────────────────────────────┘
```

---

## 🔧 Các lỗi đã sửa trong phiên bản này

### ❌ Lỗi 1: Logic điều khiển góc lái SAI (NGHIÊM TRỌNG!)
**Trước đây**:
```python
cmd.angular.z = -self.lane_center_offset * max_angular_speed  # SAI!
# offset > 0 (lệch phải) → angular.z < 0 (quay phải) → Xe lệch phải càng xa!
```

**Đã sửa**:
```python
cmd.angular.z = self.lane_center_offset * max_angular_speed  # ĐÚNG!
# offset > 0 (lệch phải) → angular.z > 0 (quay trái) → Xe quay về giữa! ✅
```

### ❌ Lỗi 2: Slope classification SAI (phân loại vạch trái/phải)
**Trước đây**:
```python
if slope < -0.2 and mid_x < center_x:  # SAI! Vạch trái không có slope âm
    left_lines.append(line)
```

**Đã sửa**:
```python
# ĐÚNG: Trong hệ tọa độ ảnh (Y tăng từ trên xuống):
# - Vạch TRÁI: từ trên-trái xuống dưới-phải → slope DƯƠNG
# - Vạch PHẢI: từ trên-phải xuống dưới-trái → slope ÂM
if slope > 0.2 and mid_x < center_x:  # ĐÚNG! ✅
    left_lines.append(line)
```

### ❌ Lỗi 3: Camera pitch angle quá lớn
**Trước đây**:
```xml
<origin xyz="0.0 0 1.2" rpy="0 1.4 0"/>
<!-- pitch = 1.4 rad ≈ 80° → Nhìn gần như thẳng xuống, chỉ thấy trước mặt xe vài cm! -->
```

**Đã sửa**:
```xml
<origin xyz="0.2 0 0.25" rpy="0 0.4 0"/>
<!-- pitch = 0.4 rad ≈ 23° → Nhìn xuống đường vừa phải, thấy xa hơn! ✅ -->
```

### ❌ Lỗi 4: Thiếu giới hạn góc lái cho Ackermann
**Đã thêm**:
```python
# Giới hạn angular velocity theo max_steer_angle (~30°)
max_angular_for_ackermann = self.max_angular_speed * 0.9
cmd.angular.z = max(-max_angular_for_ackermann,
                   min(max_angular_for_ackermann, desired_angular))
```

### ❌ Lỗi 5: Comments và priority logic sai
- ✅ Đã sửa tất cả comments cho đúng với logic thực tế
- ✅ Làm rõ LiDAR có priority cao hơn Camera

---

## 📋 Yêu cầu hệ thống

### Hệ điều hành
- **Ubuntu 22.04 LTS** (khuyến nghị mạnh mẽ)
- Ubuntu 20.04 với ROS2 Foxy (cần điều chỉnh)

### Phần mềm
- **ROS2 Humble Hawksbill** (bắt buộc)
- Python 3.10+
- OpenCV (`python3-opencv`)
- NumPy (`numpy<2.0` - quan trọng!)
- Gazebo 11 (cho simulation)
- `ackermann_steering_controller` (ROS2 control)

### Phần cứng (Robot thật)
- Raspberry Pi 4 (4GB RAM trở lên) hoặc máy tính Linux
- **RPLIDAR A1** hoặc tương đương (360° laser scanner)
- **USB Camera** (640x480 trở lên)
- **Khung xe 4 bánh Ackermann**:
  - 2 bánh trước với servo lái (góc lái ±30°)
  - 2 bánh sau cố định với motor
  - Wheelbase (khoảng cách trước-sau): ~40cm
  - Track width (khoảng cách trái-phải): ~28cm

### Yêu cầu tối thiểu
- **CPU**: 4 cores (2 cores tối thiểu)
- **RAM**: 8GB (4GB tối thiểu)
- **Dung lượng**: 30GB trống
- **GPU**: Không bắt buộc (khuyến nghị cho Gazebo)

---

## 🚀 Cài đặt

### Bước 1: Cài đặt ROS2 Humble

```bash
# Cập nhật hệ thống
sudo apt update && sudo apt upgrade -y

# Cài đặt locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Thêm ROS2 repository
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Cài đặt ROS2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Cài đặt development tools
sudo apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Khởi tạo rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Bước 2: Clone repository

```bash
# Tạo workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone project
git clone https://github.com/TUPM96/xe_tu_lai.git
cd ~/ros2_ws
```

### Bước 3: Cài đặt dependencies

```bash
# Dependencies cho xe tự lái
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-ackermann-msgs \
    ros-humble-xacro \
    python3-opencv \
    python3-numpy \
    python3-pip \
    gazebo

# QUAN TRỌNG: NumPy phải < 2.0 (cv_bridge chưa hỗ trợ NumPy 2.x)
pip3 install "numpy<2.0"

# Cài đặt dependencies từ package.xml
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Bước 4: Cài đặt Ackermann Steering Controller

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_controllers.git -b humble

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build ackermann_steering_controller
colcon build --packages-select ackermann_steering_controller
source install/setup.bash
```

### Bước 5: Build project

```bash
cd ~/ros2_ws

# Cấp quyền thực thi cho Python scripts
chmod +x src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py

# Build toàn bộ workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Thêm vào .bashrc để tự động
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Bước 6: Kiểm tra cài đặt

```bash
# Kiểm tra package
ros2 pkg list | grep xe_lidar

# Kiểm tra node có chạy được không
ros2 run xe_lidar obstacle_avoidance.py --ros-args --help

# Kiểm tra ackermann controller
ros2 pkg list | grep ackermann_steering_controller
```

---

## 🎮 Sử dụng

### 1. Chạy Simulation (Gazebo) - Khuyến nghị để test

```bash
# Terminal 1: Chạy simulation với Ackermann steering
cd ~/ros2_ws
source install/setup.bash
ros2 launch xe_lidar simulation_ackermann.launch.py

# Robot sẽ tự động xuất hiện trong Gazebo và bắt đầu tự lái!
# Gazebo world mặc định: road_map.world (đường 2 làn với vật cản 2 bên)
```

**Chọn world khác**:
```bash
# World test với nhiều vật cản
ros2 launch xe_lidar simulation_ackermann.launch.py world:=test_map.world

# World mê cung (thử thách!)
ros2 launch xe_lidar simulation_ackermann.launch.py world:=maze_map.world

# World trống (tự do)
ros2 launch xe_lidar simulation_ackermann.launch.py world:=empty.world
```

### 2. Chạy trên Robot thật

**Terminal 1 - Khởi động robot hardware**:
```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy với Ackermann steering (4 bánh)
ros2 launch xe_lidar launch_robot_ackermann.launch.py
```

**Terminal 2 - Khởi động autonomous drive**:
```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy autonomous drive node
ros2 launch xe_lidar autonomous_drive.launch.py
```

### 3. Chạy từng thành phần riêng lẻ (Debug)

**LiDAR**:
```bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

**Camera**:
```bash
sudo chmod 666 /dev/video0
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Autonomous Drive**:
```bash
ros2 run xe_lidar obstacle_avoidance.py
```

### 4. Điều khiển thủ công (Tùy chọn)

```bash
# Cài đặt teleop (nếu chưa có)
sudo apt install ros-humble-teleop-twist-keyboard

# Chạy
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## ⚙️ Cấu hình

### Tham số Autonomous Drive

Có thể điều chỉnh trong `launch/autonomous_drive.launch.py` hoặc khi chạy:

```bash
ros2 run xe_lidar obstacle_avoidance.py --ros-args \
    -p min_distance:=0.5 \
    -p safe_distance:=0.8 \
    -p max_linear_speed:=0.3 \
    -p max_angular_speed:=1.0 \
    -p front_angle_range:=60 \
    -p max_steer_angle:=0.5236 \
    -p use_camera:=true \
    -p debug_camera:=false
```

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `min_distance` | 0.5 | Khoảng cách tối thiểu để dừng (m) |
| `safe_distance` | 0.8 | Khoảng cách an toàn để bắt đầu tránh (m) |
| `max_linear_speed` | 0.3 | Tốc độ tiến/lùi tối đa (m/s) |
| `max_angular_speed` | 1.0 | Tốc độ quay tối đa (rad/s) |
| `front_angle_range` | 60 | Góc phát hiện phía trước (degrees) |
| `max_steer_angle` | 0.5236 | Góc lái tối đa (rad ≈ 30°) |
| `use_camera` | true | Bật/tắt camera lane following |
| `debug_camera` | false | Hiển thị debug output camera |

### Tham số Ackermann Controller

File: `config/my_controllers_ackermann.yaml`

```yaml
ackermann_steering_controller:
  ros__parameters:
    # Geometry (điều chỉnh theo robot thật)
    wheel_radius: 0.034        # Bán kính bánh (m)
    wheelbase: 0.4             # Khoảng cách bánh trước-sau (m)
    track_width: 0.28          # Khoảng cách bánh trái-phải (m)

    # Steering limits
    max_steer_angle: 0.5236    # ~30 degrees

    # Velocity limits
    linear.x.max_velocity: 1.0
    angular.z.max_velocity: 1.0
```

**Lưu ý**: Phải đo chính xác `wheelbase` và `track_width` từ robot thật!

### Camera Lane Detection Tuning

Trong `scripts/obstacle_avoidance.py`:

```python
# HSV white color range (điều chỉnh nếu vạch đường không trắng)
lower_white = np.array([0, 0, 200])    # Ngưỡng dưới
upper_white = np.array([180, 30, 255])  # Ngưỡng trên

# Canny edge detection
edges = cv2.Canny(blurred, 50, 150)    # Threshold: 50, 150

# HoughLinesP parameters
lines = cv2.HoughLinesP(
    edges,
    1,
    np.pi/180,
    threshold=30,      # Số điểm tối thiểu để tạo thành đường
    minLineLength=20,  # Độ dài tối thiểu của đường (pixel)
    maxLineGap=15      # Khoảng cách tối đa giữa các điểm (pixel)
)

# ROI (Region of Interest) - vùng dưới ảnh
roi_top = int(height * 0.4)  # Bắt đầu từ 40% chiều cao
```

---

## 🔍 Monitoring và Debug

### 1. Xem dữ liệu LiDAR

```bash
# Echo topic
ros2 topic echo /scan

# Visualize trong RViz2
rviz2
# Add → LaserScan → Topic: /scan
```

### 2. Xem camera feed

```bash
# Cài đặt image view (nếu chưa có)
sudo apt install ros-humble-rqt-image-view

# Xem camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### 3. Xem lệnh điều khiển

```bash
ros2 topic echo /cmd_vel
```

### 4. Xem logs

```bash
# Xem log của autonomous drive node
ros2 topic echo /rosout | grep autonomous_drive

# Hoặc chạy với debug level
ros2 run xe_lidar obstacle_avoidance.py --ros-args --log-level debug
```

### 5. RViz2 visualization

```bash
rviz2
```

Thêm các components:
- **RobotModel**: Xem 3D model xe
- **LaserScan** (`/scan`): Xem dữ liệu LiDAR
- **Image** (`/camera/image_raw`): Xem camera
- **TF**: Xem coordinate frames

---

## 🛠️ Troubleshooting

### ❌ Lỗi: `executable 'obstacle_avoidance.py' not found`

```bash
# Kiểm tra file có quyền thực thi
ls -la ~/ros2_ws/src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py

# Cấp quyền
chmod +x ~/ros2_ws/src/xe_tu_lai/xe_lidar/scripts/obstacle_avoidance.py

# Build lại
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### ❌ Lỗi: LiDAR không hoạt động

```bash
# 1. Kiểm tra device
ls -l /dev/ttyUSB*

# 2. Cấp quyền
sudo chmod 666 /dev/ttyUSB0
sudo usermod -a -G dialout $USER
# (Logout và login lại)

# 3. Test LiDAR
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200
```

### ❌ Lỗi: Camera không hoạt động

```bash
# 1. Kiểm tra device
ls -l /dev/video*

# 2. Cấp quyền
sudo chmod 666 /dev/video0
sudo usermod -a -G video $USER

# 3. Test camera
ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -p video_device:=/dev/video0
```

### ❌ Lỗi: Xe không di chuyển (simulation)

```bash
# Kiểm tra Gazebo đang chạy
ps aux | grep gazebo

# Kiểm tra cmd_vel có được publish không
ros2 topic echo /cmd_vel

# Kiểm tra controller
ros2 control list_controllers
```

### ❌ Lỗi: `ackermann_steering_controller` not found

```bash
# Cài đặt từ source (xem Bước 4)
cd ~/ros2_ws/src
git clone https://github.com/ros-controls/ros2_controllers.git -b humble
cd ~/ros2_ws
colcon build --packages-select ackermann_steering_controller
source install/setup.bash
```

### ❌ Lỗi: NumPy version incompatible

```bash
# cv_bridge chỉ hỗ trợ NumPy < 2.0
pip3 install --upgrade "numpy<2.0"
```

### ❌ Xe đi lệch hoặc không về giữa làn đường

✅ **Đã sửa trong version này!**
- Kiểm tra đã update code mới nhất chưa
- Logic `angular.z` đã được sửa (bỏ dấu trừ)
- Slope classification đã được sửa đúng

---

## 📐 Nguyên lý hoạt động

### 1. Kiến trúc hệ thống

```
┌──────────────────────────────────────────────────────┐
│                    ROS2 Nodes                         │
├──────────────────────────────────────────────────────┤
│                                                       │
│  ┌─────────────┐         ┌──────────────┐           │
│  │   Camera    │────────→│              │           │
│  │   Node      │ img_raw │              │           │
│  └─────────────┘         │  Autonomous  │           │
│                          │    Drive     │  cmd_vel  │
│  ┌─────────────┐         │    Node      │───────────┤───→ Controller
│  │   LiDAR     │────────→│              │           │
│  │   Node      │  scan   │              │           │
│  └─────────────┘         └──────────────┘           │
│                                                       │
└──────────────────────────────────────────────────────┘
                                    ↓
                    ┌───────────────────────────────┐
                    │ Ackermann Steering Controller │
                    └───────────────────────────────┘
                                    ↓
            ┌───────────────┬───────────────┬───────────────┐
            ↓               ↓               ↓               ↓
     ┌──────────┐    ┌──────────┐   ┌──────────┐   ┌──────────┐
     │  Front   │    │  Front   │   │   Rear   │   │   Rear   │
     │  Left    │    │  Right   │   │   Left   │   │  Right   │
     │  Wheel   │    │  Wheel   │   │  Wheel   │   │  Wheel   │
     │ + Steer  │    │ + Steer  │   │  (Fixed) │   │  (Fixed) │
     └──────────┘    └──────────┘   └──────────┘   └──────────┘
```

### 2. Camera Lane Detection

**Quy trình xử lý**:
```
Raw Image (640x480)
    ↓
[ROI Selection] - Chọn vùng dưới ảnh (40% → 100% height)
    ↓
[HSV Conversion] - Chuyển sang HSV color space
    ↓
[White Mask] - Tạo mask cho màu trắng (vạch kẻ đường)
    ↓
[Gaussian Blur] - Làm mịn ảnh
    ↓
[Canny Edge Detection] - Phát hiện cạnh
    ↓
[HoughLinesP] - Phát hiện đường thẳng
    ↓
[Slope Classification] - Phân loại vạch trái/phải
    ↓
[Calculate Center] - Tính giữa đường
    ↓
[Calculate Offset] - Tính độ lệch từ giữa (-1 đến +1)
    ↓
lane_center_offset → Dùng cho điều khiển
```

**Hệ tọa độ ảnh**:
```
(0,0) ────────► X (width)
  │
  │     /        \      Camera view từ trên nhìn xuống
  │    /          \
  │   / (LEFT)  (RIGHT) \
  │  ╱                  ╲
  ▼ Y
(height)

- Vạch TRÁI: slope > 0 (y↑, x↑)
- Vạch PHẢI: slope < 0 (y↑, x↓)
```

### 3. LiDAR Obstacle Detection

```
360° Laser Scan
    ↓
[Filter Front Region] - Chỉ lấy 60° phía trước (±30°)
    ↓
[Find Closest Obstacle] - Tìm vật cản gần nhất
    ↓
[Check Safe Distance] - So sánh với ngưỡng 0.8m
    ↓
IF obstacle < safe_distance:
    [Determine Direction] - Xác định vật cản bên trái/phải/giữa
    ↓
    obstacle_detected = True
    obstacle_direction = -1 (left) / 0 (center) / 1 (right)
```

### 4. Ackermann Steering Geometry

```
        Front Axle
         /      \
        /        \    ← Góc lái khác nhau
       /          \
    ●─────────────●  Front wheels (steering)
    │             │
    │             │  ← Wheelbase (0.4m)
    │             │
    ●─────────────●  Rear wheels (fixed)

    ↑           ↑
    Track Width (0.28m)
```

**Công thức Ackermann**:
```
δ_inner = atan(L / (R - W/2))    # Góc bánh trong
δ_outer = atan(L / (R + W/2))    # Góc bánh ngoài

Trong đó:
- L: wheelbase
- W: track_width
- R: bán kính quay
- δ: góc lái
```

**ROS2 `ackermann_steering_controller` tự động tính toán:**
- Input: `cmd_vel` (linear.x, angular.z)
- Output:
  - Góc lái cho 2 bánh trước (tự động tính theo Ackermann)
  - Vận tốc cho 4 bánh (tính theo công thức động học)

### 5. Control Logic Flow

```python
def control_loop():
    # PRIORITY 1: LiDAR (Safety)
    if obstacle_detected:
        if obstacle_direction == 0:  # Center/Both sides
            cmd.linear.x = -0.15  # Reverse
            cmd.angular.z = 0.8   # Turn right
        elif obstacle_direction < 0:  # Left side
            cmd.linear.x = 0.18   # Slow forward
            cmd.angular.z = -0.7  # Turn right (away from obstacle)
        else:  # Right side
            cmd.linear.x = 0.18   # Slow forward
            cmd.angular.z = 0.7   # Turn left (away from obstacle)

    # PRIORITY 2: Camera (Navigation)
    elif lane_detected:
        cmd.linear.x = 0.3  # Full speed

        # QUAN TRỌNG: Công thức ĐÚNG (đã sửa lỗi!)
        desired_angular = lane_center_offset * max_angular_speed * 0.8
        # offset > 0 (lệch phải) → angular.z > 0 (quay trái) ✅
        # offset < 0 (lệch trái) → angular.z < 0 (quay phải) ✅

        # Giới hạn góc lái
        cmd.angular.z = clamp(desired_angular, -max_angular, max_angular)

    # No lane detected
    else:
        cmd.linear.x = 0.3  # Go straight
        cmd.angular.z = 0.0

    publish(cmd_vel)
```

---

## 📚 Tài liệu tham khảo

### ROS2 Documentation
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Ackermann Steering Controller](https://control.ros.org/humble/doc/ros2_controllers/ackermann_steering_controller/doc/userdoc.html)
- [Gazebo ROS2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

### Ackermann Steering
- [Ackermann Steering Geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
- Paper: "Kinematic Models for Wheeled Mobile Robots"

### Computer Vision
- [OpenCV Lane Detection Tutorial](https://opencv.org/)
- [Canny Edge Detection](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html)
- [Hough Line Transform](https://docs.opencv.org/4.x/d9/db0/tutorial_hough_lines.html)

---

## 🤝 Đóng góp

Contributions are welcome! Hãy:
1. Fork repository
2. Tạo branch mới (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Tạo Pull Request

---

## 📝 License

Project này được phát hành dưới license [MIT License](LICENSE).

---

## 🐛 Báo lỗi

Nếu gặp lỗi, hãy tạo issue tại:
- **GitHub Issues**: [https://github.com/TUPM96/xe_tu_lai/issues](https://github.com/TUPM96/xe_tu_lai/issues)

Khi báo lỗi, vui lòng cung cấp:
- Ubuntu version
- ROS2 version (`ros2 --version`)
- Log output (`ros2 launch ... --log-level debug`)
- Screenshot/video nếu có thể

---

## ✅ Checklist trước khi chạy

- [ ] Ubuntu 22.04 + ROS2 Humble đã cài đặt
- [ ] Workspace đã build: `colcon build --symlink-install`
- [ ] Đã source: `source install/setup.bash`
- [ ] NumPy < 2.0: `pip3 list | grep numpy`
- [ ] Ackermann controller đã có: `ros2 pkg list | grep ackermann`
- [ ] File script có quyền thực thi: `ls -la scripts/`
- [ ] LiDAR và Camera được kết nối (robot thật)
- [ ] Quyền truy cập serial/video: `groups $USER`

---

## 🎉 Kết luận

Hệ thống xe tự lái này đã được sửa lại hoàn toàn với:
- ✅ Logic điều khiển Ackermann steering ĐÚNG
- ✅ Slope classification cho lane detection ĐÚNG
- ✅ Camera angle phù hợp
- ✅ Giới hạn góc lái an toàn
- ✅ Priority logic rõ ràng
- ✅ Documentation chi tiết

**Sẵn sàng để chạy và test!** 🚗💨

---

**Author**: TUPM96
**Repository**: [https://github.com/TUPM96/xe_tu_lai](https://github.com/TUPM96/xe_tu_lai)
**Last Updated**: 2025-12-03
