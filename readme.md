# Xe Tự Lái - Hệ Thống Ackermann Steering với ROS2 Jazzy

Hệ thống xe tự lái sử dụng **Camera** (lane following) và **LiDAR** (obstacle avoidance) với ROS2 Jazzy.

## 📋 Yêu cầu hệ thống

- **OS**: Ubuntu 24.04 LTS (khuyến nghị)
- **ROS2**: Jazzy Jalisco
- **Phần cứng**:
  - Raspberry Pi 4/5 hoặc máy tính Linux
  - RPLIDAR A1 (hoặc tương đương)
  - USB Camera
  - Arduino (đã upload code `ackermann_motor_control.ino`)
  - 1 Motor DC + 1 Servo (Ackermann Steering)

---

## 🚀 CÀI ĐẶT

### Bước 1: Cài đặt ROS2 Jazzy

```bash
# Cài đặt aptitude (nếu chưa có)
sudo apt install -y aptitude

# Cập nhật hệ thống
sudo aptitude update && sudo aptitude full-upgrade -y

# Cài đặt locale
sudo aptitude install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Thêm ROS2 repository
sudo aptitude install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Cài đặt ROS2 Jazzy Desktop
sudo aptitude update
sudo aptitude install -y ros-jazzy-desktop

# Cài đặt development tools
sudo aptitude install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Khởi tạo rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Bước 2: Clone và build workspace

```bash
# Tạo workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone project (thay đổi URL nếu cần)
# git clone <your-repo-url> xe_tu_lai
# Hoặc copy project vào đây
cd ~/ros2_ws

# Cài đặt dependencies ROS2 (ĐƠN GIẢN - KHÔNG CẦN cv_bridge!)
sudo aptitude install -y \
    ros-jazzy-twist-mux \
    ros-jazzy-controller-manager \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    python3-opencv \
    python3-numpy \
    python3-pip \
    python3-serial

# Cài Python packages
pip3 install pyserial

# Lưu ý: camera_node.py tự convert OpenCV → ROS Image (KHÔNG CẦN cv_bridge!)

# Cài đặt dependencies từ package.xml
rosdep install --from-paths src --ignore-src -r -y

# QUAN TRỌNG: Cấp quyền thực thi cho scripts TRƯỚC KHI build
chmod +x ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts/*.py

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Thêm vào .bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Kiểm tra scripts đã được install
ls -la install/xe_lidar/libexec/xe_lidar/
ls -la install/xe_lidar/lib/xe_lidar/
```

### Bước 3: Cấp quyền truy cập thiết bị

```bash
# Thêm user vào groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Logout và login lại để áp dụng

# Cấp quyền tạm thời (nếu cần)
sudo chmod 666 /dev/ttyACM0  # Arduino
sudo chmod 666 /dev/ttyUSB0  # LiDAR
sudo chmod 666 /dev/video0   # Camera
```

---

## 🧪 TEST TỪNG PHẦN CỨNG

**⚠️ QUAN TRỌNG**: Phải test từng phần cứng trước khi chạy tự động!

### Test Camera

```bash
cd ~/ros2_ws

# QUAN TRỌNG: Cấp quyền và rebuild trước
chmod +x src/xe_tu_lai/raspberry_pi/xe_lidar/scripts/camera_node.py
colcon build --packages-select xe_lidar
source install/setup.bash

# Chạy camera node (OpenCV - KHÔNG CẦN cv_bridge!)
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Kiểm tra:**
- Terminal hiển thị: "Camera đã mở tại /dev/video0" và "Camera Node đã khởi động! (KHÔNG CẦN cv_bridge)"
- Xem ảnh camera: `ros2 run rqt_image_view rqt_image_view /camera/image_raw`
- Kiểm tra topic: `ros2 topic echo /camera/image_raw --once`

**Tùy chọn:**
```bash
# Thay đổi resolution và FPS
ros2 launch xe_lidar camera.launch.py \
    video_device:=/dev/video0 \
    width:=320 \
    height:=240 \
    fps:=15
```

**Nếu camera bị nhảy hình:**
```bash
# Thử resolution nhỏ hơn
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0 width:=320 height:=240 fps:=15
```

### Test LiDAR

```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy LiDAR node
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

**Kiểm tra:**
- Terminal hiển thị: "RPLidar S/N: ..." và "RPLidar health status : OK"
- Xem dữ liệu: `ros2 topic echo /scan --once`
- Visualize: `rviz2` → Add → LaserScan → Topic: `/scan`

**Nếu không kết nối được:**
- Kiểm tra port: `ls -l /dev/ttyUSB*`
- Kiểm tra quyền: `sudo chmod 666 /dev/ttyUSB0`
- Thử port khác: `serial_port:=/dev/ttyUSB1`

### Test Arduino

```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy Arduino bridge
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

**Kiểm tra:**
- Terminal hiển thị: "✅ Đã kết nối với Arduino tại /dev/ttyACM0"
- Test gửi lệnh thủ công (Terminal 2):
```bash
# Tiến thẳng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Quay trái
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Dừng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

**Kiểm tra:**
- Motor quay khi gửi lệnh tiến (linear.x > 0)
- Servo quay khi gửi lệnh có angular (angular.z != 0)
- Robot dừng khi gửi lệnh dừng

---

## 🚗 CHẠY THẬT VỚI ROS2

Sau khi test xong tất cả phần cứng, chạy hệ thống tự lái:

### Cách 1: Chạy tất cả cùng lúc (Khuyến nghị)

```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy tất cả: Camera + LiDAR + Arduino + Autonomous Drive
ros2 launch xe_lidar autonomous_drive_arduino.launch.py
```

**Hoặc chỉ định port cụ thể:**
```bash
ros2 launch xe_lidar autonomous_drive_arduino.launch.py \
    arduino_serial_port:=/dev/ttyACM0 \
    lidar_serial_port:=/dev/ttyUSB0 \
    video_device:=/dev/video0
```

### Cách 2: Chạy từng phần (Để debug)

**Terminal 1 - Camera:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Terminal 2 - LiDAR:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 3 - Arduino Bridge:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

**Terminal 4 - Autonomous Drive:**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run xe_lidar obstacle_avoidance.py
```

---

## ⚙️ CẤU HÌNH

### Tham số Autonomous Drive

Có thể điều chỉnh khi chạy node:

```bash
ros2 run xe_lidar obstacle_avoidance.py --ros-args \
    -p min_distance:=0.5 \
    -p safe_distance:=0.8 \
    -p max_linear_speed:=0.3 \
    -p max_angular_speed:=1.0 \
    -p front_angle_range:=60 \
    -p use_camera:=true
```

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `min_distance` | 0.5 | Khoảng cách tối thiểu để dừng (m) |
| `safe_distance` | 0.8 | Khoảng cách an toàn để tránh (m) |
| `max_linear_speed` | 0.3 | Tốc độ tối đa (m/s) |
| `max_angular_speed` | 1.0 | Tốc độ quay tối đa (rad/s) |
| `front_angle_range` | 60 | Góc phát hiện phía trước (degrees) |
| `use_camera` | true | Bật/tắt camera lane following |

---

## 🔍 KIỂM TRA HỆ THỐNG

### Xem tất cả topics

```bash
ros2 topic list
```

### Xem dữ liệu từng topic

```bash
# Camera
ros2 topic echo /camera/image_raw --once

# LiDAR
ros2 topic echo /scan --once

# Lệnh điều khiển
ros2 topic echo /cmd_vel
```

### Xem nodes đang chạy

```bash
ros2 node list
```

### Visualize với RViz2

```bash
rviz2
```

Thêm các components:
- **LaserScan**: Topic `/scan`
- **Image**: Topic `/camera/image_raw`
- **TF**: Xem coordinate frames

---

## 🛠️ TROUBLESHOOTING

### Camera không hoạt động

**Lỗi: executable 'camera_node.py' not found**

```bash
# 1. Cấp quyền thực thi cho scripts
chmod +x ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts/camera_node.py

# 2. Rebuild workspace
cd ~/ros2_ws
colcon build --packages-select xe_lidar

# 3. Source lại
source install/setup.bash

# 4. Kiểm tra file đã được install
ls -la install/xe_lidar/libexec/xe_lidar/camera_node.py
ls -la install/xe_lidar/lib/xe_lidar/camera_node.py

# 5. Test lại
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Lưu ý**: `camera_node.py` tự convert OpenCV → ROS Image message, **KHÔNG CẦN cv_bridge**! Chỉ cần `python3-opencv`.

**Lỗi: Không mở được camera**

```bash
# Kiểm tra camera device
ls -l /dev/video*

# Cấp quyền
sudo chmod 666 /dev/video0
sudo usermod -a -G video $USER
# Logout và login lại

# Kiểm tra camera có hoạt động không
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL'); cap.release()"
```

**Camera bị nhảy hình:**
```bash
# Thử resolution nhỏ hơn
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0 width:=320 height:=240 fps:=15

# Hoặc cài đặt GStreamer plugins
sudo aptitude install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    v4l-utils
```

**Lỗi: Rviz "Message Filter dropping message: frame 'camera_link_optical'"**

```bash
# Lỗi này xảy ra vì thiếu TF transform cho camera frame
# Đã được fix trong camera.launch.py (có static_transform_publisher)
# Nếu vẫn lỗi, kiểm tra:

# 1. Kiểm tra transform có được publish không
ros2 run tf2_ros tf2_echo base_link camera_link_optical

# 2. Nếu không có base_link, dùng frame khác hoặc xem ảnh bằng rqt_image_view
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# 3. Hoặc publish transform thủ công (Terminal 2):
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link camera_link_optical
```

**Lưu ý**: 
- Project này dùng `camera_node.py` (OpenCV) - **KHÔNG CẦN cv_bridge**!
- `camera_node.py` tự convert OpenCV → ROS Image message, chỉ cần `python3-opencv`
- Không cần cài `v4l2_camera` package
- Launch file đã có static transform publisher cho rviz

### LiDAR không hoạt động

```bash
# Kiểm tra port
ls -l /dev/ttyUSB*

# Cấp quyền
sudo chmod 666 /dev/ttyUSB0

# Kiểm tra kết nối
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200
```

**Lỗi: Rviz "Message Filter dropping message: frame 'laser_frame'"**

```bash
# Lỗi này xảy ra vì thiếu TF transform cho laser frame
# Đã được fix trong rplidar.launch.py (có static_transform_publisher)
# Nếu vẫn lỗi, kiểm tra:

# 1. Kiểm tra transform có được publish không
ros2 run tf2_ros tf2_echo base_link laser_frame

# 2. Hoặc publish transform thủ công (Terminal 2):
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link laser_frame

# 3. Kiểm tra topic /scan có dữ liệu không
ros2 topic echo /scan --once
```

### Arduino không nhận lệnh

**Lỗi: executable 'arduino_bridge.py' not found**

```bash
# 1. Cấp quyền thực thi cho scripts
chmod +x ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts/*.py

# 2. Rebuild workspace
cd ~/ros2_ws
colcon build --symlink-install

# 3. Source lại
source install/setup.bash

# 4. Kiểm tra file đã được install
ls -la install/xe_lidar/libexec/xe_lidar/arduino_bridge.py
ls -la install/xe_lidar/lib/xe_lidar/arduino_bridge.py

# 5. Test lại
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

**Kiểm tra Serial Monitor trên Arduino IDE:**
- Phải thấy "READY" khi khởi động

**Kiểm tra quyền:**
```bash
sudo chmod 666 /dev/ttyACM0
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Robot không di chuyển

```bash
# Kiểm tra cmd_vel có được publish không
ros2 topic echo /cmd_vel

# Kiểm tra Arduino bridge có chạy không
ros2 node list | grep arduino

# Kiểm tra motor driver và nguồn
```

---

## 📚 TÀI LIỆU THAM KHẢO

- **Arduino Code**: Xem `arduino/README.md`
- **Sửa lỗi Camera**: Xem `FIX_CAMERA.md`
- **Cài đặt Dependencies**: Xem `INSTALL_DEPENDENCIES.md`
- **ROS2 Jazzy**: [https://docs.ros.org/en/jazzy/](https://docs.ros.org/en/jazzy/)

---

## ✅ CHECKLIST TRƯỚC KHI CHẠY

- [ ] Ubuntu 24.04 + ROS2 Jazzy đã cài đặt
- [ ] Workspace đã build: `colcon build --symlink-install`
- [ ] Đã source: `source install/setup.bash`
- [ ] NumPy < 2.0: `pip3 list | grep numpy`
- [ ] Camera test OK
- [ ] LiDAR test OK
- [ ] Arduino test OK
- [ ] Quyền truy cập serial/video: `groups $USER`

---

**Chúc bạn thành công! 🚗💨**

