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
# Cập nhật hệ thống
sudo apt update && sudo apt full-upgrade -y

# Cài đặt locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Thêm Universe repository (bắt buộc cho ROS)
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y

# Thêm ROS 2 repository (CÁCH ĐÚNG cho Ubuntu 24.04)
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
> /etc/apt/sources.list.d/ros2-latest.list'

# Cài đặt ROS 2 Jazzy Desktop
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Cài đặt development tools (KHÔNG dùng rosdep)
sudo apt install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool

# Setup ROS environment
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Fix rviz2 trên Raspberry Pi (software rendering)
echo 'alias rviz2="LIBGL_ALWAYS_SOFTWARE=1 rviz2"' >> ~/.bashrc

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
sudo apt install -y \
    ros-jazzy-twist-mux \
    ros-jazzy-controller-manager \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    python3-opencv \
    python3-numpy \
    python3-pip \
    python3-serial

# Lưu ý: camera_node.py tự convert OpenCV → ROS Image (KHÔNG CẦN cv_bridge!)

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

### Test Servo (Bánh lái)

Script test servo điều khiển bánh lái với các góc quay khác nhau.

```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts

# Cấp quyền thực thi (chỉ cần làm 1 lần)
chmod +x test_servo.py

# Chạy test servo
python3 test_servo.py /dev/ttyACM0
```

**Chế độ test:**
1. **Test tự động**: Servo sẽ tự động quay qua các góc: trái nhẹ, trái mạnh, phải nhẹ, phải mạnh, tối đa
2. **Test thủ công**: Điều khiển servo bằng bàn phím:
   - `a`: Quay trái
   - `d`: Quay phải
   - `c`: Về vị trí giữa (center)
   - `w/s`: Tăng/giảm bước góc quay
   - `q`: Thoát

**Kết quả mong đợi:**
- Servo quay mượt qua các góc
- Không có tiếng kêu bất thường
- Góc quay đối xứng trái/phải

### Test Motor DC (Tiến/Lùi)

Script test motor DC điều khiển tiến/lùi với các tốc độ khác nhau.

```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts

# Cấp quyền thực thi (chỉ cần làm 1 lần)
chmod +x test_motor.py

# Chạy test motor
python3 test_motor.py /dev/ttyACM0
```

**⚠️ CẢNH BÁO**: Đặt xe lên giá đỡ hoặc nơi an toàn trước khi test motor!

**Chế độ test:**
1. **Test tự động**: Motor sẽ chạy qua các tốc độ: 30%, 50%, 70%, 100% (tiến và lùi)
2. **Test thủ công**: Điều khiển motor bằng bàn phím:
   - `w`: Tiến / Tăng tốc
   - `s`: Lùi / Giảm tốc
   - `x`: Dừng khẩn cấp (Emergency Stop)
   - `+/-`: Tăng/giảm bước tốc độ
   - `q`: Thoát
3. **Test Ramp**: Tăng/giảm tốc độ từ từ từ 0% → 100% → 0%

**Kết quả mong đợi:**
- Motor quay đúng chiều (tiến/lùi)
- Tốc độ tăng/giảm theo lệnh
- Motor dừng ngay khi nhận lệnh dừng

### Test Kết hợp Servo + Motor (ROS2)

Test điều khiển cả servo và motor qua ROS2 topic `/cmd_vel`:

```bash
cd ~/ros2_ws
source install/setup.bash

# Terminal 1: Chạy Arduino bridge
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0

# Terminal 2: Gửi lệnh test
# Tiến thẳng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -1

# Tiến + quay trái
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}" -1

# Tiến + quay phải
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}" -1

# Lùi thẳng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -1

# Lùi + quay trái (servo đảo chiều so với tiến)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}" -1

# Dừng
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" -1
```

**Giải thích tham số:**
- `linear.x`: Tốc độ tiến/lùi (m/s). Dương = tiến, âm = lùi
- `angular.z`: Tốc độ quay (rad/s). Dương = trái, âm = phải

**Lưu ý về Ackermann Steering:**
- Khi `linear.x = 0`, servo sẽ không quay (vì Ackermann steering cần tốc độ để tính góc lái)
- Khi lùi, chiều quay của servo sẽ đảo ngược để xe lùi đúng hướng

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
- Xem ảnh camera (cài rqt_image_view nếu chưa có):
  ```bash
  # Cài đặt rqt_image_view (chỉ cần cài 1 lần)
  sudo apt install -y ros-jazzy-rqt-image-view
  
  # Xem ảnh camera
  ros2 run rqt_image_view rqt_image_view /camera/image_raw
  
  # Xem ảnh debug (có vẽ lane và hướng lái)
  ros2 run rqt_image_view rqt_image_view /camera/image_debug
  ```
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

# Chạy tất cả bằng script
ros2 run xe_lidar start_all.py

# Hoặc chỉ định port cụ thể
ros2 run xe_lidar start_all.py \
    --lidar-port /dev/ttyUSB0 \
    --arduino-port /dev/ttyACM0 \
    --camera-device /dev/video0
```

**Hoặc dùng launch file:**
```bash
ros2 launch xe_lidar autonomous_drive_arduino.launch.py \
    arduino_serial_port:=/dev/ttyACM0 \
    lidar_serial_port:=/dev/ttyUSB0 \
    video_device:=/dev/video0
```

### Cách 2: Chạy từng script riêng lẻ (Khuyến nghị để debug)

Mỗi script chạy trong 1 terminal riêng:

**Terminal 1 - Robot Description (URDF/TF):**
```bash
ros2 run xe_lidar start_robot_description.py
```

**Terminal 2 - LiDAR:**
```bash
ros2 run xe_lidar start_lidar.py --port /dev/ttyUSB0
```

**Terminal 3 - Camera:**
```bash
# Camera Full HD 1920x1080 (mặc định)
ros2 run xe_lidar start_camera.py --device /dev/video0

# Hoặc chọn resolution khác
ros2 run xe_lidar start_camera.py --device /dev/video0 --width 1280 --height 720
ros2 run xe_lidar start_camera.py --device /dev/video0 --width 640 --height 480
```

**Terminal 4 - Arduino Bridge:**
```bash
ros2 run xe_lidar start_arduino.py --port /dev/ttyACM0
```

**Terminal 5 - Autonomous Drive:**
```bash
ros2 run xe_lidar start_autonomous.py

# Hoặc với options
ros2 run xe_lidar start_autonomous.py --max-speed 0.5 --safe-distance 1.0
ros2 run xe_lidar start_autonomous.py --no-camera  # Không dùng camera
```

### Cách 3: Chạy bằng launch files (Cách cũ)

**Terminal 1 - Camera:**
```bash
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Terminal 2 - LiDAR:**
```bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 3 - Arduino Bridge:**
```bash
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

**Terminal 4 - Autonomous Drive:**
```bash
ros2 run xe_lidar obstacle_avoidance.py
```

### Cách 5: Chạy Lane Detection riêng (Để test/debug)

Lane Detection node chạy độc lập, chỉ xử lý camera và publish kết quả:

```bash
cd ~/ros2_ws
source install/setup.bash

# Chạy Lane Detection riêng
ros2 run xe_lidar start_lane_detection.py

# Hoặc với options
ros2 run xe_lidar start_lane_detection.py --full-image      # Dùng full ảnh (không crop)
ros2 run xe_lidar start_lane_detection.py --camera-topic /camera/image_raw
ros2 run xe_lidar start_lane_detection.py --no-debug        # Tắt debug image
ros2 run xe_lidar start_lane_detection.py --roi-top 0.5     # ROI từ 50% chiều cao
ros2 run xe_lidar start_lane_detection.py --lane-width 250  # Khoảng cách vạch (pixels)
```

**Topics được publish:**

| Topic | Type | Mô tả |
|-------|------|-------|
| `/lane_detection/image_debug` | Image | Ảnh debug có vẽ vạch |
| `/lane_detection/offset` | Float32 | Offset từ giữa (-1 đến 1) |
| `/lane_detection/detected` | Bool | Có phát hiện lane không |

**Xem kết quả:**
```bash
# Xem ảnh debug
ros2 run rqt_image_view rqt_image_view /lane_detection/image_debug

# Xem offset
ros2 topic echo /lane_detection/offset

# Xem detected flag
ros2 topic echo /lane_detection/detected
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

### Debug từng topic

#### 1. Camera Topics

```bash
# Xem ảnh camera gốc (chỉ hiển thị metadata, không hiển thị ảnh)
ros2 topic echo /camera/image_raw --once | head -20

# Kiểm tra camera có đang publish không (tần số)
ros2 topic hz /camera/image_raw

# Xem thông tin camera (intrinsic/extrinsic parameters)
ros2 topic echo /camera/camera_info --once

# Xem ảnh debug (có vẽ lane detection và hướng lái)
ros2 topic echo /camera/image_debug --once | head -20

# Kiểm tra debug image có đang publish không
ros2 topic hz /camera/image_debug

# Xem ảnh trực tiếp (cần cài rqt_image_view)
# Cài đặt: sudo apt install -y ros-jazzy-rqt-image-view
ros2 run rqt_image_view rqt_image_view /camera/image_raw
ros2 run rqt_image_view rqt_image_view /camera/image_debug
```

#### 2. LiDAR Topics

```bash
# Xem dữ liệu LiDAR (scan)
ros2 topic echo /scan --once | head -50

# Kiểm tra LiDAR có đang publish không (tần số)
ros2 topic hz /scan

# Xem thông tin topic (loại message, số subscribers/publishers)
ros2 topic info /scan
```

#### 3. Điều khiển Topics

```bash
# Xem lệnh điều khiển đang được gửi (cmd_vel)
ros2 topic echo /cmd_vel

# Kiểm tra tần số publish cmd_vel
ros2 topic hz /cmd_vel

# Xem thông tin topic
ros2 topic info /cmd_vel

# Gửi lệnh điều khiển thủ công (test)
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

#### 4. Odometry Topics (từ Arduino Bridge)

```bash
# Xem odometry (vị trí và vận tốc của robot)
ros2 topic echo /odom

# Kiểm tra tần số publish
ros2 topic hz /odom

# Xem thông tin topic
ros2 topic info /odom
```

#### 5. Robot Description & TF Topics

```bash
# Xem robot description (URDF)
ros2 topic echo /robot_description --once | head -100

# Xem joint states (cần cho RSP)
ros2 topic echo /joint_states

# Kiểm tra tần số publish joint_states
ros2 topic hz /joint_states

# Xem TF transforms
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo odom base_link

# Xem tất cả frames trong TF tree
ros2 run tf2_ros tf2_monitor

# Tạo file PDF của TF tree
ros2 run tf2_tools view_frames
# File frames.pdf sẽ được tạo ra trong thư mục hiện tại
```

#### 6. Kiểm tra tất cả topics cùng lúc

```bash
# Xem danh sách tất cả topics
ros2 topic list

# Xem tần số của tất cả topics đang active
ros2 topic hz /camera/image_raw /scan /cmd_vel /odom

# Xem type của các topics
ros2 topic list -t

# Xem chi tiết một topic cụ thể
ros2 topic info /camera/image_raw
ros2 topic info /scan
ros2 topic info /cmd_vel
```

### Xem nodes đang chạy

```bash
ros2 node list
```

### Kiểm tra Robot State Publisher (RSP) và khung xe

```bash
# 1. Kiểm tra RSP node có đang chạy không
ros2 node list | grep robot_state_publisher

# 2. Kiểm tra topic /robot_description có được publish không
ros2 topic list | grep robot_description
ros2 topic echo /robot_description --once | head -50

# 3. Kiểm tra TF tree (xem các frames có được publish không)
ros2 run tf2_tools view_frames
# File frames.pdf sẽ được tạo ra trong thư mục hiện tại

# 4. Kiểm tra TF giữa các frames
ros2 run tf2_ros tf2_echo base_link laser_frame
# Hoặc
ros2 run tf2_ros tf2_echo odom base_link

# 5. Xem tất cả frames hiện có
ros2 run tf2_ros tf2_monitor

# 6. Kiểm tra node info của RSP
ros2 node info /robot_state_publisher

# 7. Nếu RSP có lỗi, kiểm tra xem URDF file có hợp lệ không:

# Cách 1: Lấy robot_description từ RSP node đang chạy (khuyến nghị)
ros2 param get /robot_state_publisher robot_description > /tmp/test_urdf.urdf
cat /tmp/test_urdf.urdf | head -100

# Cách 2: Kiểm tra URDF từ source directory
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar
ros2 run xacro xacro description/robot_ackermann.urdf.xacro use_ros2_control:=false sim_mode:=false > /tmp/test_urdf.urdf
cat /tmp/test_urdf.urdf | head -100

# Cách 3: Dùng đường dẫn từ package share (sau khi build)
cd ~/ros2_ws
source install/setup.bash
PKG_PATH=$(ros2 pkg prefix xe_lidar)/share/xe_lidar
ros2 run xacro xacro ${PKG_PATH}/description/robot_ackermann.urdf.xacro use_ros2_control:=false sim_mode:=false > /tmp/test_urdf.urdf
cat /tmp/test_urdf.urdf | head -100

# 8. Kiểm tra có cần joint_states không (nếu URDF có joints)
ros2 topic list | grep joint_state
# Nếu không có /joint_states và RSP không hiện khung, có thể cần joint_state_publisher
```

### Visualize với RViz2

```bash
# Đã được set alias trong .bashrc (tự động dùng software rendering)
rviz2
# Hoặc chạy trực tiếp với software rendering:
# LIBGL_ALWAYS_SOFTWARE=1 rviz2
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
- [ ] Camera test OK
- [ ] LiDAR test OK
- [ ] Arduino test OK
- [ ] Quyền truy cập serial/video: `groups $USER`

---

