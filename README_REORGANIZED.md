# Xe Tự Lái - Hệ thống Ackermann Steering

Hệ thống xe tự lái sử dụng **Camera** và **LiDAR** để phát hiện và tránh vật cản tự động, điều khiển bằng **1 Motor DC + 1 Servo** (Ackermann Steering).

## 📋 Tổng quan hệ thống

- **Camera**: Phát hiện vạch kẻ đường và điều chỉnh để đi giữa đường
- **LiDAR**: Phát hiện và tránh vật cản tự động
- **Arduino**: Điều khiển 1 Motor DC (tốc độ) + 1 Servo (bánh lái)
- **Raspberry Pi**: Xử lý AI và điều khiển qua ROS2

## 📁 Cấu trúc thư mục

```
xe_tu_lai/
├── raspberry_pi/          # Code cho Raspberry Pi (ROS2)
│   ├── xe_lidar/          # Package ROS2 chính
│   │   ├── scripts/
│   │   │   ├── obstacle_avoidance.py    # Node tự lái (Camera + LiDAR)
│   │   │   └── arduino_bridge.py        # Node bridge gửi cmd_vel tới Arduino
│   │   ├── launch/
│   │   │   ├── autonomous_drive_arduino.launch.py  # Launch tất cả (Camera + LiDAR + Arduino)
│   │   │   ├── arduino_bridge.launch.py            # Chỉ launch Arduino bridge
│   │   │   ├── camera.launch.py                    # Chỉ launch Camera
│   │   │   └── rplidar.launch.py                   # Chỉ launch LiDAR
│   │   └── ...
│   └── libs/              # Các thư viện ROS2
│       ├── rplidar_ros/   # Driver cho RPLIDAR
│       └── serial/        # Thư viện Serial
│
└── arduino/               # Code cho Arduino
    ├── ackermann_motor_control.ino  # Code điều khiển 1 Motor DC + 1 Servo
    └── README.md          # Hướng dẫn chi tiết Arduino
```

## 🔄 Kiến trúc hệ thống

```
┌─────────────────────────────────────────────────────────┐
│  Raspberry Pi (ROS2)                                    │
│                                                         │
│  ┌──────────────┐  ┌──────────────┐                    │
│  │   Camera     │  │    LiDAR     │                    │
│  │   Node       │  │    Node      │                    │
│  └──────┬───────┘  └──────┬───────┘                    │
│         │                  │                            │
│         │  /camera/image   │  /scan                     │
│         │                  │                            │
│         └──────────┬───────┘                            │
│                    │                                    │
│         ┌──────────▼──────────┐                         │
│         │  Autonomous Drive   │                         │
│         │     Node            │                         │
│         │  (obstacle_avoidance)│                        │
│         └──────────┬──────────┘                         │
│                    │                                    │
│                    │ /cmd_vel                           │
│                    │                                    │
│         ┌──────────▼──────────┐                         │
│         │  Arduino Bridge     │                         │
│         │      Node           │                         │
│         └──────────┬──────────┘                         │
└────────────────────┼────────────────────────────────────┘
                     │ Serial (USB) 115200 baud
                     │ Format: "V:linear:angular\n"
┌────────────────────▼────────────────────────────────────┐
│  Arduino                                                │
│                                                         │
│  ┌──────────────────────────────────────┐              │
│  │  ackermann_motor_control.ino         │              │
│  │                                       │              │
│  │  - Parse Serial command               │              │
│  │  - Tính góc lái (Ackermann)          │              │
│  │  - Điều khiển Servo (bánh lái)       │              │
│  │  - Điều khiển 1 Motor DC (tiến/lùi)  │              │
│  └──────────────────────────────────────┘              │
│                                                         │
│              ┌──────────┬──────────┐                    │
│              │          │          │                    │
│         ┌────▼───┐  ┌───▼────┐                         │
│         │ Servo  │  │ 1 Motor│                         │
│         │ (Pin 9)│  │   DC   │                         │
│         │        │  │(L298N) │                         │
│         └────────┘  │Pin 2,3,5│                         │
│                     └─────────┘                         │
└─────────────────────────────────────────────────────────┘
```

## 🚀 Hướng dẫn sử dụng

> **⚠️ Lưu ý quan trọng**: Nên **test phần cứng trước** bằng các script Python (Bước 3) trước khi chạy với ROS2 để tránh lỗi và tiết kiệm thời gian debug.

### Bước 1: Chuẩn bị phần cứng

1. **Kết nối Arduino với Motor DC và Servo**
   - Xem hướng dẫn chi tiết trong `arduino/README.md`
   - Upload code `arduino/ackermann_motor_control.ino` vào Arduino
   - Kiểm tra Serial Monitor (115200 baud) - phải thấy "READY"

2. **Kết nối phần cứng với Raspberry Pi**
   - Arduino → USB → Raspberry Pi
   - LiDAR → USB → Raspberry Pi  
   - Camera → USB → Raspberry Pi

3. **Kiểm tra các thiết bị**
   ```bash
   # Kiểm tra Arduino
   ls /dev/ttyACM*
   # Thường là /dev/ttyACM0
   
   # Kiểm tra LiDAR
   ls /dev/ttyUSB*
   # Thường là /dev/ttyUSB0
   
   # Kiểm tra Camera
   ls /dev/video*
   # Thường là /dev/video0
   ```

4. **Cấp quyền truy cập Serial**
   ```bash
   sudo usermod -a -G dialout $USER
   sudo usermod -a -G video $USER
   # Logout và login lại để áp dụng
   ```

### Bước 2: Cài đặt trên Raspberry Pi

```bash
# 1. Cài đặt dependencies cơ bản
sudo apt update
sudo apt install -y \
    python3-opencv \
    python3-numpy \
    python3-pip \
    python3-serial

# 2. Cài đặt pyserial cho test script và Arduino bridge
pip3 install pyserial

# 3. Cài đặt ROS2 dependencies (nếu chưa có ROS2)
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-ackermann-msgs

# 4. Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 5. Thêm vào .bashrc để tự động source
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# 6. Cấp quyền thực thi cho test scripts
chmod +x src/xe_tu_lai/raspberry_pi/xe_lidar/scripts/test_*.py
```

### Bước 3: Test phần cứng trước (Khuyến nghị)

**TRƯỚC KHI chạy ROS2**, nên test từng phần cứng để đảm bảo hoạt động tốt:

#### Test Camera đầy đủ với Lane Detection (không cần ROS2)

```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts
python3 test_camera_full.py --device 0
```

Hoặc chỉ định device khác:
```bash
python3 test_camera_full.py --device 1 --width 640 --height 480
```

**Tính năng:**
- Hiển thị ảnh real-time với FPS counter
- Phát hiện vạch kẻ đường (Lane Detection)
- Vẽ vạch trái/phải/giữa đường
- Hiển thị offset từ giữa đường

**Điều khiển:**
- Nhấn 'q' để thoát
- Nhấn 's' để lưu ảnh
- Nhấn 'd' để bật/tắt lane detection

#### Test LiDAR A1M8 đầy đủ (không cần ROS2)

**Cách 1: Sử dụng ROS2 topic (Khuyến nghị - chính xác nhất)**

```bash
# Terminal 1: Chạy rplidar_ros node
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0

# Terminal 2: Chạy test script
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts
python3 test_lidar_a1m8.py --use-ros2 --duration 30
```

**Cách 2: Đọc trực tiếp từ Serial (đơn giản)**

```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts
python3 test_lidar_a1m8.py --port /dev/ttyUSB0 --use-serial --duration 30
```

**Tính năng:**
- Đọc dữ liệu quét 360 độ
- Phát hiện vật cản phía trước (vùng 60°)
- Hiển thị thống kê: số điểm, khoảng cách trung bình/gần nhất
- Cảnh báo vật cản khi < 0.8m

**Kiểm tra:**
- Kết nối được với LiDAR
- Nhận được dữ liệu quét
- Phát hiện vật cản chính xác
- Nhấn Ctrl+C để dừng

#### Test Arduino (không cần ROS2)

**Chế độ tương tác:**
```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts
python3 test_arduino.py --port /dev/ttyACM0
```

Sau đó nhập lệnh:
```
0.3,0.0    # Tiến 0.3 m/s (thẳng)
0.3,-0.5   # Tiến và quay trái
0.2,0.5    # Tiến và quay phải
0,0        # Dừng
q          # Thoát
```

**Chế độ tự động:**
```bash
python3 test_arduino.py --port /dev/ttyACM0 --auto
```

**Kiểm tra:**
- Arduino trả lời "READY" khi khởi động
- Motor quay khi gửi lệnh tiến (linear > 0)
- Servo quay khi gửi lệnh có angular (angular != 0)
- Robot dừng khi gửi lệnh dừng (0,0)

### Bước 4: Chạy Autonomous Drive với Python (KHÔNG CẦN ROS2)

**Script chính chạy toàn bộ hệ thống giống như ROS2:**

```bash
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts

# Chạy với tất cả tính năng
python3 autonomous_drive_python.py \
    --camera-device 0 \
    --lidar-port /dev/ttyUSB0 \
    --arduino-port /dev/ttyACM0 \
    --show-display
```

**Hoặc sử dụng LiDAR từ ROS2 (chính xác hơn):**

```bash
# Terminal 1: Chạy rplidar_ros node
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0

# Terminal 2: Chạy autonomous drive
python3 autonomous_drive_python.py \
    --use-ros2-lidar \
    --camera-device 0 \
    --arduino-port /dev/ttyACM0 \
    --show-display
```

**Các tùy chọn:**
```bash
--camera-device 0              # Device ID camera
--lidar-port /dev/ttyUSB0      # Port LiDAR (nếu không dùng ROS2)
--arduino-port /dev/ttyACM0    # Port Arduino (None = auto detect)
--use-ros2-lidar               # Sử dụng ROS2 topic /scan (khuyến nghị)
--show-display                 # Hiển thị camera và thông tin
--no-camera                    # Tắt camera
--max-linear-speed 0.3         # Tốc độ tối đa (m/s)
--max-angular-speed 1.0        # Tốc độ quay tối đa (rad/s)
--safe-distance 0.8            # Khoảng cách an toàn (m)
```

**Logic điều khiển (giống hệt ROS2):**
- ✅ Ưu tiên 1: Tránh vật cản (LiDAR) - Safety
- ✅ Ưu tiên 2: Đi theo vạch kẻ đường (Camera) - Navigation
- ✅ Điều khiển Ackermann: 1 Motor DC + 1 Servo

**Điều khiển:**
- Nhấn 'q' để thoát
- Nhấn 's' để dừng robot

### Bước 5: Chạy hệ thống với ROS2 (Tùy chọn)

Sau khi test xong tất cả phần cứng, mới chạy với ROS2. Có 2 cách: **Chạy tất cả cùng lúc** (khuyến nghị) hoặc **Chạy từng phần**.

#### Cách 1: Chạy tất cả cùng lúc (Khuyến nghị)

```bash
source ~/ros2_ws/install/setup.bash

# Chạy tất cả: Camera + LiDAR + Arduino + Autonomous Drive
ros2 launch xe_lidar autonomous_drive_arduino.launch.py
```

Hoặc chỉ định port cụ thể:
```bash
ros2 launch xe_lidar autonomous_drive_arduino.launch.py \
    arduino_serial_port:=/dev/ttyACM0 \
    lidar_serial_port:=/dev/ttyUSB0 \
    video_device:=/dev/video0
```

#### Cách 2: Chạy từng phần (Để test hoặc debug)

**Terminal 1 - Bật Camera:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

**Terminal 2 - Bật LiDAR:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar rplidar.launch.py serial_port:=/dev/ttyUSB0
```

**Terminal 3 - Bật Arduino Bridge:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
```

**Terminal 4 - Bật Autonomous Drive:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run xe_lidar obstacle_avoidance.py
```

### Bước 6: Kiểm tra hệ thống ROS2

#### Kiểm tra Camera
```bash
# Xem ảnh từ camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Hoặc kiểm tra topic
ros2 topic echo /camera/image_raw --once
```

#### Kiểm tra LiDAR
```bash
# Xem dữ liệu LiDAR
ros2 topic echo /scan --once

# Hoặc xem trong RViz2
rviz2
# Thêm LaserScan topic: /scan
```

#### Kiểm tra Arduino
```bash
# Kiểm tra topic cmd_vel
ros2 topic echo /cmd_vel

# Test gửi lệnh thủ công
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.3, y: 0.0, z: 0.0}, \
     angular: {x: 0.0, y: 0.0, z: 0.0}"

# Test quay
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "linear: {x: 0.2, y: 0.0, z: 0.0}, \
     angular: {x: 0.0, y: 0.0, z: -0.5}"
```

#### Kiểm tra toàn bộ hệ thống
```bash
# Xem tất cả topics
ros2 topic list

# Xem node đang chạy
ros2 node list

# Xem log của autonomous_drive
ros2 topic echo /rosout | grep autonomous_drive
```

## 📡 Giao thức giao tiếp

### Format lệnh từ Raspberry Pi → Arduino

```
V:linear:angular\n
```

- `linear`: Tốc độ tuyến tính (m/s), -1.0 đến 1.0
- `angular`: Tốc độ góc (rad/s), -1.0 đến 1.0

**Ví dụ:**
```
V:0.3:0.0\n     -> Tiến thẳng 0.3 m/s
V:0.3:-0.5\n    -> Tiến và quay trái
V:0.0:0.0\n     -> Dừng
```

### Timeout

- Nếu không nhận được lệnh trong **500ms**, Arduino sẽ tự động dừng robot (safety feature)

## ⚙️ Cấu hình

### Tham số Autonomous Drive

Có thể điều chỉnh trong launch file hoặc khi chạy node:

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

### Tham số Arduino

Điều chỉnh trong file `arduino/ackermann_motor_control.ino`:

```cpp
const float WHEELBASE = 0.4;           // Khoảng cách bánh trước/sau (m)
const float TRACK_WIDTH = 0.28;        // Khoảng cách bánh trái/phải (m)
const float MAX_STEER_ANGLE = 0.5236;  // Góc quay tối đa (rad) ~30 độ
const float WHEEL_RADIUS = 0.034;      // Bán kính bánh xe (m)
const int SERVO_CENTER = 90;           // Góc giữa của servo (degrees)
const float MAX_LINEAR_VELOCITY = 1.0; // Tốc độ tối đa (m/s)
```

## 🔧 Kết nối phần cứng

### Arduino Pin

- **Pin 9**: Servo bánh lái (PWM signal)
- **1 Motor DC chính** (điều khiển tốc độ tiến/lùi):
  - Pin 2: IN1 (Motor Driver L298N/TB6612)
  - Pin 3: IN2 (Motor Driver L298N/TB6612)
  - Pin 5: PWM (Enable pin)

### Serial Ports

- **Arduino**: `/dev/ttyACM0` hoặc `/dev/ttyUSB0` (baudrate: 115200)
- **LiDAR**: `/dev/ttyUSB0` hoặc `/dev/ttyUSB1` (baudrate: 115200)
- **Camera**: `/dev/video0`

Kiểm tra ports:
```bash
ls -l /dev/tty* | grep -E "ACM|USB"
ls -l /dev/video*
```

## 📝 Lưu ý quan trọng

1. **Thứ tự khởi động**: 
   - Khuyến nghị dùng launch file `autonomous_drive_arduino.launch.py` để chạy tất cả cùng lúc
   - Nếu chạy riêng, khởi động theo thứ tự: Camera → LiDAR → Arduino Bridge → Autonomous Drive

2. **Quyền truy cập**: 
   - Đảm bảo user đã được thêm vào groups `dialout` và `video`
   - Logout và login lại sau khi thêm groups

3. **Port Serial**: 
   - Nếu có nhiều thiết bị USB, port có thể thay đổi
   - Dùng `ls -l /dev/tty*` để kiểm tra port mới nhất

4. **Safety**: 
   - Luôn test trong môi trường an toàn
   - Code có timeout tự động dừng khi mất kết nối
   - Kiểm tra robot trước khi chạy tự động

5. **Điều chỉnh tham số**: 
   - Điều chỉnh tốc độ và khoảng cách an toàn phù hợp với môi trường thực tế
   - Điều chỉnh servo center nếu bánh lái không thẳng

## 🔍 Troubleshooting

### Camera không hoạt động
```bash
# Kiểm tra device
ls -l /dev/video*

# Cấp quyền
sudo chmod 777 /dev/video0

# Test camera
ros2 run v4l2_camera v4l2_camera_node
```

### LiDAR không hoạt động
```bash
# Kiểm tra port
ls -l /dev/ttyUSB*

# Cấp quyền
sudo chmod 666 /dev/ttyUSB0

# Test LiDAR
ros2 run rplidar_ros rplidar_composition --ros-args \
    -p serial_port:=/dev/ttyUSB0 \
    -p serial_baudrate:=115200
```

### Arduino không nhận lệnh
```bash
# Kiểm tra Serial Monitor trên Arduino IDE
# Phải thấy "READY" khi khởi động

# Kiểm tra kết nối từ Raspberry Pi
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0

# Kiểm tra quyền truy cập
ls -l /dev/ttyACM0
sudo usermod -a -G dialout $USER
```

### Robot không di chuyển
```bash
# Kiểm tra cmd_vel có được publish không
ros2 topic echo /cmd_vel

# Kiểm tra Arduino bridge có gửi lệnh không
ros2 topic echo /rosout | grep arduino_bridge

# Kiểm tra motor driver và nguồn
# Xem hướng dẫn trong arduino/README.md
```

### Autonomous Drive không hoạt động
```bash
# Kiểm tra các topic có dữ liệu không
ros2 topic echo /scan --once
ros2 topic echo /camera/image_raw --once

# Kiểm tra log
ros2 topic echo /rosout | grep autonomous_drive

# Kiểm tra node có đang chạy không
ros2 node list
```

## 📚 Tài liệu tham khảo

- **Arduino**: Xem `arduino/README.md` để biết chi tiết về code Arduino
- **ROS2**: [ROS2 Documentation](https://docs.ros.org/en/humble/)
- **RPLIDAR**: [rplidar_ros](https://github.com/Slamtec/rplidar_ros)

## 🎯 Tóm tắt lệnh nhanh

### Test phần cứng (KHÔNG CẦN ROS2)

```bash
# Test Camera
cd ~/ros2_ws/src/xe_tu_lai/raspberry_pi/xe_lidar/scripts
python3 test_camera.py

# Test LiDAR
python3 test_lidar.py --port /dev/ttyUSB0

# Test Arduino
python3 test_arduino.py --port /dev/ttyACM0
```

### Chạy với ROS2

```bash
# Chạy tất cả
ros2 launch xe_lidar autonomous_drive_arduino.launch.py

# Chạy từng phần
ros2 launch xe_lidar camera.launch.py
ros2 launch xe_lidar rplidar.launch.py
ros2 launch xe_lidar arduino_bridge.launch.py
ros2 run xe_lidar obstacle_avoidance.py

# Kiểm tra
ros2 topic list
ros2 node list
ros2 topic echo /cmd_vel
```
