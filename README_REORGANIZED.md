# Xe Tự Lái - Cấu trúc Code đã tổ chức lại

Dự án đã được tổ chức lại thành 2 phần chính: **Raspberry Pi** (ROS2) và **Arduino** (điều khiển phần cứng).

## 📁 Cấu trúc thư mục

```
xe_tu_lai/
├── raspberry_pi/          # Code cho Raspberry Pi (ROS2)
│   ├── xe_lidar/          # Package ROS2 chính
│   │   ├── scripts/
│   │   │   ├── obstacle_avoidance.py    # Node tự lái (Camera + LiDAR)
│   │   │   └── arduino_bridge.py        # Node bridge gửi cmd_vel tới Arduino
│   │   ├── launch/
│   │   │   ├── autonomous_drive_arduino.launch.py  # Launch với Arduino
│   │   │   ├── arduino_bridge.launch.py            # Chỉ launch bridge
│   │   │   └── ... (các launch file khác)
│   │   └── ...
│   └── libs/              # Các thư viện ROS2
│       ├── rplidar_ros/   # Driver cho RPLIDAR
│       └── serial/        # Thư viện Serial
│
└── arduino/               # Code cho Arduino
    ├── ackermann_motor_control.ino  # Code điều khiển motor + servo
    └── README.md          # Hướng dẫn Arduino
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
│  │  - Điều khiển Motor Driver (4 bánh)  │              │
│  └──────────────────────────────────────┘              │
│                                                         │
│         ┌──────────┬──────────┐                         │
│         │          │          │                         │
│    ┌────▼───┐  ┌───▼───┐  ┌──▼────┐                    │
│    │ Servo  │  │Motor A│  │Motor B│                    │
│    │ (Pin 9)│  │(L298N)│  │(L298N)│                    │
│    └────────┘  └───────┘  └───────┘                    │
└─────────────────────────────────────────────────────────┘
```

## 🚀 Sử dụng

### 1. Cài đặt trên Raspberry Pi

```bash
# Cài đặt pyserial cho Arduino bridge
pip3 install pyserial

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Upload code Arduino

1. Mở `arduino/ackermann_motor_control.ino` trong Arduino IDE
2. Chọn board (Arduino Uno/Nano)
3. Upload code
4. Kiểm tra Serial Monitor (115200 baud) - phải thấy "READY"

### 3. Chạy hệ thống

#### Chạy đầy đủ với Arduino:

```bash
source ~/ros2_ws/install/setup.bash

# Chạy với Arduino (tự động phát hiện port)
ros2 launch xe_lidar autonomous_drive_arduino.launch.py

# Hoặc chỉ định port cụ thể
ros2 launch xe_lidar autonomous_drive_arduino.launch.py \
    arduino_serial_port:=/dev/ttyACM0 \
    lidar_serial_port:=/dev/ttyUSB0 \
    video_device:=/dev/video0
```

#### Chạy từng phần:

```bash
# Chỉ Arduino Bridge (để test)
ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0

# Test gửi lệnh thủ công
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.3}, angular: {z: 0.0}"

# Chỉ Autonomous Drive (không có Arduino)
ros2 run xe_lidar obstacle_avoidance.py
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

## ⚙️ Cấu hình Arduino

Các tham số trong `ackermann_motor_control.ino` có thể điều chỉnh:

```cpp
const float WHEELBASE = 0.4;           // Khoảng cách bánh trước/sau (m)
const float TRACK_WIDTH = 0.28;        // Khoảng cách bánh trái/phải (m)
const float MAX_STEER_ANGLE = 0.5236;  // Góc quay tối đa (rad) ~30 độ
const float WHEEL_RADIUS = 0.034;      // Bán kính bánh xe (m)
const int SERVO_CENTER = 90;           // Góc giữa của servo (degrees)
```

## 🔧 Kết nối phần cứng

### Arduino Pin

- **Pin 9**: Servo bánh lái (PWM)
- **Motor A** (Bánh trái):
  - Pin 2: IN1
  - Pin 3: IN2
  - Pin 5: PWM
- **Motor B** (Bánh phải):
  - Pin 4: IN1
  - Pin 7: IN2
  - Pin 6: PWM

### Serial

- Kết nối USB giữa Arduino và Raspberry Pi
- Baudrate: **115200**

## 📝 Lưu ý

1. **Điều chỉnh servo center**: Nếu bánh lái không thẳng, điều chỉnh `SERVO_CENTER` trong Arduino code

2. **Hướng motor**: Nếu motor quay ngược, đổi chỗ IN1 và IN2 trong code

3. **Port Serial**: 
   - Arduino thường ở `/dev/ttyACM0` hoặc `/dev/ttyUSB0`
   - LiDAR thường ở `/dev/ttyUSB0` hoặc `/dev/ttyUSB1`
   - Có thể dùng `ls /dev/tty*` để kiểm tra

4. **Quyền truy cập Serial**: Đảm bảo user có quyền truy cập serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout và login lại
   ```

5. **Safety**: Code Arduino có tính năng timeout - nếu mất kết nối, robot sẽ tự động dừng sau 500ms

## 🔍 Troubleshooting

### Arduino không nhận lệnh
- Kiểm tra Serial Monitor xem có nhận được "READY" không
- Kiểm tra baudrate (115200)
- Kiểm tra kết nối USB
- Kiểm tra quyền truy cập: `ls -l /dev/ttyACM0`

### Robot không di chuyển
- Kiểm tra motor driver đã được cấp nguồn chưa
- Kiểm tra kết nối motor
- Test từng motor riêng lẻ trong code Arduino

### Servo không hoạt động
- Kiểm tra pin 9 đã kết nối đúng
- Kiểm tra nguồn servo (5V)
- Điều chỉnh `SERVO_CENTER` nếu bánh lái không thẳng

### Bridge không kết nối
- Kiểm tra port: `ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0`
- Kiểm tra log của node để xem lỗi cụ thể

