# Arduino Code - Điều khiển Ackermann Steering

Code Arduino để nhận lệnh từ Raspberry Pi và điều khiển hệ thống Ackermann steering với **1 Motor DC** và **1 Servo**.

## 📋 Tổng quan

- **1 Motor DC**: Điều khiển tốc độ tiến/lùi của xe
- **1 Servo**: Điều khiển góc quay bánh lái (Ackermann steering)

## 🔌 Cấu hình phần cứng

### Pin kết nối Arduino

#### Servo bánh lái
- **Pin 9**: Servo signal (PWM) cho bánh lái
- Nguồn: 5V (từ Arduino hoặc nguồn ngoài)
- GND: Chân GND của Arduino

#### Motor Driver (L298N hoặc TB6612)
**1 Motor DC chính** (điều khiển tốc độ tiến/lùi):
- **Pin 2**: IN1 (điều khiển hướng)
- **Pin 3**: IN2 (điều khiển hướng)
- **Pin 5**: PWM/EN (điều khiển tốc độ)
- **Nguồn motor**: Nối trực tiếp vào Motor Driver (không từ Arduino)
- **GND**: Chia sẻ GND với Arduino

### Sơ đồ kết nối đơn giản

```
Arduino          Motor Driver (L298N)      Motor DC
Pin 2  ────────> IN1
Pin 3  ────────> IN2
Pin 5  ────────> EN (Enable)
GND    ────────> GND ───────────────────> GND
                  OUT1 ──────────────────> Motor DC (+)
                  OUT2 ──────────────────> Motor DC (-)
                  
Arduino          Servo
Pin 9  ────────> Signal (Yellow/Orange)
5V     ────────> Power (Red)
GND    ────────> Ground (Brown/Black)
```

### Kết nối Serial
- Arduino kết nối với Raspberry Pi qua **USB Serial**
- Baudrate: **115200**

## 📡 Giao thức giao tiếp

### Format lệnh từ Raspberry Pi

Raspberry Pi gửi lệnh qua Serial với format:
```
V:linear:angular\n
```

**Trong đó:**
- `linear`: Tốc độ tuyến tính (m/s), từ **-1.0 đến 1.0**
  - Giá trị **dương**: tiến
  - Giá trị **âm**: lùi
- `angular`: Tốc độ góc (rad/s), từ **-1.0 đến 1.0**
  - Giá trị **dương**: quay trái
  - Giá trị **âm**: quay phải

### Ví dụ lệnh

```
V:0.3:0.0\n     -> Tiến thẳng với tốc độ 0.3 m/s
V:0.3:-0.5\n    -> Tiến và quay trái (tốc độ 0.3 m/s, quay 0.5 rad/s)
V:-0.2:0.0\n    -> Lùi với tốc độ 0.2 m/s
V:0.0:0.0\n     -> Dừng (motor dừng, servo về giữa)
```

### Timeout (An toàn)
- Nếu không nhận được lệnh trong **500ms**, robot sẽ **tự động dừng** (safety feature)
- Motor sẽ dừng và servo về vị trí giữa

## ⚙️ Tham số robot

Các tham số này phải **khớp với file URDF** của robot. Nếu robot của bạn có kích thước khác, hãy điều chỉnh trong code:

```cpp
const float WHEELBASE = 0.4;           // Khoảng cách bánh trước/sau (m)
const float TRACK_WIDTH = 0.21;        // Khoảng cách bánh trái/phải (m)
const float MAX_STEER_ANGLE = 0.5236;  // Góc quay tối đa (rad) ~30 độ
const float WHEEL_RADIUS = 0.034;      // Bán kính bánh xe (m)
const int SERVO_CENTER_DEFAULT = 100;  // Góc giữa mặc định của servo (degrees)
const int SERVO_RANGE = 45;            // ± độ so với center
const float MAX_LINEAR_VELOCITY = 1.0; // Tốc độ tối đa (m/s)
```

## 📥 Cài đặt

1. Mở file `ackermann_motor_control.ino` trong **Arduino IDE**
2. Chọn board Arduino (Arduino Uno, Nano, hoặc tương đương)
3. Chọn cổng Serial đúng (Tools → Port)
4. **Upload code** vào Arduino
5. Mở **Serial Monitor** (Ctrl+Shift+M) với baudrate **115200**
6. Arduino sẽ gửi `READY` khi khởi động xong

## 🧪 Kiểm tra

### Test Serial Connection
Mở Serial Monitor và kiểm tra:
- Phải thấy `READY`
- Phải thấy các thông số: Wheelbase, Track Width, Max Steer Angle

### Test Motor
Gửi lệnh trong Serial Monitor:
```
V:0.3:0.0
```
Motor sẽ quay để xe tiến với tốc độ 0.3 m/s. Nếu motor quay ngược chiều, xem phần "Lưu ý" bên dưới.

### Test Servo
Gửi:
```
V:0.1:0.5
```
Servo sẽ quay để xe quay trái khi tiến. Quan sát bánh lái có quay đúng không.

### Test kết hợp
Gửi:
```
V:0.3:-0.5
```
Xe sẽ tiến và quay trái cùng lúc.

## ⚠️ Lưu ý quan trọng

1. **Điều chỉnh servo center**: 
   - Nếu bánh lái không thẳng khi ở giữa, điều chỉnh `SERVO_CENTER` (mặc định 90 độ)
   - Test: Gửi `V:0.1:0.0` và quan sát bánh lái có thẳng không

2. **Điều chỉnh hướng motor**: 
   - Nếu motor quay ngược chiều (lùi khi lệnh tiến), đổi chỗ `IN1` và `IN2` trong code:
     ```cpp
     // Trong setMotorsForward(), đổi:
     digitalWrite(MOTOR_IN1, LOW);   // Thay vì HIGH
     digitalWrite(MOTOR_IN2, HIGH);  // Thay vì LOW
     ```

3. **Điều chỉnh tốc độ**: 
   - Nếu tốc độ quá nhanh/chậm, điều chỉnh `MAX_LINEAR_VELOCITY`
   - Hoặc điều chỉnh hệ số PWM trong hàm `controlMotors()`

4. **Nguồn cấp**: 
   - Motor DC cần nguồn riêng (không lấy từ Arduino)
   - Servo có thể dùng 5V từ Arduino nếu nhẹ, hoặc nguồn ngoài nếu nặng
   - **BẮT BUỘC**: Chia sẻ GND giữa Arduino, Motor Driver và Servo

5. **Safety**: 
   - Code có tính năng timeout - nếu mất kết nối với Raspberry Pi, robot sẽ tự động dừng sau 500ms
   - Luôn test trong môi trường an toàn trước khi chạy tự động

6. **Motor Driver**: 
   - Code được thiết kế cho L298N hoặc TB6612
   - Nếu dùng driver khác, có thể cần điều chỉnh logic điều khiển

## 🔧 Troubleshooting

### Arduino không nhận lệnh
- ✅ Kiểm tra Serial Monitor xem có nhận được `READY` không
- ✅ Kiểm tra baudrate (115200)
- ✅ Kiểm tra kết nối USB
- ✅ Kiểm tra format lệnh (phải có `\n` ở cuối)
- ✅ Thử gửi thủ công trong Serial Monitor: `V:0.3:0.0`

### Robot không di chuyển
- ✅ Kiểm tra motor driver đã được cấp nguồn chưa
- ✅ Kiểm tra kết nối motor DC (pins 2, 3, 5)
- ✅ Kiểm tra motor có quay khi test thủ công không (nối trực tiếp vào nguồn)
- ✅ Kiểm tra Serial Monitor xem có nhận được lệnh không
- ✅ Điều chỉnh `MAX_LINEAR_VELOCITY` nếu tốc độ quá thấp

### Servo không hoạt động
- ✅ Kiểm tra pin 9 đã kết nối đúng chưa
- ✅ Kiểm tra nguồn cho servo (5V)
- ✅ Kiểm tra dây signal (màu vàng/cam) đã nối đúng chưa
- ✅ Điều chỉnh `SERVO_CENTER` nếu bánh lái không thẳng
- ✅ Test servo bằng cách gửi các giá trị góc khác nhau

### Motor quay ngược chiều
- Sửa trong code: đổi chỗ `IN1` và `IN2` (xem phần "Lưu ý" ở trên)

### Lệnh bị timeout liên tục
- Kiểm tra kết nối Serial với Raspberry Pi
- Kiểm tra node `arduino_bridge` có đang chạy không
- Kiểm tra port Serial có đúng không

## 🔗 Kết nối với Raspberry Pi

Sau khi upload code Arduino và test thành công:

1. Kết nối Arduino với Raspberry Pi qua USB
2. Kiểm tra port Serial:
   ```bash
   ls /dev/tty* | grep -i "acm\|usb"
   # Thường là /dev/ttyACM0 hoặc /dev/ttyUSB0
   ```

3. Cấp quyền truy cập:
   ```bash
   sudo usermod -a -G dialout $USER
   # Logout và login lại
   ```

4. Chạy Arduino Bridge trên Raspberry Pi:
   ```bash
   ros2 launch xe_lidar arduino_bridge.launch.py serial_port:=/dev/ttyACM0
   ```

5. Test gửi lệnh từ ROS2:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.3}, angular: {z: 0.0}"
   ```

## 📚 Tài liệu tham khảo

- Motor Driver L298N: [Datasheet](https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf)
- Servo Motor: [Arduino Servo Library](https://www.arduino.cc/reference/en/libraries/servo/)
- Ackermann Steering: [Wikipedia](https://en.wikipedia.org/wiki/Ackermann_steering_geometry)
