# Arduino Code - Điều khiển Ackermann Steering

Code Arduino để nhận lệnh từ Raspberry Pi và điều khiển động cơ + servo cho hệ thống Ackermann steering.

## Cấu hình phần cứng

### Pin kết nối

#### Servo bánh lái
- **Pin 9**: Servo signal (PWM) cho bánh lái

#### Motor Driver (L298N hoặc TB6612) - 1 Motor DC chính
- **Motor chính** (điều khiển tốc độ tiến/lùi):
  - Pin 2: IN1
  - Pin 3: IN2
  - Pin 5: PWM (EN)

### Kết nối Serial
- Arduino kết nối với Raspberry Pi qua USB Serial
- Baudrate: **115200**

## Giao thức giao tiếp

### Format lệnh từ Raspberry Pi

Raspberry Pi gửi lệnh qua Serial với format:
```
V:linear:angular\n
```

Trong đó:
- `linear`: Tốc độ tuyến tính (m/s), từ -1.0 đến 1.0
  - Giá trị dương: tiến
  - Giá trị âm: lùi
- `angular`: Tốc độ góc (rad/s), từ -1.0 đến 1.0
  - Giá trị dương: quay trái
  - Giá trị âm: quay phải

### Ví dụ lệnh
```
V:0.3:0.0\n     -> Tiến thẳng với tốc độ 0.3 m/s
V:0.3:-0.5\n    -> Tiến và quay trái (tốc độ 0.3 m/s, quay 0.5 rad/s)
V:-0.2:0.0\n    -> Lùi với tốc độ 0.2 m/s
V:0.0:0.0\n     -> Dừng
```

### Timeout
- Nếu không nhận được lệnh trong **500ms**, robot sẽ tự động dừng (safety feature)

## Tham số robot

Các tham số này phải khớp với file URDF của robot:

```cpp
const float WHEELBASE = 0.4;      // 0.4m - khoảng cách bánh trước/sau
const float TRACK_WIDTH = 0.28;   // 0.28m - khoảng cách bánh trái/phải
const float MAX_STEER_ANGLE = 0.5236;  // ~30 độ (radians)
const float WHEEL_RADIUS = 0.034; // 0.034m - bán kính bánh xe
```

Nếu robot của bạn có kích thước khác, hãy điều chỉnh các giá trị này trong code.

## Cài đặt

1. Mở file `ackermann_motor_control.ino` trong Arduino IDE
2. Chọn board Arduino (Arduino Uno, Nano, hoặc tương đương)
3. Chọn cổng Serial đúng
4. Upload code vào Arduino
5. Mở Serial Monitor (115200 baud) để kiểm tra
6. Arduino sẽ gửi "READY" khi khởi động xong

## Kiểm tra

### Test Serial
Mở Serial Monitor và gửi thủ công:
```
V:0.3:0.0
```
Robot sẽ tiến với tốc độ 0.3 m/s.

### Test Servo
Gửi:
```
V:0.1:0.5
```
Servo sẽ quay để xe quay trái.

## Lưu ý

1. **Điều chỉnh servo center**: Nếu bánh lái không thẳng khi ở giữa, điều chỉnh `SERVO_CENTER` (mặc định 90 độ)

2. **Điều chỉnh hướng motor**: Nếu motor quay ngược chiều, đổi chỗ IN1 và IN2

3. **Điều chỉnh PWM**: Nếu tốc độ quá nhanh/chậm, điều chỉnh `MAX_LINEAR_VELOCITY` hoặc hệ số PWM

4. **Safety**: Code có tính năng timeout - nếu mất kết nối với Raspberry Pi, robot sẽ tự động dừng sau 500ms

5. **Motor Driver**: Code được thiết kế cho L298N hoặc TB6612. Nếu dùng driver khác, có thể cần điều chỉnh logic điều khiển

## Troubleshooting

### Robot không di chuyển
- Kiểm tra kết nối motor driver
- Kiểm tra nguồn cấp cho motor
- Kiểm tra Serial Monitor xem có nhận được lệnh không

### Servo không hoạt động
- Kiểm tra pin 9 đã kết nối đúng chưa
- Kiểm tra nguồn cho servo (5V)
- Điều chỉnh `SERVO_CENTER` nếu cần

### Lệnh không được nhận
- Kiểm tra baudrate (115200)
- Kiểm tra kết nối USB
- Kiểm tra format lệnh (phải có "\n" ở cuối)

## Kết nối với Raspberry Pi

Sau khi upload code Arduino, kết nối Arduino với Raspberry Pi qua USB. Trên Raspberry Pi, Arduino thường xuất hiện ở `/dev/ttyACM0` hoặc `/dev/ttyUSB0`.

Sử dụng node ROS2 `arduino_bridge` (trong thư mục raspberry_pi) để gửi lệnh từ ROS2 tới Arduino.

