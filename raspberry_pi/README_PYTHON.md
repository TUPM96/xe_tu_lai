# Hướng dẫn chạy Xe Tự Lái trên Raspberry Pi (KHÔNG CẦN ROS2)

Hướng dẫn chi tiết để chạy hệ thống xe tự lái bằng Python thuần trên Raspberry Pi, không cần cài đặt ROS2.

## 📋 Yêu cầu hệ thống

- **Raspberry Pi**: Raspberry Pi 5
- **OS**: Raspberry Pi OS
- **Python**: Python 3.8 trở lên
- **Phần cứng**:
  - RPLIDAR A1M8 (hoặc tương đương)
  - USB Camera
  - Arduino (đã upload code `ackermann_motor_control.ino`)
  - 1 Motor DC + 1 Servo (Ackermann Steering)

## 🚀 Cài đặt

### Bước 1: Cài đặt dependencies

```bash
# Cài đặt aptitude (nếu chưa có)
sudo apt install -y aptitude

# Cập nhật hệ thống
sudo aptitude update && sudo aptitude full-upgrade -y

# Cài đặt Python packages
sudo aptitude install -y \
    python3 \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-serial

# Cài đặt pyserial
pip3 install pyserial
```

### Bước 2: Cấp quyền truy cập Serial và Video

```bash
# Thêm user vào groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER

# Logout và login lại để áp dụng thay đổi
```

### Bước 3: Kiểm tra thiết bị

```bash
# Kiểm tra Camera
ls -l /dev/video*

# Kiểm tra Arduino
ls -l /dev/ttyACM*

# Kiểm tra LiDAR
ls -l /dev/ttyUSB*

# Nếu cần, cấp quyền tạm thời
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0
sudo chmod 777 /dev/video0
```

### Bước 4: Copy code vào Raspberry Pi

```bash
# Tạo thư mục làm việc
mkdir -p ~/xe_tu_lai
cd ~/xe_tu_lai

# Copy các script Python (từ máy dev hoặc clone repo)
# Đảm bảo có các file:
# - test_camera_full.py
# - test_lidar_a1m8.py
# - test_arduino.py
# - autonomous_drive_python.py

# Cấp quyền thực thi
chmod +x *.py
```

## 🧪 Test từng phần cứng

### Test Camera với Lane Detection

```bash
cd ~/xe_tu_lai
python3 test_camera_full.py --device 0
```

**Tính năng:**
- ✅ Hiển thị ảnh real-time với FPS counter
- ✅ Phát hiện vạch kẻ đường (Lane Detection)
- ✅ Vẽ vạch trái/phải/giữa đường
- ✅ Hiển thị offset từ giữa đường (-1 đến 1)

**Điều khiển:**
- `q`: Thoát
- `s`: Lưu ảnh
- `d`: Bật/tắt lane detection

**Ví dụ:**
```bash
# Test với camera khác
python3 test_camera_full.py --device 1 --width 640 --height 480

# Tắt lane detection (chỉ xem ảnh)
python3 test_camera_full.py --no-lane
```

### Test LiDAR A1M8

```bash
cd ~/xe_tu_lai
python3 test_lidar_a1m8.py --port /dev/ttyUSB0 --use-serial --duration 30
```

**Thông tin hiển thị:**
- Số vòng quét nhận được
- Tổng số điểm trong mỗi vòng
- Thống kê vùng phía trước (0°±30°):
  - Số điểm phát hiện
  - Khoảng cách trung bình
  - Khoảng cách gần nhất
- ⚠️ Cảnh báo vật cản khi < 0.8m

**Lưu ý**: Script đọc trực tiếp từ serial port, không cần ROS2.

### Test Arduino

```bash
cd ~/xe_tu_lai
python3 test_arduino.py --port /dev/ttyACM0
```

**Chế độ tương tác:**

Sau khi chạy, nhập lệnh theo format: `linear,angular`

```
0.3,0.0     # Tiến 0.3 m/s (thẳng)
0.3,-0.5    # Tiến và quay trái
0.2,0.5     # Tiến và quay phải
-0.2,0.0    # Lùi
0,0         # Dừng
q           # Thoát
```

**Chế độ tự động:**

```bash
python3 test_arduino.py --port /dev/ttyACM0 --auto
```

Script sẽ tự động gửi các lệnh test:
- Dừng
- Tiến thẳng
- Tiến và quay trái
- Tiến và quay phải
- Lùi
- Dừng

**Kiểm tra:**
- ✅ Arduino phải trả lời "READY" khi khởi động
- ✅ Motor quay khi gửi lệnh tiến (linear > 0)
- ✅ Servo quay khi gửi lệnh có angular (angular != 0)
- ✅ Robot dừng khi gửi lệnh dừng (0,0)

## 🚗 Chạy Autonomous Drive (Tự lái hoàn toàn)

Sau khi test xong tất cả phần cứng, chạy script chính để xe tự lái:

### Chạy Autonomous Drive

```bash
cd ~/xe_tu_lai
python3 autonomous_drive_python.py \
    --camera-device 0 \
    --lidar-port /dev/ttyUSB0 \
    --arduino-port /dev/ttyACM0 \
    --show-display
```

**Lưu ý**: 
- Script sẽ đọc LiDAR trực tiếp từ serial port `/dev/ttyUSB0`
- Nếu LiDAR chưa kết nối, script sẽ dùng mock data để test logic camera và điều khiển
- Để có dữ liệu LiDAR thật, đảm bảo LiDAR đã kết nối và port đúng

### Tùy chọn tham số

```bash
python3 autonomous_drive_python.py \
    --camera-device 0              # Device ID camera (mặc định: 0)
    --lidar-port /dev/ttyUSB0      # Port LiDAR (mặc định: /dev/ttyUSB0)
    --arduino-port /dev/ttyACM0    # Port Arduino (None = auto detect)
    --show-display                 # Hiển thị camera và thông tin debug
    --no-camera                    # Tắt camera (chỉ dùng LiDAR)
    --min-distance 0.5             # Khoảng cách tối thiểu để dừng (m)
    --safe-distance 0.8            # Khoảng cách an toàn để tránh (m)
    --max-linear-speed 0.3         # Tốc độ tối đa tiến/lùi (m/s)
    --max-angular-speed 1.0        # Tốc độ quay tối đa (rad/s)
    --front-angle-range 60         # Góc phía trước để kiểm tra (degrees)
```

### Logic điều khiển

Script sử dụng logic điều khiển thông minh:

1. **Ưu tiên 1 (CAO - Safety)**: Tránh vật cản (LiDAR)
   - Vật cản bên trái → Quay phải
   - Vật cản bên phải → Quay trái
   - Vật cản ở giữa → Lùi và quay

2. **Ưu tiên 2 (THẤP - Navigation)**: Đi theo vạch kẻ đường (Camera)
   - Phát hiện vạch kẻ đường → Điều chỉnh để đi giữa đường
   - Không phát hiện vạch → Đi thẳng

3. **Điều khiển Ackermann**: 
   - Motor DC: Tốc độ tiến/lùi (từ `linear`)
   - Servo: Góc quay bánh lái (từ `angular` qua công thức Ackermann)

### Điều khiển khi chạy

- `q`: Thoát và dừng robot
- `s`: Dừng robot ngay lập tức (không thoát)

## 📊 Hiển thị thông tin

Script sẽ in thông tin mỗi 0.5 giây:

```
⏱️  Loop: 123
   📡 LiDAR: ✅ OK
   📷 Camera: ✅ Vạch, Offset: 0.15
   🎮 Control: linear=0.30 m/s, angular=-0.12 rad/s
```

Hoặc khi có vật cản:

```
⏱️  Loop: 456
   📡 LiDAR: ⚠️ VẬT CẢN
      Hướng: PHẢI
   📷 Camera: ✅ Vạch, Offset: -0.08
   🎮 Control: linear=0.18 m/s, angular=0.70 rad/s
```

Nếu bật `--show-display`, sẽ có cửa sổ hiển thị camera với thông tin overlay.

## 🔧 Troubleshooting

### Camera không hoạt động

```bash
# Kiểm tra device
ls -l /dev/video*

# Cấp quyền
sudo chmod 777 /dev/video0

# Test camera đơn giản
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"
```

### LiDAR không hoạt động

```bash
# Kiểm tra port
ls -l /dev/ttyUSB*

# Cấp quyền
sudo chmod 666 /dev/ttyUSB0

# Kiểm tra kết nối
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1); print('OK' if s.is_open else 'FAIL')"
```

### Arduino không nhận lệnh

```bash
# Kiểm tra port
ls -l /dev/ttyACM*

# Cấp quyền
sudo chmod 666 /dev/ttyACM0

# Kiểm tra Arduino Serial Monitor
# Phải thấy "READY" khi Arduino khởi động

# Test kết nối
python3 test_arduino.py --port /dev/ttyACM0
```

### Script không chạy

```bash
# Kiểm tra Python version
python3 --version  # Phải >= 3.8

# Kiểm tra dependencies
python3 -c "import cv2, numpy, serial; print('OK')"

# Cấp quyền thực thi
chmod +x autonomous_drive_python.py

# Kiểm tra đường dẫn
pwd  # Phải ở đúng thư mục chứa script
```

### Robot không di chuyển

1. **Kiểm tra Arduino có nhận lệnh không:**
   - Chạy `test_arduino.py` và test thủ công
   - Kiểm tra Serial Monitor trên Arduino IDE

2. **Kiểm tra motor driver:**
   - Kiểm tra nguồn cấp cho motor
   - Kiểm tra kết nối motor driver (L298N/TB6612)
   - Kiểm tra pins trên Arduino (2, 3, 5)

3. **Kiểm tra servo:**
   - Kiểm tra nguồn 5V cho servo
   - Kiểm tra pin 9 trên Arduino
   - Test servo bằng cách gửi các góc khác nhau

4. **Kiểm tra log:**
   - Xem log trong terminal khi chạy script
   - Kiểm tra có lỗi gì không

## 📝 Cấu hình tham số

Các tham số có thể điều chỉnh khi chạy script:

| Tham số | Mặc định | Mô tả |
|---------|----------|-------|
| `--min-distance` | 0.5 | Khoảng cách tối thiểu để dừng (m) |
| `--safe-distance` | 0.8 | Khoảng cách an toàn để bắt đầu tránh (m) |
| `--max-linear-speed` | 0.3 | Tốc độ tối đa tiến/lùi (m/s) |
| `--max-angular-speed` | 1.0 | Tốc độ quay tối đa (rad/s) |
| `--front-angle-range` | 60 | Góc phía trước để kiểm tra vật cản (degrees) |

**Ví dụ điều chỉnh cho môi trường hẹp:**

```bash
python3 autonomous_drive_python.py \
    --safe-distance 0.5 \
    --max-linear-speed 0.2 \
    --front-angle-range 90 \
    --show-display
```

## 🎯 Workflow khuyến nghị

1. **Test từng phần cứng:**
   ```bash
   # Test Camera
   python3 test_camera_full.py
   
   # Test Arduino
   python3 test_arduino.py --port /dev/ttyACM0
   
   # Test LiDAR
   python3 test_lidar_a1m8.py --port /dev/ttyUSB0 --use-serial
   ```

2. **Chạy autonomous drive:**
   ```bash
   # Chạy với đầy đủ Camera + LiDAR + Arduino
   python3 autonomous_drive_python.py \
       --camera-device 0 \
       --lidar-port /dev/ttyUSB0 \
       --arduino-port /dev/ttyACM0 \
       --show-display
   ```

3. **Kiểm tra và điều chỉnh:**
   - Quan sát log trong terminal
   - Xem camera display (nếu bật `--show-display`)
   - Điều chỉnh tham số phù hợp với môi trường

## 📚 Tài liệu tham khảo

- **Arduino Code**: Xem `arduino/README.md`
- **Code Scripts**: Xem trong thư mục `xe_lidar/scripts/`

## ⚠️ Lưu ý an toàn

1. **Luôn test trong môi trường an toàn** trước khi chạy tự động
2. **Kiểm tra robot** trước khi bật tự lái
3. **Sẵn sàng dừng** - nhấn `Ctrl+C` hoặc `q` để dừng ngay
4. **Đảm bảo không gian** đủ rộng để robot di chuyển
5. **Giám sát liên tục** - không để robot chạy một mình

## 🎮 Tóm tắt lệnh nhanh

```bash
# Test Camera
python3 test_camera_full.py

# Test Arduino  
python3 test_arduino.py --port /dev/ttyACM0

# Test LiDAR
python3 test_lidar_a1m8.py --port /dev/ttyUSB0 --use-serial

# Chạy Autonomous Drive
python3 autonomous_drive_python.py \
    --camera-device 0 \
    --lidar-port /dev/ttyUSB0 \
    --arduino-port /dev/ttyACM0 \
    --show-display
```

