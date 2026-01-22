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

# Fix rviz2 trên Raspberry Pi (software rendering) - chỉ cho rviz2, không ảnh hưởng xrdp
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

### Bước 4: Cài đặt xrdp (Remote Desktop)

```bash
# Gỡ VNC nếu đã cài (nếu có)
# 1. Kill các VNC server đang chạy
vncserver -kill :1 2>/dev/null
vncserver -kill :2 2>/dev/null
pkill -9 vncserver 2>/dev/null

# 2. Dừng và tắt systemd services
sudo systemctl stop vncserver@1.service 2>/dev/null
sudo systemctl stop vncserver@2.service 2>/dev/null
sudo systemctl disable vncserver@1.service 2>/dev/null
sudo systemctl disable vncserver@2.service 2>/dev/null
sudo rm -f /etc/systemd/system/vncserver@.service
sudo systemctl daemon-reload

# 3. Xóa VNC auto-start trong .bashrc, .profile
# Kiểm tra và xóa dòng có vncserver trong .bashrc
sed -i '/vncserver/d' ~/.bashrc 2>/dev/null
sed -i '/vncserver/d' ~/.profile 2>/dev/null
sed -i '/vncserver/d' ~/.bash_profile 2>/dev/null

# 4. Xóa VNC khỏi autostart (nếu có)
rm -f ~/.config/autostart/*vnc* 2>/dev/null

# 5. Xóa cron jobs chạy VNC
crontab -l 2>/dev/null | grep -v vncserver | crontab - 2>/dev/null

# 6. Gỡ package VNC
sudo apt remove -y tightvncserver tigervnc-* vnc4server x11vnc 2>/dev/null
sudo apt autoremove -y

# 7. Xóa thư mục VNC config (tùy chọn)
# rm -rf ~/.vnc

# Cài đặt xrdp và desktop environment (XFCE - nhẹ, phù hợp Raspberry Pi)
sudo aptitude install -y xrdp xfce4 xfce4-goodies

# Tạo file .xsession để xrdp chạy XFCE desktop
echo "xfce4-session" > ~/.xsession
chmod +x ~/.xsession

# Hoặc nếu muốn dùng desktop mặc định của hệ thống:
# cat > ~/.xsession << 'EOF'
# #!/bin/bash
# unset DBUS_SESSION_BUS_ADDRESS
# if [ -f /etc/X11/Xsession ]; then
#     exec /etc/X11/Xsession
# fi
# EOF
# chmod +x ~/.xsession

# Restart xrdp
sudo systemctl restart xrdp
sudo systemctl enable xrdp

# Mở firewall port cho xrdp (port 3389)
sudo ufw allow 3389/tcp

# Kiểm tra xrdp đang chạy
sudo systemctl status xrdp

# Kiểm tra port đã mở
sudo netstat -tlnp | grep 3389
```

**Kết nối xrdp:**
- Địa chỉ: `IP_Raspberry_Pi` (ví dụ: `192.168.137.219`)
- Dùng Remote Desktop Connection (Windows) hoặc Remmina (Linux)
- Port: `3389` (mặc định)
- Khi connect, chọn session: **"Xorg"** (không phải "Xvnc")
- Username: tên user của bạn
- Password: password của bạn

**Nếu gặp màn hình xanh (không có desktop) khi kết nối xrdp:**
```bash
# 1. Kiểm tra desktop environment đã cài chưa
ls /usr/share/xsessions/
# Phải thấy có file như: xfce.desktop, xfce4.desktop, etc.

# 2. Nếu chưa có XFCE, cài đặt:
sudo apt update
sudo apt install -y xfce4 xfce4-goodies

# 3. Đảm bảo file .xsession đúng:
cat ~/.xsession
# Phải có: xfce4-session

# 4. Nếu chưa đúng hoặc file không tồn tại, tạo lại:
echo "xfce4-session" > ~/.xsession
chmod +x ~/.xsession

# 5. Kiểm tra file có quyền thực thi
ls -la ~/.xsession

# 6. Thêm xrdp vào group ssl-cert (nếu cần)
sudo adduser xrdp ssl-cert

# 7. Restart xrdp
sudo systemctl restart xrdp

# 8. Kiểm tra xrdp đang chạy
sudo systemctl status xrdp

# 9. Kiểm tra log để xem lỗi (nếu vẫn không được)
sudo tail -50 /var/log/xrdp-sesman.log
sudo tail -50 /var/log/xrdp.log

# 10. Nếu gặp lỗi "X server failed to start" hoặc "Timed out waiting for X server":
# Kiểm tra đã cài xorgxrdp chưa (QUAN TRỌNG!)
dpkg -l | grep xorgxrdp

# Nếu chưa có, cài đặt xorgxrdp:
sudo apt update
sudo apt install -y xorgxrdp

# Kiểm tra xorgxrdp config:
ls -la /etc/xrdp/xorg.conf
# Nếu không có file, thử:
sudo find /usr -name "xorg.conf" 2>/dev/null

# 11. Kiểm tra log X server để xem lỗi chi tiết:
ls -lt ~/.xorgxrdp.*.log | head -1
cat $(ls -t ~/.xorgxrdp.*.log | head -1) | tail -100

# 12. Nếu X server vẫn không khởi động, kiểm tra log X server chi tiết:
# Tìm file log mới nhất (sau khi đã kết nối và bị lỗi)
ls -lt ~/.xorgxrdp.*.log 2>/dev/null | head -1

# Xem log X server để tìm lỗi cụ thể
cat $(ls -t ~/.xorgxrdp.*.log 2>/dev/null | head -1) 2>/dev/null | tail -100

# Hoặc xem tất cả log X server
cat ~/.xorgxrdp.*.log 2>/dev/null | grep -i error | tail -20

# 13. Nếu log cho thấy thiếu driver hoặc module, cài thêm packages:
sudo apt install -y xserver-xorg-core xserver-xorg-input-all xserver-xorg-video-dummy

# 14. Nếu vẫn lỗi, thử tạo xorg.conf manually:
sudo mkdir -p /etc/xrdp
sudo nano /etc/xrdp/xorg.conf
# Paste nội dung xorg.conf cơ bản (xem bên dưới)

# 15. Nếu log X server cho thấy lỗi "Cannot open /dev/tty0 (Permission denied)":
# Tạo file xorg.conf để bỏ qua /dev/tty0:
sudo mkdir -p /etc/xrdp
sudo tee /etc/xrdp/xorg.conf > /dev/null << 'EOF'
Section "ServerFlags"
    Option "DontVTSwitch" "true"
    Option "DontZap" "true"
    Option "AllowMouseOpenFail" "true"
    Option "AutoAddDevices" "true"
    Option "AutoEnableDevices" "true"
EndSection

Section "ServerLayout"
    Identifier "XServer0"
    Screen "Screen0" 0 0
EndSection

Section "Monitor"
    Identifier "Monitor0"
    HorizSync 28.0-80.0
    VertRefresh 48.0-75.0
    Modeline "1920x1080_60.00" 173.00 1920 2048 2248 2576 1080 1083 1088 1120 -hsync +vsync
EndSection

Section "Device"
    Identifier "Device0"
    Driver "modesetting"
    Option "AccelMethod" "none"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device "Device0"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Modes "1920x1080"
    EndSubSection
EndSection
EOF

# 16. Nếu log vẫn báo "Unable to locate/open config file: xrdp/xorg.conf":
# Xrdp tìm file ở ~/.config/xrdp/xorg.conf hoặc /etc/xrdp/xorg.conf
# Kiểm tra xrdp tìm ở đâu:
sudo grep -r "xorg.conf" /etc/xrdp/ 2>/dev/null

# Tạo file ở cả 2 nơi để chắc chắn:
sudo mkdir -p /etc/xrdp
sudo mkdir -p ~/.config/xrdp

# Copy file config vào cả 2 nơi
sudo cp /etc/xrdp/xorg.conf ~/.config/xrdp/xorg.conf 2>/dev/null || true
sudo cp ~/.config/xrdp/xorg.conf /etc/xrdp/xorg.conf 2>/dev/null || true

# 17. Nếu vẫn lỗi "Cannot open virtual console", tạo xorg.conf với DontVTSwitch:
sudo tee /etc/xrdp/xorg.conf > /dev/null << 'EOF'
Section "ServerFlags"
    Option "DontVTSwitch" "true"
    Option "DontZap" "true"
    Option "DontZoom" "true"
    Option "AllowMouseOpenFail" "true"
    Option "AutoAddDevices" "true"
    Option "AutoEnableDevices" "true"
    Option "UseDefaultFontPath" "true"
EndSection

Section "ServerLayout"
    Identifier "XServer0"
    Screen "Screen0" 0 0
    Option "AutoAddDevices" "true"
    Option "AutoEnableDevices" "true"
EndSection

Section "Monitor"
    Identifier "Monitor0"
    HorizSync 28.0-80.0
    VertRefresh 48.0-75.0
EndSection

Section "Device"
    Identifier "Device0"
    Driver "modesetting"
    Option "AccelMethod" "none"
    Option "ShadowFB" "true"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device "Device0"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Virtual 1920 1080
    EndSubSection
EndSection
EOF

# Copy vào home directory cũng
cp /etc/xrdp/xorg.conf ~/.config/xrdp/xorg.conf
chmod 644 /etc/xrdp/xorg.conf
chmod 644 ~/.config/xrdp/xorg.conf

# 19. QUAN TRỌNG: Kiểm tra xrdp tìm xorg.conf ở đâu:
sudo grep -r "xorg.conf" /etc/xrdp/ 2>/dev/null
# Nếu thấy "param=xrdp/xorg.conf", có nghĩa là xrdp tìm file ở đường dẫn tương đối
# File cần được tạo ở: ~/xrdp/xorg.conf (trong home directory)

# 20. Tạo file xorg.conf ở đúng chỗ (trong thư mục xrdp trong home):
mkdir -p ~/xrdp
cp /etc/xrdp/xorg.conf ~/xrdp/xorg.conf 2>/dev/null || \
sudo tee ~/xrdp/xorg.conf > /dev/null << 'EOF'
Section "ServerFlags"
    Option "DontVTSwitch" "true"
    Option "DontZap" "true"
    Option "DontZoom" "true"
    Option "AllowMouseOpenFail" "true"
    Option "AutoAddDevices" "true"
    Option "AutoEnableDevices" "true"
EndSection

Section "ServerLayout"
    Identifier "XServer0"
    Screen "Screen0" 0 0
EndSection

Section "Monitor"
    Identifier "Monitor0"
    HorizSync 28.0-80.0
    VertRefresh 48.0-75.0
EndSection

Section "Device"
    Identifier "Device0"
    Driver "modesetting"
    Option "AccelMethod" "none"
    Option "ShadowFB" "true"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device "Device0"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Virtual 1920 1080
    EndSubSection
EndSection
EOF

chmod 644 ~/xrdp/xorg.conf
ls -la ~/xrdp/xorg.conf

# 22. Nếu log cho thấy "Using config file: xrdp/xorg.conf" nhưng vẫn lỗi virtual console:
# Kiểm tra file config đã có DontVTSwitch chưa:
grep -i "DontVTSwitch" ~/xrdp/xorg.conf

# Nếu không có hoặc không đúng, sửa lại file:
cat > ~/xrdp/xorg.conf << 'EOF'
Section "ServerFlags"
    Option "DontVTSwitch" "true"
    Option "DontZap" "true"
    Option "DontZoom" "true"
    Option "AllowMouseOpenFail" "true"
    Option "AutoAddDevices" "true"
    Option "AutoEnableDevices" "true"
EndSection

Section "ServerLayout"
    Identifier "XServer0"
    Screen "Screen0" 0 0
EndSection

Section "Monitor"
    Identifier "Monitor0"
    HorizSync 28.0-80.0
    VertRefresh 48.0-75.0
EndSection

Section "Device"
    Identifier "Device0"
    Driver "modesetting"
    Option "AccelMethod" "none"
    Option "ShadowFB" "true"
EndSection

Section "Screen"
    Identifier "Screen0"
    Device "Device0"
    Monitor "Monitor0"
    DefaultDepth 24
    SubSection "Display"
        Depth 24
        Virtual 1920 1080
    EndSubSection
EndSection
EOF

chmod 644 ~/xrdp/xorg.conf

# 23. Hoặc thử thêm user vào group (nhưng thường không cần nếu đã có DontVTSwitch):
sudo usermod -a -G video $USER
sudo usermod -a -G tty $USER

# 17. Restart xrdp và test lại:
sudo systemctl restart xrdp

# 18. Debug: Xem log chi tiết để tìm lỗi:
# Xem log xrdp-sesman (log chính):
sudo tail -100 /var/log/xrdp-sesman.log

# Xem log xrdp:
sudo tail -100 /var/log/xrdp.log

# Xem log X server (sau khi kết nối và bị lỗi):
cat $(ls -t ~/.xorgxrdp.*.log 2>/dev/null | head -1) 2>/dev/null

# Xem chỉ các dòng ERROR:
grep -i "EE\|ERROR\|Fatal" ~/.xorgxrdp.*.log 2>/dev/null

# Xem log real-time khi kết nối (chạy trước khi kết nối):
# Terminal 1:
sudo tail -f /var/log/xrdp-sesman.log

# Terminal 2:
tail -f ~/.xorgxrdp.*.log 2>/dev/null

# Xem tất cả log liên quan đến xrdp:
sudo journalctl -u xrdp -n 100 --no-pager
sudo journalctl -u xrdp-sesman -n 100 --no-pager
```

**Lưu ý khi kết nối xrdp:**
- Khi login, ở màn hình chọn session, chọn **"Xorg"** (KHÔNG chọn "Xvnc")
- Nếu không thấy menu chọn session, thử logout và login lại
- Đảm bảo username và password đúng

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

