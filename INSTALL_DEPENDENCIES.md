# Cài đặt Dependencies cho ROS2 Jazzy

Hướng dẫn cài đặt tất cả các dependencies cần thiết cho dự án xe tự lái.

## Cài đặt nhanh (Tất cả dependencies)

```bash
# Cài đặt aptitude (nếu chưa có)
sudo apt install -y aptitude

# Cập nhật package list
sudo aptitude update

# Cài đặt tất cả dependencies
sudo aptitude install -y \
    ros-jazzy-desktop \
    ros-jazzy-cv-bridge \
    ros-jazzy-v4l2-camera \
    ros-jazzy-twist-mux \
    ros-jazzy-controller-manager \
    ros-jazzy-ackermann-msgs \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    python3-opencv \
    python3-numpy \
    python3-pip \
    python3-serial \
    gazebo
```

## Cài đặt từng phần

### 1. Dependencies cơ bản cho Camera

```bash
sudo aptitude install -y ros-jazzy-v4l2-camera
```

### 2. Dependencies cho điều khiển robot

```bash
sudo aptitude install -y \
    ros-jazzy-twist-mux \
    ros-jazzy-controller-manager \
    ros-jazzy-ackermann-msgs
```

### 3. Dependencies cho xử lý ảnh

```bash
sudo aptitude install -y \
    ros-jazzy-cv-bridge \
    python3-opencv \
    python3-numpy
```

### 4. Dependencies cho URDF/Xacro

```bash
sudo aptitude install -y \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher
```

### 5. Python packages

```bash
pip3 install pyserial
pip3 install "numpy<2.0"  # Quan trọng: cv_bridge chưa hỗ trợ NumPy 2.x
```

## Kiểm tra cài đặt

```bash
# Kiểm tra các package đã cài
ros2 pkg list | grep -E "v4l2_camera|cv_bridge|twist_mux|controller_manager"

# Kiểm tra Python packages
python3 -c "import cv2, numpy, serial; print('✅ Tất cả Python packages OK')"
```

## Troubleshooting

### Lỗi: package 'v4l2_camera' not found

```bash
# Cài đặt v4l2_camera
sudo aptitude install -y ros-jazzy-v4l2-camera

# Source lại ROS2 environment
source /opt/ros/jazzy/setup.bash
```

### Lỗi: package 'twist_mux' not found

```bash
sudo aptitude install -y ros-jazzy-twist-mux
source /opt/ros/jazzy/setup.bash
```

### Lỗi: package 'controller_manager' not found

```bash
sudo aptitude install -y ros-jazzy-controller-manager
source /opt/ros/jazzy/setup.bash
```

### Lỗi: NumPy version incompatible

```bash
# cv_bridge chỉ hỗ trợ NumPy < 2.0
pip3 uninstall numpy -y
pip3 install "numpy<2.0"
```

## Sau khi cài đặt

```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace (nếu đã build)
cd ~/ros2_ws
source install/setup.bash

# Test camera launch
ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0
```

