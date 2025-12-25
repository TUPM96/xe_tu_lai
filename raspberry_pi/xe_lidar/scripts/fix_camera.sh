#!/bin/bash
# Script sửa lỗi camera USB bị nhảy hình

echo "🔧 Đang sửa lỗi camera USB..."

# Kiểm tra camera device
echo "📷 Kiểm tra camera device..."
ls -l /dev/video* 2>/dev/null || echo "⚠️  Không tìm thấy /dev/video*"

# Cài đặt GStreamer plugins
echo "📦 Cài đặt GStreamer plugins..."
sudo aptitude update
sudo aptitude install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    v4l-utils \
    v4l2-utils \
    cheese

# Cài đặt v4l2-utils nếu chưa có
if ! command -v v4l2-ctl &> /dev/null; then
    echo "📦 Cài đặt v4l2-utils..."
    sudo aptitude install -y v4l-utils
fi

# Kiểm tra thông tin camera
if [ -e /dev/video0 ]; then
    echo "📊 Thông tin camera:"
    v4l2-ctl --device=/dev/video0 --all | head -20
    
    echo ""
    echo "📋 Formats hỗ trợ:"
    v4l2-ctl --device=/dev/video0 --list-formats-ext | head -30
    
    echo ""
    echo "🔧 Đang cấu hình camera với format MJPEG..."
    v4l2-ctl --device=/dev/video0 \
        --set-fmt-video=width=640,height=480,pixelformat=MJPG 2>/dev/null || \
    v4l2-ctl --device=/dev/video0 \
        --set-fmt-video=width=640,height=480,pixelformat=YUYV
    
    echo "✅ Đã cấu hình camera"
else
    echo "⚠️  Camera /dev/video0 không tồn tại"
fi

# Cấp quyền
echo "🔐 Cấp quyền truy cập camera..."
sudo usermod -a -G video $USER
sudo chmod 666 /dev/video0 2>/dev/null || echo "⚠️  Không thể cấp quyền /dev/video0"

echo ""
echo "✅ Hoàn tất!"
echo ""
echo "📝 Lưu ý:"
echo "   - Logout và login lại để áp dụng group changes"
echo "   - Test camera: cheese"
echo "   - Test với ROS2: ros2 launch xe_lidar camera.launch.py video_device:=/dev/video0 pixel_format:=MJPG"

