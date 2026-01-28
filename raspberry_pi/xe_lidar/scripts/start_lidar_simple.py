#!/usr/bin/env python3
"""
Script đơn giản để chỉ chạy LiDAR - không cần nhiều tham số
Sử dụng: python3 start_lidar_simple.py [--port /dev/ttyUSB0]
"""

import subprocess
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Chạy LiDAR đơn giản')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port của LiDAR (mặc định: /dev/ttyUSB0)')
    args = parser.parse_args()

    print("=" * 60)
    print("🚀 KHỞI ĐỘNG LIDAR ĐƠN GIẢN")
    print("=" * 60)
    print(f"📡 Serial Port: {args.port}")
    print("=" * 60)

    # Kiểm tra port tồn tại
    if not os.path.exists(args.port):
        print(f"⚠️  [WARN] Port {args.port} không tồn tại!")
        print("   Đảm bảo LiDAR đã được kết nối và có quyền truy cập.")
        response = input("   Tiếp tục? (y/n): ")
        if response.lower() != 'y':
            sys.exit(1)

    # Chạy ROS2 launch với tham số tối thiểu
    # Có thể dùng rplidar_simple.launch.py hoặc rplidar.launch.py
    cmd = [
        'ros2', 'launch', 'xe_lidar', 'rplidar_simple.launch.py',
        f'serial_port:={args.port}'
    ]

    print("\n📊 LiDAR sẽ publish dữ liệu vào topic: /scan")
    print("💡 Để xem dữ liệu: ros2 topic echo /scan")
    print("💡 Để visualize: rviz2\n")

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n✅ Đã dừng LiDAR.")
    except subprocess.CalledProcessError as e:
        print(f"❌ Lỗi khi chạy LiDAR: {e}")
        sys.exit(1)
    except FileNotFoundError:
        print("❌ Không tìm thấy ros2. Đảm bảo ROS2 đã được cài đặt và source.")
        print("   Chạy: source /opt/ros/jazzy/setup.bash")
        sys.exit(1)


if __name__ == '__main__':
    main()
