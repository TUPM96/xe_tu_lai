#!/usr/bin/env python3
"""
=============================================================================
SCRIPT CHẠY XE TỰ LÁI - TẤT CẢ TRONG 1 LỆNH
=============================================================================
Khởi động tất cả nodes: Camera, LiDAR, Arduino Bridge, Autonomous Drive

Sử dụng:
  python3 start_autonomous.py                          # Chạy với giá trị mặc định
  python3 start_autonomous.py --speed 0.5              # Tốc độ 0.5 m/s
  python3 start_autonomous.py --speed 0.3 --smooth 0.8 # Tốc độ 0.3, smoothing 0.8
  python3 start_autonomous.py --help                   # Xem hướng dẫn
=============================================================================
"""

import argparse
import subprocess
import sys


def print_banner():
    """In banner khởi động"""
    print("=" * 65)
    print("              🚗 KHỞI ĐỘNG XE TỰ LÁI 🚗                        ")
    print("=" * 65)


def print_config(args):
    """In cấu hình hiện tại"""
    print("\n📊 CẤU HÌNH:")
    print(f"   Tốc độ tối đa:    {args.speed} m/s")
    print(f"   PWM tối thiểu:    {args.pwm}")
    print(f"   Lane threshold:   {args.threshold}")
    print(f"   Smoothing:        {args.smooth}")
    print(f"   Dead zone:        {args.deadzone}")
    print(f"   PID:              Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    print("\n📡 THIẾT BỊ:")
    print(f"   LiDAR:            {args.lidar}")
    print(f"   Arduino:          {args.arduino}")
    print(f"   Camera:           {args.camera}")
    print("=" * 65)


def main():
    parser = argparse.ArgumentParser(
        description='Chạy xe tự lái với tất cả tham số cấu hình',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ví dụ:
  # Chạy với giá trị mặc định
  python3 start_autonomous.py

  # Tốc độ nhanh hơn
  python3 start_autonomous.py --speed 0.5 --pwm 120

  # Giảm giật, tăng smoothing
  python3 start_autonomous.py --smooth 0.85 --deadzone 0.1

  # Chỉ nhận vạch đen đậm (loại bỏ xám)
  python3 start_autonomous.py --threshold 35

  # Điều chỉnh PID
  python3 start_autonomous.py --kp 0.7 --ki 0.01 --kd 0.1

  # Đổi port thiết bị
  python3 start_autonomous.py --lidar /dev/ttyUSB1 --arduino /dev/ttyACM1
        """
    )

    # Tham số tốc độ
    speed_group = parser.add_argument_group('Tham số tốc độ')
    speed_group.add_argument('--speed', '-s', type=float, default=0.3,
                            help='Tốc độ tối đa (m/s) [mặc định: 0.3]')
    speed_group.add_argument('--pwm', '-p', type=int, default=100,
                            help='PWM tối thiểu motor (0-255) [mặc định: 100]')

    # Tham số lane detection
    lane_group = parser.add_argument_group('Tham số lane detection')
    lane_group.add_argument('--threshold', '-t', type=int, default=25,
                           help='Ngưỡng C cho lane detection (cao hơn = chỉ nhận đen) [mặc định: 25]')
    lane_group.add_argument('--smooth', type=float, default=0.7,
                           help='Hệ số làm mượt (0.0-0.95, cao hơn = mượt hơn) [mặc định: 0.7]')
    lane_group.add_argument('--deadzone', type=float, default=0.05,
                           help='Vùng chết offset (bỏ qua dao động nhỏ) [mặc định: 0.05]')

    # Tham số PID
    pid_group = parser.add_argument_group('Tham số PID')
    pid_group.add_argument('--kp', type=float, default=0.5,
                          help='Hệ số P [mặc định: 0.5]')
    pid_group.add_argument('--ki', type=float, default=0.0,
                          help='Hệ số I [mặc định: 0.0]')
    pid_group.add_argument('--kd', type=float, default=0.0,
                          help='Hệ số D [mặc định: 0.0]')

    # Tham số thiết bị
    device_group = parser.add_argument_group('Tham số thiết bị')
    device_group.add_argument('--lidar', type=str, default='/dev/ttyUSB0',
                             help='Port LiDAR [mặc định: /dev/ttyUSB0]')
    device_group.add_argument('--arduino', type=str, default='/dev/ttyACM0',
                             help='Port Arduino [mặc định: /dev/ttyACM0]')
    device_group.add_argument('--camera', type=str, default='/dev/video0',
                             help='Device camera [mặc định: /dev/video0]')

    args = parser.parse_args()

    # In banner và cấu hình
    print_banner()
    print_config(args)

    print("\n🚀 Đang khởi động tất cả nodes...")
    print("   - Robot State Publisher")
    print("   - Joint State Publisher")
    print("   - RPLIDAR Node")
    print("   - Camera Node")
    print("   - Arduino Bridge")
    print("   - Autonomous Drive")
    print("\n   Nhấn Ctrl+C để dừng\n")

    # Tạo lệnh ros2 launch với tất cả tham số
    cmd = [
        'ros2', 'launch', 'xe_lidar', 'autonomous_drive_arduino.launch.py',
        f'max_linear_speed:={args.speed}',
        f'motor_min_pwm:={args.pwm}',
        f'lane_threshold_c:={args.threshold}',
        f'lane_offset_smoothing:={args.smooth}',
        f'lane_dead_zone:={args.deadzone}',
        f'kp:={args.kp}',
        f'ki:={args.ki}',
        f'kd:={args.kd}',
        f'lidar_serial_port:={args.lidar}',
        f'arduino_serial_port:={args.arduino}',
        f'video_device:={args.camera}',
    ]

    try:
        # Chạy lệnh
        process = subprocess.run(cmd)
        sys.exit(process.returncode)
    except KeyboardInterrupt:
        print("\n\n⏹️  Đã dừng xe tự lái")
        sys.exit(0)
    except FileNotFoundError:
        print("\n❌ Lỗi: Không tìm thấy ros2")
        print("💡 Hãy chắc chắn đã source ROS2:")
        print("   source /opt/ros/jazzy/setup.bash")
        print("   source ~/ros2_ws/install/setup.bash")
        sys.exit(1)


if __name__ == '__main__':
    main()
