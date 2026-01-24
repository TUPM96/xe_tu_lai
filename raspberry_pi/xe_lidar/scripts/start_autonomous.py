#!/usr/bin/env python3
"""
Script khoi dong Autonomous Drive (xu ly) rieng
Su dung: python3 start_autonomous.py [--use-camera] [--max-speed 1.0]
"""

import subprocess
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong Autonomous Drive')
    parser.add_argument('--use-camera', action='store_true', default=True, help='Su dung camera')
    parser.add_argument('--no-camera', action='store_true', help='Khong su dung camera')
    parser.add_argument('--max-speed', type=float, default=1.0, help='Toc do toi da (m/s)')
    parser.add_argument('--max-angular', type=float, default=1.0, help='Toc do quay toi da (rad/s)')
    parser.add_argument('--safe-distance', type=float, default=0.8, help='Khoang cach an toan (m)')
    args = parser.parse_args()

    use_camera = not args.no_camera

    print("=" * 50)
    print("  KHOI DONG AUTONOMOUS DRIVE (XU LY)")
    print("=" * 50)
    print(f"Use Camera: {use_camera}")
    print(f"Max Linear Speed: {args.max_speed} m/s")
    print(f"Max Angular Speed: {args.max_angular} rad/s")
    print(f"Safe Distance: {args.safe_distance} m")
    print("=" * 50)

    # Chay ROS2 node
    cmd = [
        'ros2', 'run', 'xe_lidar', 'obstacle_avoidance.py',
        '--ros-args',
        '-p', 'use_sim_time:=false',
        '-p', 'min_distance:=0.5',
        '-p', f'safe_distance:={args.safe_distance}',
        '-p', f'max_linear_speed:={args.max_speed}',
        '-p', f'max_angular_speed:={args.max_angular}',
        '-p', 'front_angle_range:=60',
        '-p', f'use_camera:={str(use_camera).lower()}',
        '-p', 'camera_topic:=/camera/image_raw'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung Autonomous Drive.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay Autonomous Drive: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
