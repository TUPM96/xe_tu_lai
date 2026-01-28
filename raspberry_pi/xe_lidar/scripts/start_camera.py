#!/usr/bin/env python3
"""
Script khoi dong Camera rieng
Su dung: python3 start_camera.py [--device /dev/video0] [--width 640] [--height 480] [--fps 30]
"""

import subprocess
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong Camera')
    parser.add_argument('--device', default='/dev/video0', help='Video device')
    parser.add_argument('--width', type=int, default=0, help='Chieu rong anh (0 = full resolution)')
    parser.add_argument('--height', type=int, default=0, help='Chieu cao anh (0 = full resolution)')
    parser.add_argument('--fps', type=int, default=30, help='Frame per second')
    args = parser.parse_args()

    print("=" * 50)
    print("  KHOI DONG CAMERA")
    print("=" * 50)
    print(f"Video Device: {args.device}")
    if args.width > 0 and args.height > 0:
        print(f"Resolution: {args.width}x{args.height} @ {args.fps}fps")
    else:
        print(f"Resolution: AUTO (full) @ {args.fps}fps")
    print("=" * 50)

    # Kiem tra device ton tai
    if not os.path.exists(args.device):
        print(f"[WARN] Device {args.device} khong ton tai!")

    # Chay ROS2 launch
    cmd = [
        'ros2', 'launch', 'xe_lidar', 'camera.launch.py',
        f'video_device:={args.device}',
        f'width:={args.width}',
        f'height:={args.height}',
        f'fps:={args.fps}'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung Camera.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay Camera: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
