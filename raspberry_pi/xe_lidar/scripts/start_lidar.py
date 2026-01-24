#!/usr/bin/env python3
"""
Script khoi dong LiDAR rieng
Su dung: python3 start_lidar.py [--port /dev/ttyUSB0] [--scan-mode Sensitivity]
"""

import subprocess
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong LiDAR')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port cua LiDAR')
    parser.add_argument('--scan-mode', default='Sensitivity', help='Che do quet')
    args = parser.parse_args()

    print("=" * 50)
    print("  KHOI DONG LIDAR")
    print("=" * 50)
    print(f"Serial Port: {args.port}")
    print(f"Scan Mode: {args.scan_mode}")
    print("=" * 50)

    # Kiem tra port ton tai
    if not os.path.exists(args.port):
        print(f"[WARN] Port {args.port} khong ton tai!")

    # Chay ROS2 launch
    cmd = [
        'ros2', 'launch', 'xe_lidar', 'rplidar.launch.py',
        f'serial_port:={args.port}',
        f'scan_mode:={args.scan_mode}'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung LiDAR.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay LiDAR: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
