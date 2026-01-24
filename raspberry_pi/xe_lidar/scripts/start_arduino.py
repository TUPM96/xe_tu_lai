#!/usr/bin/env python3
"""
Script khoi dong Arduino Bridge rieng
Su dung: python3 start_arduino.py [--port /dev/ttyACM0] [--baudrate 115200]
"""

import subprocess
import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong Arduino Bridge')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port cua Arduino')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baudrate')
    parser.add_argument('--auto-detect', action='store_true', default=True, help='Tu dong tim Arduino')
    args = parser.parse_args()

    print("=" * 50)
    print("  KHOI DONG ARDUINO BRIDGE")
    print("=" * 50)
    print(f"Serial Port: {args.port}")
    print(f"Baudrate: {args.baudrate}")
    print(f"Auto Detect: {args.auto_detect}")
    print("=" * 50)

    # Kiem tra port ton tai
    if not os.path.exists(args.port):
        print(f"[WARN] Port {args.port} khong ton tai!")
        if args.auto_detect:
            print("[INFO] Se tu dong tim Arduino...")

    # Chay ROS2 node truc tiep
    cmd = [
        'ros2', 'run', 'xe_lidar', 'arduino_bridge.py',
        '--ros-args',
        '-p', f'serial_port:={args.port}',
        '-p', f'baudrate:={args.baudrate}',
        '-p', f'auto_detect:={str(args.auto_detect).lower()}'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung Arduino Bridge.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay Arduino Bridge: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
