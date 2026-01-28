#!/usr/bin/env python3
"""
Script khoi dong Lane Detection Node rieng
Su dung: ros2 run xe_lidar start_lane_detection.py [--camera-topic /camera/image_raw] [--debug]
"""

import subprocess
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong Lane Detection Node')
    parser.add_argument('--camera-topic', type=str, default='/camera/image_raw',
                        help='Topic nhan anh camera (default: /camera/image_raw)')
    parser.add_argument('--debug', action='store_true', default=True,
                        help='Hien thi debug output (default: True)')
    parser.add_argument('--no-debug', action='store_true',
                        help='Tat debug output')
    parser.add_argument('--roi-top', type=float, default=0.4,
                        help='ROI bat dau tu %% chieu cao (default: 0.4)')
    parser.add_argument('--lane-width', type=int, default=200,
                        help='Khoang cach uoc tinh giua 2 vach (pixels, default: 200)')
    parser.add_argument('--full-image', action='store_true',
                        help='Dung full anh camera (khong crop ROI)')
    args = parser.parse_args()

    debug_camera = not args.no_debug

    print("=" * 50)
    print("  KHOI DONG LANE DETECTION NODE")
    print("=" * 50)
    print(f"Camera Topic: {args.camera_topic}")
    print(f"Debug Camera: {debug_camera}")
    if args.full_image:
        print(f"Image Mode: FULL (khong crop)")
    else:
        print(f"ROI Top: {args.roi_top * 100:.0f}%")
    print(f"Lane Width: {args.lane_width} pixels")
    print("=" * 50)
    print()
    print("Publishers:")
    print("  /lane_detection/image_debug - Anh debug voi vach ve")
    print("  /lane_detection/offset      - Offset tu giua (-1 den 1)")
    print("  /lane_detection/detected    - Co phat hien lane khong")
    print("=" * 50)

    # Chay ROS2 node
    cmd = [
        'ros2', 'run', 'xe_lidar', 'lane_detection_node.py',
        '--ros-args',
        '-p', f'camera_topic:={args.camera_topic}',
        '-p', f'debug_camera:={str(debug_camera).lower()}',
        '-p', f'roi_top_percent:={args.roi_top}',
        '-p', f'lane_width_pixels:={args.lane_width}',
        '-p', f'use_full_image:={str(args.full_image).lower()}'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung Lane Detection Node.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay Lane Detection: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
