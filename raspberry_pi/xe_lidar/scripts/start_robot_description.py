#!/usr/bin/env python3
"""
Script khoi dong Robot State Publisher (RSP) va Joint State Publisher
Publish robot description (URDF) va transforms
Su dung: python3 start_robot_description.py
"""

import subprocess
import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description='Khoi dong Robot State Publisher')
    parser.add_argument('--use-sim-time', action='store_true', default=False, help='Su dung sim time')
    args = parser.parse_args()

    print("=" * 50)
    print("  KHOI DONG ROBOT STATE PUBLISHER")
    print("=" * 50)
    print(f"Use Sim Time: {args.use_sim_time}")
    print("=" * 50)

    # Chay ROS2 launch file rsp_ackermann.launch.py
    cmd = [
        'ros2', 'launch', 'xe_lidar', 'rsp_ackermann.launch.py',
        f'use_sim_time:={str(args.use_sim_time).lower()}',
        'use_ros2_control:=false'
    ]

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\nDa dung Robot State Publisher.")
    except subprocess.CalledProcessError as e:
        print(f"Loi khi chay Robot State Publisher: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
