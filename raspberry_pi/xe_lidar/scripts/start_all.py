#!/usr/bin/env python3
"""
Script chinh de khoi dong tat ca cac thanh phan
Su dung: python3 start_all.py

Cac script rieng le:
  python3 start_lidar.py      - Khoi dong LiDAR
  python3 start_camera.py     - Khoi dong Camera
  python3 start_arduino.py    - Khoi dong Arduino Bridge
  python3 start_autonomous.py - Khoi dong Autonomous Drive (xu ly)
"""

import subprocess
import argparse
import os
import sys
import time
import signal
from pathlib import Path


class RobotLauncher:
    def __init__(self):
        self.processes = []
        self.script_dir = Path(__file__).parent

    def check_device(self, device_path):
        """Kiem tra thiet bi co ton tai khong"""
        if os.path.exists(device_path):
            print(f"  [OK] {device_path}")
            return True
        else:
            print(f"  [WARN] {device_path} khong ton tai")
            return False

    def start_component(self, name, script_name, args=None):
        """Khoi dong mot thanh phan trong process rieng"""
        script_path = self.script_dir / script_name
        cmd = [sys.executable, str(script_path)]
        if args:
            cmd.extend(args)

        print(f"[START] {name}...")
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            self.processes.append((name, process))
            return process
        except Exception as e:
            print(f"[ERROR] Khong the khoi dong {name}: {e}")
            return None

    def stop_all(self):
        """Dung tat ca cac process"""
        print("\nDang dung tat ca cac thanh phan...")
        for name, process in self.processes:
            if process.poll() is None:  # Process van dang chay
                print(f"  Dung {name}...")
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
        print("Da dung tat ca.")

    def run(self, lidar_port, camera_device, arduino_port):
        """Chay tat ca cac thanh phan"""
        print("=" * 50)
        print("  KHOI DONG TAT CA THANH PHAN")
        print("=" * 50)
        print(f"LiDAR Port: {lidar_port}")
        print(f"Camera Device: {camera_device}")
        print(f"Arduino Port: {arduino_port}")
        print("=" * 50)
        print()

        # Kiem tra cac thiet bi
        print("Kiem tra cac thiet bi:")
        self.check_device(lidar_port)
        self.check_device(camera_device)
        self.check_device(arduino_port)
        print()

        # Khoi dong tung thanh phan
        print("[1/4] Khoi dong LiDAR...")
        self.start_component("LiDAR", "start_lidar.py", ['--port', lidar_port])
        time.sleep(2)

        print("[2/4] Khoi dong Camera...")
        self.start_component("Camera", "start_camera.py", ['--device', camera_device])
        time.sleep(2)

        print("[3/4] Khoi dong Arduino Bridge...")
        self.start_component("Arduino", "start_arduino.py", ['--port', arduino_port])
        time.sleep(2)

        print("[4/4] Khoi dong Autonomous Drive...")
        self.start_component("Autonomous", "start_autonomous.py")

        print()
        print("=" * 50)
        print("  TAT CA THANH PHAN DA KHOI DONG")
        print("=" * 50)
        print()
        print("Cac thanh phan dang chay:")
        print("  - LiDAR: /scan topic")
        print("  - Camera: /camera/image_raw topic")
        print("  - Arduino: nhan /cmd_vel, gui toi Arduino")
        print("  - Autonomous: xu ly va dieu khien")
        print()
        print("Nhan Ctrl+C de dung tat ca.")
        print()

        # Cho va in output tu cac process
        try:
            while True:
                all_stopped = True
                for name, process in self.processes:
                    if process.poll() is None:
                        all_stopped = False
                        # Doc output neu co
                        try:
                            line = process.stdout.readline()
                            if line:
                                print(f"[{name}] {line.strip()}")
                        except:
                            pass

                if all_stopped:
                    print("Tat ca cac process da dung.")
                    break

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.stop_all()


def main():
    parser = argparse.ArgumentParser(description='Khoi dong tat ca cac thanh phan')
    parser.add_argument('--lidar-port', default='/dev/ttyUSB0', help='Serial port cua LiDAR')
    parser.add_argument('--camera-device', default='/dev/video0', help='Video device')
    parser.add_argument('--arduino-port', default='/dev/ttyACM0', help='Serial port cua Arduino')
    args = parser.parse_args()

    launcher = RobotLauncher()

    # Xu ly Ctrl+C
    def signal_handler(sig, frame):
        launcher.stop_all()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    launcher.run(args.lidar_port, args.camera_device, args.arduino_port)


if __name__ == '__main__':
    main()
