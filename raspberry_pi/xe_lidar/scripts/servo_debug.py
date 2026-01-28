#!/usr/bin/env python3
"""
Script debug servo: đẩy góc servo và set góc mặc định (thẳng).

- S:angle  -> set servo trực tiếp tới góc (độ), dùng để chỉnh/thử
- C:angle  -> đặt góc đó làm mặc định (center), từ giờ "thẳng" = angle độ

Chạy: python3 servo_debug.py [serial_port]
      ros2 run xe_lidar servo_debug.py -- /dev/ttyACM0
"""

import serial
import time
import sys
import argparse


def send_servo_angle(ser, angle):
    """Gửi S:angle - set servo trực tiếp (độ)."""
    angle = max(0, min(180, int(angle)))
    cmd = f"S:{angle}\n"
    ser.write(cmd.encode('utf-8'))
    ser.flush()
    return angle


def send_servo_center(ser, angle):
    """Gửi C:angle - đặt góc mặc định (center)."""
    angle = max(0, min(180, int(angle)))
    cmd = f"C:{angle}\n"
    ser.write(cmd.encode('utf-8'))
    ser.flush()
    return angle


def main():
    parser = argparse.ArgumentParser(
        description='Debug servo: đẩy góc và set góc mặc định (thẳng)'
    )
    parser.add_argument('port', nargs='?', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    args = parser.parse_args()

    port = args.port
    print("=" * 55)
    print("  DEBUG SERVO – Đẩy góc & set góc mặc định (thẳng)")
    print("=" * 55)
    print(f"Port: {port}")
    print()
    print("Lệnh:")
    print("  <số>        Ví dụ: 88   -> Đẩy servo tới 88 độ (S:88)")
    print("  c <số>      Ví dụ: c 88 -> Đặt 88 làm góc mặc định (C:88)")
    print("  + / -       Tăng/giảm 1 độ so với góc hiện tại")
    print("  q           Thoát")
    print("=" * 55)

    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=0.5)
        time.sleep(2)
        if ser.in_waiting > 0:
            ser.read_all()
        print("Đã kết nối Arduino.\n")
    except serial.SerialException as e:
        print(f"Lỗi kết nối: {e}")
        sys.exit(1)

    current_angle = 90  # Giả sử bắt đầu 90

    try:
        while True:
            line = input("Góc (độ) hoặc lệnh [c/số, +, -, q]: ").strip().lower()
            if not line:
                continue

            if line == 'q':
                break

            if line.startswith('c '):
                part = line[2:].strip()
                if part.isdigit():
                    ang = int(part)
                    send_servo_center(ser, ang)
                    current_angle = ang
                    print(f"  -> Đã đặt góc mặc định (thẳng) = {ang} độ\n")
                else:
                    print("  -> Nhập: c <số>, ví dụ: c 88\n")
                continue

            if line == '+' or line == '+1':
                current_angle = send_servo_angle(ser, current_angle + 1)
                print(f"  -> Servo = {current_angle} độ\n")
                continue
            if line == '-' or line == '-1':
                current_angle = send_servo_angle(ser, current_angle - 1)
                print(f"  -> Servo = {current_angle} độ\n")
                continue

            if line.isdigit():
                ang = int(line)
                current_angle = send_servo_angle(ser, ang)
                print(f"  -> Servo = {current_angle} độ\n")
                continue

            print("  -> Lệnh không hiểu. Thử: 88, c 88, +, -, q\n")

    except KeyboardInterrupt:
        print("\nĐã dừng.")
    finally:
        ser.close()
        print("Đã đóng port.")


if __name__ == '__main__':
    main()
