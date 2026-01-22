#!/usr/bin/env python3
"""
Script test servo điều khiển bánh lái

Test servo quay trái/phải theo các góc khác nhau.
Gửi lệnh trực tiếp qua Serial tới Arduino.

Cách chạy:
    python3 test_servo.py [serial_port]

Ví dụ:
    python3 test_servo.py /dev/ttyACM0
"""

import serial
import time
import sys


class ServoTester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None

    def connect(self):
        """Kết nối với Arduino qua Serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            # Đợi Arduino khởi động
            time.sleep(2)

            # Đọc và xóa buffer
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.read_all().decode('utf-8', errors='ignore')
                print(f"Arduino response: {response}")

            print(f"✅ Đã kết nối với Arduino tại {self.port}")
            return True
        except serial.SerialException as e:
            print(f"❌ Không thể kết nối: {e}")
            return False

    def disconnect(self):
        """Ngắt kết nối Serial"""
        if self.serial_conn and self.serial_conn.is_open:
            # Gửi lệnh dừng trước khi đóng
            self.send_command(0.0, 0.0)
            time.sleep(0.1)
            self.serial_conn.close()
            print("✅ Đã ngắt kết nối")

    def send_command(self, linear, angular):
        """Gửi lệnh điều khiển tới Arduino

        Format: V:linear:angular
        - linear: tốc độ tuyến tính (m/s)
        - angular: tốc độ góc (rad/s), dương = quay trái, âm = quay phải
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ Chưa kết nối với Arduino")
            return False

        try:
            command = f"V:{linear:.3f}:{angular:.3f}\n"
            self.serial_conn.write(command.encode('utf-8'))
            self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"❌ Lỗi gửi lệnh: {e}")
            return False

    def test_steering(self, angular, duration=2.0, linear=0.1):
        """Test servo với góc quay cụ thể

        Args:
            angular: tốc độ góc (rad/s), dương = trái, âm = phải
            duration: thời gian giữ (giây)
            linear: tốc độ tuyến tính nhỏ để servo hoạt động (Ackermann cần tốc độ)
        """
        direction = "TRÁI" if angular > 0 else "PHẢI" if angular < 0 else "THẲNG"
        print(f"\n🔄 Test quay {direction}: angular={angular:.2f} rad/s, linear={linear:.2f} m/s")

        self.send_command(linear, angular)
        time.sleep(duration)

        # Về vị trí giữa
        print("   → Về vị trí giữa")
        self.send_command(0.0, 0.0)
        time.sleep(0.5)

    def run_auto_test(self):
        """Chạy test tự động các góc quay"""
        print("\n" + "="*50)
        print("🎮 BẮT ĐẦU TEST SERVO TỰ ĐỘNG")
        print("="*50)

        # Test 1: Về vị trí giữa
        print("\n📍 Test 1: Về vị trí giữa (thẳng)")
        self.send_command(0.0, 0.0)
        time.sleep(1)

        # Test 2: Quay trái nhẹ
        print("\n📍 Test 2: Quay trái nhẹ")
        self.test_steering(angular=0.3, linear=0.1, duration=2)

        # Test 3: Quay trái mạnh
        print("\n📍 Test 3: Quay trái mạnh")
        self.test_steering(angular=0.8, linear=0.1, duration=2)

        # Test 4: Quay phải nhẹ
        print("\n📍 Test 4: Quay phải nhẹ")
        self.test_steering(angular=-0.3, linear=0.1, duration=2)

        # Test 5: Quay phải mạnh
        print("\n📍 Test 5: Quay phải mạnh")
        self.test_steering(angular=-0.8, linear=0.1, duration=2)

        # Test 6: Quay trái tối đa
        print("\n📍 Test 6: Quay trái tối đa")
        self.test_steering(angular=1.0, linear=0.1, duration=2)

        # Test 7: Quay phải tối đa
        print("\n📍 Test 7: Quay phải tối đa")
        self.test_steering(angular=-1.0, linear=0.1, duration=2)

        # Về vị trí giữa
        print("\n📍 Kết thúc: Về vị trí giữa")
        self.send_command(0.0, 0.0)

        print("\n" + "="*50)
        print("✅ HOÀN THÀNH TEST SERVO TỰ ĐỘNG")
        print("="*50)

    def run_manual_test(self):
        """Chạy test thủ công qua bàn phím"""
        print("\n" + "="*50)
        print("🎮 CHẾ ĐỘ TEST SERVO THỦ CÔNG")
        print("="*50)
        print("""
Các phím điều khiển:
  a / ←  : Quay trái
  d / →  : Quay phải
  w      : Tăng góc quay
  s      : Giảm góc quay
  c      : Về vị trí giữa (center)
  q      : Thoát

Lưu ý: Ackermann steering cần tốc độ tuyến tính nhỏ để servo hoạt động.
        """)

        angular = 0.0
        linear = 0.1  # Tốc độ nhỏ để servo hoạt động
        step = 0.2

        try:
            import sys
            import tty
            import termios

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)

            try:
                tty.setraw(sys.stdin.fileno())

                while True:
                    key = sys.stdin.read(1)

                    if key == 'q' or key == '\x03':  # q hoặc Ctrl+C
                        break
                    elif key == 'a':  # Quay trái
                        angular = min(angular + step, 1.0)
                    elif key == 'd':  # Quay phải
                        angular = max(angular - step, -1.0)
                    elif key == 'w':  # Tăng góc
                        step = min(step + 0.1, 0.5)
                        print(f"\r Step = {step:.1f}    ", end='')
                        continue
                    elif key == 's':  # Giảm góc
                        step = max(step - 0.1, 0.1)
                        print(f"\r Step = {step:.1f}    ", end='')
                        continue
                    elif key == 'c':  # Center
                        angular = 0.0

                    # Gửi lệnh
                    self.send_command(linear, angular)
                    direction = "TRÁI" if angular > 0 else "PHẢI" if angular < 0 else "THẲNG"
                    print(f"\r Angular: {angular:+.2f} rad/s ({direction})    ", end='')

            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        except ImportError:
            # Windows không hỗ trợ termios
            print("\n⚠️ Chế độ thủ công không hỗ trợ trên Windows.")
            print("Sử dụng chế độ tự động thay thế.")
            self.run_auto_test()
            return

        # Về vị trí giữa khi thoát
        self.send_command(0.0, 0.0)
        print("\n\n✅ Đã thoát chế độ test thủ công")


def main():
    # Lấy port từ argument hoặc dùng mặc định
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

    print("="*50)
    print("🔧 TEST SERVO ĐIỀU KHIỂN BÁNH LÁI")
    print("="*50)
    print(f"Serial port: {port}")

    tester = ServoTester(port=port)

    if not tester.connect():
        sys.exit(1)

    try:
        # Hỏi người dùng chọn chế độ test
        print("\nChọn chế độ test:")
        print("  1. Test tự động (khuyến nghị)")
        print("  2. Test thủ công (dùng bàn phím)")

        choice = input("\nNhập lựa chọn (1/2): ").strip()

        if choice == '2':
            tester.run_manual_test()
        else:
            tester.run_auto_test()

    except KeyboardInterrupt:
        print("\n\n⚠️ Đã dừng bởi người dùng")
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()
