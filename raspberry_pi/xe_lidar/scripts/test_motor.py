#!/usr/bin/env python3
"""
Script test motor DC điều khiển tiến/lùi

Test motor DC chạy tiến, lùi với các tốc độ khác nhau.
Gửi lệnh trực tiếp qua Serial tới Arduino.

Cách chạy:
    python3 test_motor.py [serial_port]

Ví dụ:
    python3 test_motor.py /dev/ttyACM0
"""

import serial
import time
import sys


class MotorTester:
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
        - linear: tốc độ tuyến tính (m/s), dương = tiến, âm = lùi
        - angular: tốc độ góc (rad/s)
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

    def test_motor(self, linear, duration=2.0):
        """Test motor với tốc độ cụ thể

        Args:
            linear: tốc độ tuyến tính (m/s), dương = tiến, âm = lùi
            duration: thời gian chạy (giây)
        """
        direction = "TIẾN" if linear > 0 else "LÙI" if linear < 0 else "DỪNG"
        speed_percent = abs(linear) / 1.0 * 100
        print(f"\n🚗 Test {direction}: linear={linear:.2f} m/s ({speed_percent:.0f}%)")

        self.send_command(linear, 0.0)
        time.sleep(duration)

        # Dừng motor
        print("   → Dừng motor")
        self.send_command(0.0, 0.0)
        time.sleep(0.5)

    def run_auto_test(self):
        """Chạy test tự động các tốc độ"""
        print("\n" + "="*50)
        print("🎮 BẮT ĐẦU TEST MOTOR DC TỰ ĐỘNG")
        print("="*50)
        print("\n⚠️ CẢNH BÁO: Motor sẽ quay! Đảm bảo xe được đặt an toàn.")

        input("\nNhấn Enter để bắt đầu test...")

        # Test 1: Dừng
        print("\n📍 Test 1: Dừng motor")
        self.send_command(0.0, 0.0)
        time.sleep(1)

        # Test 2: Tiến chậm (30%)
        print("\n📍 Test 2: Tiến chậm (30%)")
        self.test_motor(linear=0.3, duration=2)

        # Test 3: Tiến vừa (50%)
        print("\n📍 Test 3: Tiến vừa (50%)")
        self.test_motor(linear=0.5, duration=2)

        # Test 4: Tiến nhanh (70%)
        print("\n📍 Test 4: Tiến nhanh (70%)")
        self.test_motor(linear=0.7, duration=2)

        # Test 5: Tiến tối đa (100%)
        print("\n📍 Test 5: Tiến tối đa (100%)")
        self.test_motor(linear=1.0, duration=2)

        # Nghỉ giữa tiến và lùi
        print("\n⏸️ Nghỉ 1 giây trước khi test lùi...")
        time.sleep(1)

        # Test 6: Lùi chậm (30%)
        print("\n📍 Test 6: Lùi chậm (30%)")
        self.test_motor(linear=-0.3, duration=2)

        # Test 7: Lùi vừa (50%)
        print("\n📍 Test 7: Lùi vừa (50%)")
        self.test_motor(linear=-0.5, duration=2)

        # Test 8: Lùi nhanh (70%)
        print("\n📍 Test 8: Lùi nhanh (70%)")
        self.test_motor(linear=-0.7, duration=2)

        # Test 9: Lùi tối đa (100%)
        print("\n📍 Test 9: Lùi tối đa (100%)")
        self.test_motor(linear=-1.0, duration=2)

        # Dừng
        print("\n📍 Kết thúc: Dừng motor")
        self.send_command(0.0, 0.0)

        print("\n" + "="*50)
        print("✅ HOÀN THÀNH TEST MOTOR DC TỰ ĐỘNG")
        print("="*50)

    def run_manual_test(self):
        """Chạy test thủ công qua bàn phím"""
        print("\n" + "="*50)
        print("🎮 CHẾ ĐỘ TEST MOTOR DC THỦ CÔNG")
        print("="*50)
        print("""
Các phím điều khiển:
  w / ↑  : Tiến (tăng tốc độ)
  s / ↓  : Lùi (giảm tốc độ / đảo chiều)
  x      : Dừng ngay lập tức (emergency stop)
  +      : Tăng bước tốc độ
  -      : Giảm bước tốc độ
  q      : Thoát

⚠️ CẢNH BÁO: Motor sẽ quay! Đảm bảo xe được đặt an toàn.
        """)

        linear = 0.0
        step = 0.1

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
                    elif key == 'w':  # Tiến
                        linear = min(linear + step, 1.0)
                    elif key == 's':  # Lùi / Giảm tốc
                        linear = max(linear - step, -1.0)
                    elif key == 'x':  # Emergency stop
                        linear = 0.0
                        print("\r🛑 EMERGENCY STOP!             ", end='')
                    elif key == '+' or key == '=':  # Tăng step
                        step = min(step + 0.05, 0.5)
                        print(f"\r Step = {step:.2f}    ", end='')
                        continue
                    elif key == '-':  # Giảm step
                        step = max(step - 0.05, 0.05)
                        print(f"\r Step = {step:.2f}    ", end='')
                        continue

                    # Gửi lệnh
                    self.send_command(linear, 0.0)
                    direction = "TIẾN" if linear > 0 else "LÙI" if linear < 0 else "DỪNG"
                    speed_percent = abs(linear) / 1.0 * 100
                    print(f"\r Linear: {linear:+.2f} m/s ({direction}, {speed_percent:.0f}%)    ", end='')

            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        except ImportError:
            # Windows không hỗ trợ termios
            print("\n⚠️ Chế độ thủ công không hỗ trợ trên Windows.")
            print("Sử dụng chế độ tự động thay thế.")
            self.run_auto_test()
            return

        # Dừng motor khi thoát
        self.send_command(0.0, 0.0)
        print("\n\n✅ Đã thoát chế độ test thủ công")

    def run_speed_ramp_test(self):
        """Test tăng/giảm tốc độ từ từ (ramp test)"""
        print("\n" + "="*50)
        print("🎮 TEST TĂNG/GIẢM TỐC ĐỘ TỪ TỪ (RAMP)")
        print("="*50)
        print("\n⚠️ CẢNH BÁO: Motor sẽ quay! Đảm bảo xe được đặt an toàn.")

        input("\nNhấn Enter để bắt đầu test...")

        # Ramp up tiến
        print("\n📈 Tăng tốc tiến từ 0% đến 100%...")
        for speed in range(0, 101, 10):
            linear = speed / 100.0
            self.send_command(linear, 0.0)
            print(f"   Tốc độ: {speed}%")
            time.sleep(0.5)

        # Giữ tốc độ tối đa
        print("\n⏳ Giữ tốc độ tối đa 2 giây...")
        time.sleep(2)

        # Ramp down
        print("\n📉 Giảm tốc từ 100% xuống 0%...")
        for speed in range(100, -1, -10):
            linear = speed / 100.0
            self.send_command(linear, 0.0)
            print(f"   Tốc độ: {speed}%")
            time.sleep(0.5)

        # Nghỉ
        print("\n⏸️ Nghỉ 1 giây...")
        time.sleep(1)

        # Ramp up lùi
        print("\n📈 Tăng tốc lùi từ 0% đến 100%...")
        for speed in range(0, 101, 10):
            linear = -speed / 100.0
            self.send_command(linear, 0.0)
            print(f"   Tốc độ lùi: {speed}%")
            time.sleep(0.5)

        # Giữ tốc độ tối đa
        print("\n⏳ Giữ tốc độ tối đa 2 giây...")
        time.sleep(2)

        # Ramp down
        print("\n📉 Giảm tốc từ 100% xuống 0%...")
        for speed in range(100, -1, -10):
            linear = -speed / 100.0
            self.send_command(linear, 0.0)
            print(f"   Tốc độ lùi: {speed}%")
            time.sleep(0.5)

        # Dừng
        self.send_command(0.0, 0.0)

        print("\n" + "="*50)
        print("✅ HOÀN THÀNH TEST RAMP")
        print("="*50)


def main():
    # Lấy port từ argument hoặc dùng mặc định
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'

    print("="*50)
    print("🔧 TEST MOTOR DC ĐIỀU KHIỂN TIẾN/LÙI")
    print("="*50)
    print(f"Serial port: {port}")

    tester = MotorTester(port=port)

    if not tester.connect():
        sys.exit(1)

    try:
        # Hỏi người dùng chọn chế độ test
        print("\nChọn chế độ test:")
        print("  1. Test tự động (khuyến nghị)")
        print("  2. Test thủ công (dùng bàn phím)")
        print("  3. Test tăng/giảm tốc từ từ (ramp)")

        choice = input("\nNhập lựa chọn (1/2/3): ").strip()

        if choice == '2':
            tester.run_manual_test()
        elif choice == '3':
            tester.run_speed_ramp_test()
        else:
            tester.run_auto_test()

    except KeyboardInterrupt:
        print("\n\n⚠️ Đã dừng bởi người dùng")
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()
