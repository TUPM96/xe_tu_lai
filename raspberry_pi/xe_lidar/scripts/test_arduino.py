#!/usr/bin/env python3
"""
Script test Arduino trên Raspberry Pi (không cần ROS2)
Gửi lệnh cmd_vel trực tiếp qua Serial để test motor và servo
"""

import serial
import time
import sys
import argparse
import serial.tools.list_ports


def find_arduino_port():
    """Tự động tìm port Arduino"""
    ports = serial.tools.list_ports.comports()
    
    # Tìm các cổng có thể là Arduino
    for port in ports:
        desc = port.description.lower()
        if 'arduino' in desc or 'ch340' in desc or 'ch341' in desc or 'cp210' in desc:
            return port.device
    
    # Thử các cổng phổ biến
    common_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    for port_name in common_ports:
        try:
            test_serial = serial.Serial(port_name, timeout=0.1)
            test_serial.close()
            return port_name
        except:
            continue
    
    return None


def test_arduino_connection(port=None, baudrate=115200):
    """
    Test kết nối với Arduino
    
    Args:
        port: Serial port của Arduino (None để tự động tìm)
        baudrate: Baudrate (mặc định 115200)
    """
    # Tự động tìm port nếu không chỉ định
    if port is None:
        print("🔍 Đang tìm Arduino...")
        port = find_arduino_port()
        if port:
            print(f"✅ Tìm thấy Arduino tại: {port}")
        else:
            print("❌ Không tìm thấy Arduino")
            print("   Kiểm tra:")
            print("   - Arduino đã được kết nối USB chưa?")
            print("   - Arduino đã upload code chưa?")
            print("   - Port có đúng không? (ls /dev/ttyACM*)")
            return False
    else:
        print(f"🔍 Đang kết nối với Arduino tại {port}...")
    
    print(f"   Baudrate: {baudrate}")
    print("-" * 50)
    
    try:
        # Mở serial port
        ser = serial.Serial(port, baudrate, timeout=1, write_timeout=1)
        time.sleep(2)  # Đợi Arduino khởi động
        
        if not ser.is_open:
            print(f"❌ Không thể mở port {port}")
            return False
        
        # Đọc dữ liệu khởi động từ Arduino
        time.sleep(0.5)
        if ser.in_waiting > 0:
            startup_msg = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print("📨 Tin nhắn từ Arduino:")
            print(startup_msg)
            
            if "READY" in startup_msg:
                print("✅ Arduino đã sẵn sàng!")
            else:
                print("⚠️  Không thấy 'READY', nhưng vẫn tiếp tục...")
        else:
            print("⚠️  Không nhận được tin nhắn từ Arduino")
        
        print("-" * 50)
        print("🧪 Test gửi lệnh:")
        print("   Format: V:linear:angular")
        print("   Ví dụ: V:0.3:0.0 -> Tiến 0.3 m/s")
        print("          V:0.0:0.0 -> Dừng")
        print("-" * 50)
        
        return ser
        
    except serial.SerialException as e:
        print(f"❌ Lỗi Serial: {str(e)}")
        print(f"   Kiểm tra:")
        print(f"   - Arduino đã được kết nối USB chưa?")
        print(f"   - Port đúng chưa? (ls /dev/ttyACM*)")
        print(f"   - Quyền truy cập: sudo chmod 666 {port}")
        print(f"   - Baudrate đúng chưa? (mặc định: 115200)")
        return None
    except Exception as e:
        print(f"❌ Lỗi: {str(e)}")
        return None


def send_command(ser, linear, angular):
    """Gửi lệnh tới Arduino"""
    command = f"V:{linear:.3f}:{angular:.3f}\n"
    try:
        ser.write(command.encode('utf-8'))
        ser.flush()
        return True
    except Exception as e:
        print(f"❌ Lỗi gửi lệnh: {str(e)}")
        return False


def interactive_test(ser):
    """Test tương tác"""
    print("\n🎮 Chế độ tương tác:")
    print("   Nhập lệnh: linear,angular")
    print("   Ví dụ: 0.3,0.0  -> Tiến 0.3 m/s")
    print("          0.3,-0.5 -> Tiến và quay trái")
    print("          0,0      -> Dừng")
    print("   Nhập 'q' để thoát")
    print("-" * 50)
    
    try:
        while True:
            user_input = input("Nhập lệnh (linear,angular): ").strip()
            
            if user_input.lower() == 'q':
                break
            
            try:
                parts = user_input.split(',')
                if len(parts) != 2:
                    print("❌ Format không đúng. Ví dụ: 0.3,0.0")
                    continue
                
                linear = float(parts[0])
                angular = float(parts[1])
                
                # Giới hạn giá trị
                linear = max(-1.0, min(1.0, linear))
                angular = max(-1.0, min(1.0, angular))
                
                if send_command(ser, linear, angular):
                    print(f"✅ Đã gửi: linear={linear:.3f}, angular={angular:.3f}")
                    
            except ValueError:
                print("❌ Giá trị không hợp lệ. Nhập số.")
            except Exception as e:
                print(f"❌ Lỗi: {str(e)}")
    
    except KeyboardInterrupt:
        print("\n👋 Đã dừng")


def auto_test(ser):
    """Test tự động với các lệnh mẫu"""
    print("\n🤖 Chế độ test tự động:")
    print("   Sẽ gửi các lệnh mẫu mỗi 2 giây")
    print("   Nhấn Ctrl+C để dừng")
    print("-" * 50)
    
    test_commands = [
        (0.0, 0.0, "Dừng"),
        (0.3, 0.0, "Tiến thẳng"),
        (0.0, 0.0, "Dừng"),
        (0.2, -0.3, "Tiến và quay trái"),
        (0.0, 0.0, "Dừng"),
        (0.2, 0.3, "Tiến và quay phải"),
        (0.0, 0.0, "Dừng"),
        (-0.2, 0.0, "Lùi"),
        (0.0, 0.0, "Dừng"),
    ]
    
    try:
        for linear, angular, description in test_commands:
            print(f"📤 {description}: linear={linear:.2f}, angular={angular:.2f}")
            send_command(ser, linear, angular)
            time.sleep(2)
        
        print("\n✅ Hoàn thành test tự động")
        
    except KeyboardInterrupt:
        print("\n👋 Đã dừng test tự động")
    
    # Đảm bảo dừng robot
    send_command(ser, 0.0, 0.0)
    print("🛑 Đã gửi lệnh dừng")


def main():
    parser = argparse.ArgumentParser(description='Test Arduino trên Raspberry Pi')
    parser.add_argument('--port', type=str, default=None,
                       help='Serial port của Arduino (None để tự động tìm)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Baudrate (mặc định: 115200)')
    parser.add_argument('--auto', action='store_true',
                       help='Chạy test tự động với các lệnh mẫu')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("🔌 TEST ARDUINO - Raspberry Pi")
    print("=" * 50)
    
    ser = test_arduino_connection(args.port, args.baudrate)
    
    if ser is None:
        print("=" * 50)
        print("❌ Không thể kết nối với Arduino")
        print("=" * 50)
        sys.exit(1)
    
    try:
        if args.auto:
            auto_test(ser)
        else:
            interactive_test(ser)
    finally:
        # Dừng robot trước khi đóng
        print("\n🛑 Đang dừng robot...")
        send_command(ser, 0.0, 0.0)
        time.sleep(0.5)
        ser.close()
        print("✅ Đã đóng kết nối")
    
    print("=" * 50)
    print("✅ Test hoàn thành!")
    print("=" * 50)
    sys.exit(0)


if __name__ == '__main__':
    main()

