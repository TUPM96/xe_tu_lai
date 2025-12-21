#!/usr/bin/env python3
"""
Script test RPLIDAR trên Raspberry Pi (không cần ROS2)
Sử dụng pyserial để giao tiếp trực tiếp với RPLIDAR
"""

import serial
import time
import sys
import argparse
from struct import unpack


def test_lidar_connection(port='/dev/ttyUSB0', baudrate=115200):
    """
    Test kết nối RPLIDAR
    
    Args:
        port: Serial port của RPLIDAR
        baudrate: Baudrate (mặc định 115200)
    """
    print(f"🔍 Đang kết nối với RPLIDAR tại {port}...")
    print(f"   Baudrate: {baudrate}")
    print("   Nhấn Ctrl+C để dừng")
    print("-" * 50)
    
    try:
        # Mở serial port
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Đợi RPLIDAR khởi động
        
        if not ser.is_open:
            print(f"❌ Không thể mở port {port}")
            return False
        
        print(f"✅ Đã kết nối với RPLIDAR!")
        print("-" * 50)
        
        # Xóa buffer
        ser.reset_input_buffer()
        
        # Gửi lệnh dừng quét (nếu đang quét)
        ser.write(b'\xA5\x25')
        time.sleep(0.1)
        
        # Gửi lệnh bắt đầu quét
        print("📡 Gửi lệnh bắt đầu quét...")
        ser.write(b'\xA5\x20')
        time.sleep(0.1)
        
        scan_count = 0
        sample_count = 0
        
        print("📊 Đang nhận dữ liệu quét...")
        print("   (Hiển thị một số điểm đầu tiên)")
        print("-" * 50)
        
        start_time = time.time()
        last_print_time = start_time
        
        while True:
            # Đọc dữ liệu
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                
                # Hiển thị thông tin cơ bản
                current_time = time.time()
                if current_time - last_print_time > 1.0:  # In mỗi giây
                    print(f"📡 Nhận được {len(data)} bytes")
                    print(f"   Tổng số sample: {sample_count}")
                    print(f"   Thời gian: {current_time - start_time:.1f}s")
                    last_print_time = current_time
                    
                sample_count += len(data)
                
                # Kiểm tra ký tự đặc biệt (đơn giản)
                if b'\xAA' in data or b'\xA5' in data:
                    scan_count += 1
                    if scan_count <= 5:  # Chỉ hiển thị 5 lần đầu
                        print(f"   Phát hiện scan header (lần {scan_count})")
            
            time.sleep(0.1)
            
    except serial.SerialException as e:
        print(f"❌ Lỗi Serial: {str(e)}")
        print(f"   Kiểm tra:")
        print(f"   - RPLIDAR đã được kết nối chưa?")
        print(f"   - Port đúng chưa? (ls /dev/ttyUSB*)")
        print(f"   - Quyền truy cập: sudo chmod 666 {port}")
        print(f"   - Baudrate đúng chưa? (mặc định: 115200)")
        return False
    except KeyboardInterrupt:
        print("\n👋 Đã dừng bởi người dùng")
        if ser.is_open:
            # Dừng quét
            ser.write(b'\xA5\x25')
            time.sleep(0.1)
            ser.close()
        print(f"✅ Đã ngắt kết nối. Tổng số bytes nhận: {sample_count}")
        return True
    except Exception as e:
        print(f"❌ Lỗi: {str(e)}")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return False


def test_lidar_info(port='/dev/ttyUSB0', baudrate=115200):
    """
    Test lấy thông tin RPLIDAR (đơn giản)
    """
    print(f"🔍 Đang lấy thông tin RPLIDAR tại {port}...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        
        if not ser.is_open:
            print(f"❌ Không thể mở port {port}")
            return False
        
        print(f"✅ Port {port} đã mở")
        print(f"   - Baudrate: {baudrate}")
        print(f"   - Timeout: {ser.timeout}s")
        print(f"   - Bytesize: {ser.bytesize}")
        print(f"   - Parity: {ser.parity}")
        print(f"   - Stopbits: {ser.stopbits}")
        
        # Kiểm tra có dữ liệu không
        time.sleep(0.5)
        if ser.in_waiting > 0:
            data = ser.read(min(ser.in_waiting, 100))
            print(f"   - Bytes trong buffer: {len(data)}")
            print(f"   - Dữ liệu mẫu (hex): {data[:20].hex()}")
        else:
            print(f"   - Chưa có dữ liệu trong buffer")
        
        ser.close()
        print("✅ Đã đóng kết nối")
        return True
        
    except Exception as e:
        print(f"❌ Lỗi: {str(e)}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Test RPLIDAR trên Raspberry Pi')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                       help='Serial port của RPLIDAR (mặc định: /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Baudrate (mặc định: 115200)')
    parser.add_argument('--info', action='store_true',
                       help='Chỉ hiển thị thông tin kết nối, không quét')
    
    args = parser.parse_args()
    
    print("=" * 50)
    print("📡 TEST RPLIDAR - Raspberry Pi")
    print("=" * 50)
    
    if args.info:
        success = test_lidar_info(args.port, args.baudrate)
    else:
        success = test_lidar_connection(args.port, args.baudrate)
    
    if success:
        print("=" * 50)
        print("✅ RPLIDAR có thể kết nối!")
        if not args.info:
            print("   (Để test đầy đủ, nên dùng ROS2 node rplidar_ros)")
        print("=" * 50)
        sys.exit(0)
    else:
        print("=" * 50)
        print("❌ RPLIDAR không thể kết nối")
        print("=" * 50)
        sys.exit(1)


if __name__ == '__main__':
    main()

