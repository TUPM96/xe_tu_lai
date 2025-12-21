#!/usr/bin/env python3
"""
Script test RPLIDAR A1M8 đầy đủ trên Raspberry Pi (không cần ROS2)
Có 2 chế độ:
1. Sử dụng ROS2 rplidar_ros node (nếu có ROS2)
2. Đọc trực tiếp từ serial (đơn giản, có thể không chính xác)

Khuyến nghị: Sử dụng ROS2 rplidar_ros node để có dữ liệu chính xác
"""

import time
import sys
import argparse
import math

# Thử import ROS2 để đọc từ topic
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("⚠️  ROS2 không có sẵn, sẽ dùng chế độ đọc serial trực tiếp (có thể không chính xác)")

if not ROS2_AVAILABLE:
    import serial
    import struct


class RPLidarA1M8:
    """Class để giao tiếp với RPLIDAR A1M8"""
    
    # RPLIDAR Commands
    CMD_STOP = 0x25
    CMD_SCAN = 0x20
    CMD_EXPRESS_SCAN = 0x82
    CMD_GET_INFO = 0x50
    CMD_GET_HEALTH = 0x52
    CMD_RESET = 0x40
    
    # Response descriptors
    DESCRIPTOR_LENGTH = 7
    RESP_SCAN = 0x81
    RESP_EXPRESS_SCAN = 0x82
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.scanning = False
        
    def connect(self):
        """Kết nối với RPLIDAR"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=1,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            time.sleep(2)  # Đợi RPLIDAR khởi động
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối: {str(e)}")
            return False
    
    def disconnect(self):
        """Ngắt kết nối"""
        if self.serial and self.serial.is_open:
            self.stop()
            time.sleep(0.1)
            self.serial.close()
    
    def send_command(self, cmd):
        """Gửi lệnh tới RPLIDAR"""
        if not self.serial or not self.serial.is_open:
            return False
        
        # Format: Sync byte (0xA5) + Command
        data = bytes([0xA5, cmd])
        self.serial.write(data)
        self.serial.flush()
        return True
    
    def read_descriptor(self):
        """Đọc descriptor của response"""
        if not self.serial or not self.serial.is_open:
            return None
        
        # Đọc 7 bytes descriptor
        descriptor = self.serial.read(self.DESCRIPTOR_LENGTH)
        if len(descriptor) < self.DESCRIPTOR_LENGTH:
            return None
        
        # Parse descriptor
        if descriptor[0] != 0xA5:
            return None
        
        # Skip descriptor for simplicity, just check sync byte
        return descriptor
    
    def start_scan(self):
        """Bắt đầu quét"""
        # Dừng quét hiện tại (nếu có)
        self.stop()
        time.sleep(0.1)
        
        # Xóa buffer
        if self.serial.in_waiting > 0:
            self.serial.read(self.serial.in_waiting)
        
        # Gửi lệnh quét
        self.send_command(self.CMD_SCAN)
        time.sleep(0.1)
        
        # Đọc descriptor
        descriptor = self.read_descriptor()
        if descriptor:
            self.scanning = True
            return True
        
        return False
    
    def stop(self):
        """Dừng quét"""
        if self.scanning:
            self.send_command(self.CMD_STOP)
            time.sleep(0.1)
            self.scanning = False
    
    def read_scan_data(self):
        """
        Đọc dữ liệu quét
        Returns: (quality, angle, distance) hoặc None
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        if self.serial.in_waiting < 5:
            return None
        
        # Đọc 5 bytes cho một điểm quét
        data = self.serial.read(5)
        if len(data) < 5:
            return None
        
        # Parse dữ liệu
        # Byte 0: [7:0] = S (start flag, 0x01)
        # Byte 1: [7:0] = quality (0-63)
        # Byte 2-3: angle (little endian, Q6)
        # Byte 4-5: distance (little endian, Q2)
        
        start_flag = data[0] & 0x01
        if start_flag != 1:
            return None
        
        quality = data[0] >> 2
        
        # Angle (degrees)
        angle_raw = struct.unpack('<H', data[1:3])[0]
        angle_deg = (angle_raw >> 1) / 64.0
        
        # Distance (mm)
        distance_raw = struct.unpack('<H', data[3:5])[0]
        distance_mm = (distance_raw >> 2) / 4.0
        
        return (quality, angle_deg, distance_mm)


def detect_obstacles(scan_data, front_angle_range=60, safe_distance=0.8):
    """
    Phát hiện vật cản phía trước (giống như trong obstacle_avoidance.py)
    
    Args:
        scan_data: List of (quality, angle_deg, distance_mm)
        front_angle_range: Góc phía trước để kiểm tra (degrees)
        safe_distance: Khoảng cách an toàn (m)
    
    Returns:
        (obstacle_detected, obstacle_direction, min_distance)
        - obstacle_detected: True nếu có vật cản
        - obstacle_direction: -1 (trái), 0 (giữa), 1 (phải)
        - min_distance: Khoảng cách gần nhất (m)
    """
    if not scan_data:
        return False, 0, float('inf')
    
    front_angle_rad = math.radians(front_angle_range / 2)
    safe_distance_mm = safe_distance * 1000
    
    front_points = []
    
    for quality, angle_deg, distance_mm in scan_data:
        # Chuyển góc về radians và normalize về -180 đến 180
        angle_rad = math.radians(angle_deg)
        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        
        # Kiểm tra góc phía trước (từ -front_angle/2 đến +front_angle/2)
        if abs(angle_rad) <= front_angle_rad:
            if distance_mm > 0 and distance_mm < safe_distance_mm:
                front_points.append((distance_mm, angle_rad))
    
    if not front_points:
        return False, 0, float('inf')
    
    # Tìm vật cản gần nhất
    min_distance_mm = min([p[0] for p in front_points])
    closest_point = [p for p in front_points if p[0] == min_distance_mm][0]
    
    # Xác định hướng vật cản
    obstacle_angle = closest_point[1]
    
    # Kiểm tra vật cản ở cả hai bên
    left_obstacles = [p for p in front_points if p[1] < 0]
    right_obstacles = [p for p in front_points if p[1] > 0]
    
    if left_obstacles and right_obstacles:
        obstacle_direction = 0  # Cả hai bên
    elif obstacle_angle < 0:
        obstacle_direction = -1  # Bên trái
    else:
        obstacle_direction = 1  # Bên phải
    
    return True, obstacle_direction, min_distance_mm / 1000.0


def test_lidar_ros2(duration=30):
    """Test LiDAR sử dụng ROS2 topic (chính xác nhất)"""
    print("📡 Sử dụng ROS2 rplidar_ros node...")
    print("⚠️  Đảm bảo đã chạy: ros2 launch xe_lidar rplidar.launch.py")
    print("-" * 60)
    
    rclpy.init()
    node = rclpy.create_node('test_lidar_a1m8')
    
    latest_scan = None
    
    def scan_callback(msg):
        nonlocal latest_scan
        latest_scan = msg
    
    subscription = node.create_subscription(
        LaserScan,
        '/scan',
        scan_callback,
        10
    )
    
    print("✅ Đã subscribe topic /scan")
    print("📊 Đang đợi dữ liệu...")
    print("-" * 60)
    
    start_time = time.time()
    last_print_time = start_time
    scan_count = 0
    
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if latest_scan is not None:
                scan_count += 1
                current_time = time.time()
                
                # Phát hiện vật cản
                ranges = latest_scan.ranges
                angle_min = latest_scan.angle_min
                angle_increment = latest_scan.angle_increment
                
                front_angle_rad = math.radians(30)  # ±30 độ
                front_points = []
                
                for i, dist in enumerate(ranges):
                    angle = angle_min + i * angle_increment
                    if abs(angle) <= front_angle_rad:
                        if not (math.isinf(dist) or math.isnan(dist)):
                            if 0.15 < dist < 12.0:
                                front_points.append((dist, angle))
                
                # In thông tin mỗi giây
                if current_time - last_print_time >= 1.0:
                    print(f"\n⏱️  Thời gian: {current_time - start_time:.1f}s")
                    print(f"📊 Số vòng quét: {scan_count}")
                    print(f"📊 Tổng số điểm: {len(ranges)}")
                    
                    if front_points:
                        front_distances = [p[0] for p in front_points]
                        avg_front = sum(front_distances) / len(front_distances)
                        min_front = min(front_distances)
                        
                        print(f"🎯 Phía trước (0°±30°):")
                        print(f"   Số điểm: {len(front_points)}")
                        print(f"   Khoảng cách trung bình: {avg_front:.2f}m")
                        print(f"   Khoảng cách gần nhất: {min_front:.2f}m")
                        
                        if min_front < 0.8:
                            print(f"⚠️  VẬT CẢN PHÁT HIỆN: {min_front:.2f}m")
                    else:
                        print("✅ KHÔNG có vật cản phía trước")
                    
                    last_print_time = current_time
                
                if duration > 0 and (current_time - start_time) >= duration:
                    break
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n👋 Đã dừng")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Đã dừng")


def test_lidar_serial(port='/dev/ttyUSB0', baudrate=115200, duration=30):
    """
    Test RPLIDAR A1M8 đọc trực tiếp từ serial (đơn giản)
    """
    print("=" * 60)
    print("📡 TEST RPLIDAR A1M8 - Serial Mode")
    print("=" * 60)
    print("⚠️  Chế độ này đơn giản và có thể không chính xác")
    print("   Khuyến nghị: Sử dụng ROS2 rplidar_ros node")
    print("=" * 60)
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Thời gian test: {duration}s")
    print("Nhấn Ctrl+C để dừng")
    print("-" * 60)
    
    lidar = RPLidarA1M8(port, baudrate)
    
    if not lidar.connect():
        print("❌ Không thể kết nối với RPLIDAR")
        return False
    
    print("✅ Đã kết nối với RPLIDAR")
    print("-" * 60)
    
    if not lidar.start_scan():
        print("❌ Không thể bắt đầu quét")
        lidar.disconnect()
        return False
    
    print("✅ Đã bắt đầu quét")
    print("-" * 60)
    print("📊 Thống kê:")
    print("   Angle: 0° (phía trước), 90° (phải), 180° (sau), 270° (trái)")
    print("   Distance: mm → m")
    print("   Obstacle: Detected trong vùng phía trước 60°")
    print("-" * 60)
    
    try:
        scan_count = 0
        total_points = 0
        start_time = time.time()
        last_print_time = start_time
        
        # Lưu dữ liệu quét cho một vòng (360 độ)
        current_scan = []
        
        while True:
            current_time = time.time()
            
            # Đọc dữ liệu quét
            scan_point = lidar.read_scan_data()
            
            if scan_point:
                quality, angle_deg, distance_mm = scan_point
                distance_m = distance_mm / 1000.0
                total_points += 1
                
                # Lưu vào scan hiện tại
                current_scan.append((quality, angle_deg, distance_mm))
                
                # Phát hiện vật cản
                obstacle_detected, direction, min_dist = detect_obstacles(
                    current_scan, 
                    front_angle_range=60,
                    safe_distance=0.8
                )
                
                # In thông tin mỗi giây
                if current_time - last_print_time >= 1.0:
                    print(f"\n⏱️  Thời gian: {current_time - start_time:.1f}s")
                    print(f"📊 Số điểm: {total_points}, Số vòng: {scan_count}")
                    
                    # Thống kê vùng phía trước
                    front_points = [p for p in current_scan 
                                  if abs(math.radians(p[1])) <= math.radians(30)]
                    if front_points:
                        front_distances = [p[2]/1000.0 for p in front_points]
                        avg_front = sum(front_distances) / len(front_distances)
                        min_front = min(front_distances)
                        print(f"🎯 Phía trước (0°±30°):")
                        print(f"   Số điểm: {len(front_points)}")
                        print(f"   Khoảng cách trung bình: {avg_front:.2f}m")
                        print(f"   Khoảng cách gần nhất: {min_front:.2f}m")
                    
                    # Cảnh báo vật cản
                    if obstacle_detected:
                        dir_text = ["TRÁI", "GIỮA/CẢ HAI BÊN", "PHẢI"][direction + 1]
                        print(f"⚠️  VẬT CẢN: {dir_text}, Khoảng cách: {min_dist:.2f}m")
                    else:
                        print("✅ KHÔNG có vật cản phía trước")
                    
                    last_print_time = current_time
                    
                    # Reset scan khi đủ 360 độ (hoặc sau mỗi giây)
                    if len(current_scan) > 360:
                        current_scan = []
                        scan_count += 1
                
                # Kiểm tra thời gian
                if duration > 0 and (current_time - start_time) >= duration:
                    break
            else:
                time.sleep(0.01)  # Chờ dữ liệu
            
    except KeyboardInterrupt:
        print("\n👋 Đã dừng bởi người dùng")
    except Exception as e:
        print(f"\n❌ Lỗi: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Đang dừng quét...")
        lidar.stop()
        lidar.disconnect()
        print(f"✅ Đã dừng. Tổng số điểm: {total_points}")
        print("=" * 60)
    
    return True


def main():
    parser = argparse.ArgumentParser(description='Test RPLIDAR A1M8 đầy đủ')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                       help='Serial port của RPLIDAR (chỉ dùng nếu không có ROS2)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Baudrate (mặc định: 115200)')
    parser.add_argument('--duration', type=int, default=30,
                       help='Thời gian test (giây, 0 = vô hạn, mặc định: 30)')
    parser.add_argument('--use-ros2', action='store_true',
                       help='Sử dụng ROS2 topic (yêu cầu đã chạy rplidar_ros node)')
    parser.add_argument('--use-serial', action='store_true',
                       help='Bắt buộc sử dụng serial trực tiếp')
    
    args = parser.parse_args()
    
    # Ưu tiên dùng ROS2 nếu có
    if args.use_ros2 or (not args.use_serial and ROS2_AVAILABLE):
        try:
            test_lidar_ros2(args.duration)
            sys.exit(0)
        except Exception as e:
            print(f"❌ Lỗi ROS2: {str(e)}")
            print("⚠️  Chuyển sang chế độ serial...")
    
    # Dùng serial
    if not ROS2_AVAILABLE or args.use_serial:
        success = test_lidar_serial(args.port, args.baudrate, args.duration)
        if success:
            sys.exit(0)
        else:
            sys.exit(1)
    
    sys.exit(1)


if __name__ == '__main__':
    main()

