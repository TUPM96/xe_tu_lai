#!/usr/bin/env python3
"""
Script chạy Autonomous Drive trên Raspberry Pi (KHÔNG CẦN ROS2)
- Camera: Phát hiện vạch kẻ đường (Lane Detection)
- LiDAR: Phát hiện và tránh vật cản (Obstacle Avoidance)  
- Arduino: Điều khiển 1 Motor DC + 1 Servo (Ackermann Steering)

Logic điều khiển Y HỆT như ROS2 obstacle_avoidance.py:
- Ưu tiên 1 (CAO): Tránh vật cản (LiDAR) - Safety
- Ưu tiên 2 (THẤP): Đi theo vạch kẻ đường (Camera) - Navigation

Điều khiển động cơ giống hệt ROS2:
- Motor DC: tốc độ tiến/lùi (từ linear.x)
- Servo: góc quay bánh lái (từ angular.z qua công thức Ackermann)
"""

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
import sys
import argparse
import math
import threading
from collections import deque


class LidarReader:
    """Class đọc LiDAR đơn giản (sử dụng rplidar_ros node hoặc serial trực tiếp)"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.latest_ranges = None
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.range_min = 0.15
        self.range_max = 12.0
        self.thread = None
        
    def start(self):
        """Bắt đầu đọc LiDAR"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"✅ LiDAR đã kết nối tại {self.port}")
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối LiDAR: {str(e)}")
            return False
    
    def _read_loop(self):
        """Vòng lặp đọc dữ liệu LiDAR (đơn giản - chỉ để test)"""
        # Đơn giản: giả sử đã có ROS2 node đang chạy và publish qua UDP/local
        # Hoặc sử dụng SDK rplidar nếu có
        # Ở đây tôi sẽ tạo mock data để test logic
        print("⚠️  LiDAR: Đang dùng mock data. Để có dữ liệu thật, chạy ROS2 rplidar_ros node hoặc sử dụng SDK")
        time.sleep(1)
        # Tạo dữ liệu mock
        while self.running:
            # Mock: không có vật cản
            self.latest_ranges = np.full(360, 5.0)  # 360 điểm, mỗi điểm 5m
            time.sleep(0.1)
    
    def stop(self):
        """Dừng đọc LiDAR"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
    
    def get_scan_data(self):
        """Lấy dữ liệu quét mới nhất"""
        return self.latest_ranges


class CameraReader:
    """Class đọc Camera"""
    
    def __init__(self, device=0, width=640, height=480):
        self.device = device
        self.width = width
        self.height = height
        self.cap = None
        self.latest_image = None
        self.running = False
        self.thread = None
        
    def start(self):
        """Bắt đầu đọc Camera"""
        try:
            self.cap = cv2.VideoCapture(self.device)
            if not self.cap.isOpened():
                return False
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()
            print(f"✅ Camera đã mở tại /dev/video{self.device}")
            return True
        except Exception as e:
            print(f"❌ Lỗi mở Camera: {str(e)}")
            return False
    
    def _read_loop(self):
        """Vòng lặp đọc ảnh"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.latest_image = frame
            time.sleep(0.03)  # ~30 FPS
    
    def stop(self):
        """Dừng đọc Camera"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.cap:
            self.cap.release()
    
    def get_image(self):
        """Lấy ảnh mới nhất"""
        return self.latest_image


class ArduinoController:
    """Class điều khiển Arduino"""
    
    def __init__(self, port=None, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.last_cmd_time = 0
        
    def connect(self):
        """Kết nối với Arduino"""
        # Tự động tìm port
        if self.port is None:
            ports = serial.tools.list_ports.comports()
            for p in ports:
                desc = p.description.lower()
                if 'arduino' in desc or 'ch340' in desc or 'ch341' in desc:
                    self.port = p.device
                    break
            
            if self.port is None:
                # Thử các port phổ biến
                for port_name in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']:
                    try:
                        test = serial.Serial(port_name, timeout=0.1)
                        test.close()
                        self.port = port_name
                        break
                    except:
                        continue
        
        if self.port is None:
            print("❌ Không tìm thấy Arduino")
            return False
        
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1, write_timeout=1)
            time.sleep(2)
            
            # Đọc startup message
            if self.serial.in_waiting > 0:
                msg = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='ignore')
                if "READY" in msg:
                    print(f"✅ Arduino đã sẵn sàng tại {self.port}")
            
            return True
        except Exception as e:
            print(f"❌ Lỗi kết nối Arduino: {str(e)}")
            return False
    
    def send_command(self, linear, angular):
        """Gửi lệnh tới Arduino"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            command = f"V:{linear:.3f}:{angular:.3f}\n"
            self.serial.write(command.encode('utf-8'))
            self.serial.flush()
            self.last_cmd_time = time.time()
            return True
        except Exception as e:
            print(f"❌ Lỗi gửi lệnh: {str(e)}")
            return False
    
    def stop(self):
        """Dừng và đóng kết nối"""
        if self.serial and self.serial.is_open:
            self.send_command(0.0, 0.0)
            time.sleep(0.1)
            self.serial.close()


class AutonomousDrive:
    """Class xử lý logic tự lái (giống như ROS2 node)"""
    
    def __init__(self, min_distance=0.5, safe_distance=0.8,
                 max_linear_speed=0.3, max_angular_speed=1.0,
                 front_angle_range=60, use_camera=True):
        self.min_distance = min_distance
        self.safe_distance = safe_distance
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.front_angle_range = front_angle_range
        self.use_camera = use_camera
        
        # State variables
        self.obstacle_detected = False
        self.obstacle_direction = 0  # -1: trái, 0: giữa, 1: phải
        self.lane_center_offset = 0.0
        self.lane_detected = False
        
    def process_lidar_data(self, ranges, angle_min, angle_increment):
        """Xử lý dữ liệu LiDAR (giống như ROS2)"""
        if ranges is None or len(ranges) == 0:
            self.obstacle_detected = False
            return
        
        front_angle_rad = math.radians(self.front_angle_range / 2)
        num_points = len(ranges)
        front_indices = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            if abs(angle) <= front_angle_rad:
                if not (np.isinf(ranges[i]) or np.isnan(ranges[i])):
                    if ranges[i] < 12.0 and ranges[i] > 0.15:
                        front_indices.append((i, ranges[i], angle))
        
        if not front_indices:
            self.obstacle_detected = False
            return
        
        min_distance = min([item[1] for item in front_indices])
        closest_obstacle = [item for item in front_indices if item[1] == min_distance][0]
        
        if min_distance < self.safe_distance:
            self.obstacle_detected = True
            obstacle_angle = closest_obstacle[2]
            if obstacle_angle < 0:
                self.obstacle_direction = -1
            else:
                self.obstacle_direction = 1
            
            left_obstacles = [item for item in front_indices if item[2] < 0 and item[1] < self.safe_distance]
            right_obstacles = [item for item in front_indices if item[2] > 0 and item[1] < self.safe_distance]
            
            if left_obstacles and right_obstacles:
                self.obstacle_direction = 0
        else:
            self.obstacle_detected = False
    
    def process_camera_lane_detection(self, image):
        """Xử lý camera để phát hiện vạch kẻ đường (giống như ROS2)"""
        if image is None:
            self.lane_detected = False
            self.lane_center_offset = 0.0
            return
        
        try:
            height, width = image.shape[:2]
            roi_top = int(height * 0.4)
            roi = image[roi_top:height, :]
            
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 30, 255])
            white_mask = cv2.inRange(hsv, lower_white, upper_white)
            blurred = cv2.GaussianBlur(white_mask, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, 
                                   minLineLength=20, maxLineGap=15)
            
            if lines is None or len(lines) == 0:
                self.lane_detected = False
                self.lane_center_offset = 0.0
                return
            
            left_lines = []
            right_lines = []
            center_x = width / 2
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 != x1:
                    slope = (y2 - y1) / (x2 - x1)
                    mid_x = (x1 + x2) / 2
                    
                    if slope < -0.2 and mid_x < center_x:
                        left_lines.append(line[0])
                    elif slope > 0.2 and mid_x > center_x:
                        right_lines.append(line[0])
            
            left_x_points = []
            right_x_points = []
            
            for line in left_lines:
                x1, y1, x2, y2 = line
                if y1 > y2:
                    left_x_points.append(x1)
                else:
                    left_x_points.append(x2)
            
            for line in right_lines:
                x1, y1, x2, y2 = line
                if y1 > y2:
                    right_x_points.append(x1)
                else:
                    right_x_points.append(x2)
            
            if left_x_points and right_x_points:
                left_x_avg = np.mean(left_x_points)
                right_x_avg = np.mean(right_x_points)
                lane_center = (left_x_avg + right_x_avg) / 2
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif left_x_points:
                left_x_avg = np.mean(left_x_points)
                lane_center = left_x_avg + 200
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif right_x_points:
                right_x_avg = np.mean(right_x_points)
                lane_center = right_x_avg - 200
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0
                
        except Exception as e:
            print(f"⚠️  Lỗi xử lý camera: {str(e)}")
            self.lane_detected = False
            self.lane_center_offset = 0.0
    
    def compute_control(self):
        """
        Tính toán lệnh điều khiển (giống như ROS2)
        Ưu tiên 1: Tránh vật cản (LiDAR)
        Ưu tiên 2: Đi theo vạch kẻ đường (Camera)
        """
        linear = 0.0
        angular = 0.0
        
        # Ưu tiên 1: Kiểm tra vật cản
        if self.obstacle_detected:
            if self.obstacle_direction == 0:
                # Vật cản ở giữa, lùi và quay
                linear = -self.max_linear_speed * 0.5
                angular = self.max_angular_speed * 0.8
            elif self.obstacle_direction < 0:
                # Vật cản bên trái, quay phải
                linear = self.max_linear_speed * 0.6
                angular = -self.max_angular_speed * 0.7
            else:
                # Vật cản bên phải, quay trái
                linear = self.max_linear_speed * 0.6
                angular = self.max_angular_speed * 0.7
        else:
            # Ưu tiên 2: Đi theo vạch kẻ đường
            if self.use_camera and self.lane_detected:
                linear = self.max_linear_speed
                angular = -self.lane_center_offset * self.max_angular_speed * 0.8
            else:
                # Không có vạch, đi thẳng
                linear = self.max_linear_speed
                angular = 0.0
        
        return linear, angular


def main():
    parser = argparse.ArgumentParser(description='Autonomous Drive Python (không cần ROS2)')
    parser.add_argument('--camera-device', type=int, default=0, help='Camera device ID')
    parser.add_argument('--lidar-port', type=str, default='/dev/ttyUSB0', help='LiDAR serial port')
    parser.add_argument('--arduino-port', type=str, default=None, help='Arduino serial port (None = auto detect)')
    parser.add_argument('--use-ros2-lidar', action='store_true', help='Sử dụng ROS2 topic /scan (yêu cầu rplidar_ros node)')
    parser.add_argument('--min-distance', type=float, default=0.5, help='Khoảng cách tối thiểu (m)')
    parser.add_argument('--safe-distance', type=float, default=0.8, help='Khoảng cách an toàn (m)')
    parser.add_argument('--max-linear-speed', type=float, default=0.3, help='Tốc độ tối đa (m/s)')
    parser.add_argument('--max-angular-speed', type=float, default=1.0, help='Tốc độ quay tối đa (rad/s)')
    parser.add_argument('--front-angle-range', type=int, default=60, help='Góc phía trước (degrees)')
    parser.add_argument('--no-camera', action='store_true', help='Tắt camera')
    parser.add_argument('--show-display', action='store_true', help='Hiển thị camera và thông tin')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🚗 AUTONOMOUS DRIVE - Python (KHÔNG CẦN ROS2)")
    print("=" * 60)
    
    # Khởi tạo các component
    camera = CameraReader(device=args.camera_device) if not args.no_camera else None
    lidar = LidarReader(port=args.lidar_port, use_ros2=args.use_ros2_lidar)
    arduino = ArduinoController(port=args.arduino_port)
    drive = AutonomousDrive(
        min_distance=args.min_distance,
        safe_distance=args.safe_distance,
        max_linear_speed=args.max_linear_speed,
        max_angular_speed=args.max_angular_speed,
        front_angle_range=args.front_angle_range,
        use_camera=not args.no_camera
    )
    
    # Khởi động
    print("\n🔌 Đang kết nối các thiết bị...")
    success = True
    
    if camera:
        if not camera.start():
            print("⚠️  Camera không thể mở, tiếp tục không có camera...")
            camera = None
    
    if not lidar.start():
        print("⚠️  LiDAR không thể kết nối, tiếp tục với mock data...")
    
    if not arduino.connect():
        print("❌ Arduino không thể kết nối!")
        success = False
    
    if not success:
        print("❌ Không thể khởi động hệ thống")
        sys.exit(1)
    
    print("\n✅ Tất cả thiết bị đã sẵn sàng!")
    print("=" * 60)
    print("📊 Trạng thái:")
    print(f"   Camera: {'Bật' if camera else 'Tắt'}")
    if lidar.ros2_node:
        print(f"   LiDAR: ROS2 topic /scan")
    elif lidar.running:
        print(f"   LiDAR: Mock data (test mode)")
    else:
        print(f"   LiDAR: Lỗi")
    print(f"   Arduino: {'Kết nối' if arduino.serial and arduino.serial.is_open else 'Lỗi'}")
    print("=" * 60)
    print("🎮 Điều khiển:")
    print("   Nhấn 'q' để thoát")
    print("   Nhấn 's' để dừng")
    print("=" * 60)
    print("\n🚀 Bắt đầu tự lái...\n")
    
    try:
        loop_count = 0
        last_display_time = time.time()
        
        while True:
            loop_start = time.time()
            
            # Đọc dữ liệu
            if camera:
                image = camera.get_image()
                if image is not None:
                    drive.process_camera_lane_detection(image)
            
            if lidar.latest_ranges is not None:
                # Xử lý dữ liệu LiDAR
                drive.process_lidar_data(lidar.latest_ranges, lidar.angle_min, 
                                        (lidar.angle_max - lidar.angle_min) / len(lidar.latest_ranges))
            
            # Tính toán lệnh điều khiển
            linear, angular = drive.compute_control()
            
            # Gửi lệnh tới Arduino
            arduino.send_command(linear, angular)
            
            # Hiển thị thông tin mỗi 0.5 giây
            current_time = time.time()
            if current_time - last_display_time >= 0.5:
                elapsed = current_time - (loop_start if 'loop_start' in locals() else current_time)
                print(f"⏱️  Loop: {loop_count}")
                print(f"   📡 LiDAR: {'⚠️ VẬT CẢN' if drive.obstacle_detected else '✅ OK'}")
                if drive.obstacle_detected:
                    dir_text = ["TRÁI", "GIỮA", "PHẢI"][drive.obstacle_direction + 1]
                    print(f"      Hướng: {dir_text}")
                if camera:
                    print(f"   📷 Camera: {'✅ Vạch' if drive.lane_detected else '❌ Không'}, Offset: {drive.lane_center_offset:.2f}")
                print(f"   🎮 Control: linear={linear:.2f} m/s, angular={angular:.2f} rad/s")
                print()
                last_display_time = current_time
            
            # Hiển thị camera nếu được bật
            if args.show_display and camera:
                image = camera.get_image()
                if image is not None:
                display_image = image.copy()
                cv2.putText(display_image, f"Linear: {linear:.2f} | Angular: {angular:.2f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, f"Obstacle: {'YES' if drive.obstacle_detected else 'NO'}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_image, f"Lane: {'YES' if drive.lane_detected else 'NO'}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.imshow('Autonomous Drive', display_image)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    elif key == ord('s'):
                        arduino.send_command(0.0, 0.0)
                        print("🛑 Đã dừng")
            
            loop_count += 1
            time.sleep(0.1)  # 10 Hz
            
    except KeyboardInterrupt:
        print("\n👋 Đã dừng bởi người dùng")
    except Exception as e:
        print(f"\n❌ Lỗi: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Đang dừng hệ thống...")
        arduino.stop()
        if camera:
            camera.stop()
        if lidar:
            lidar.stop()
        cv2.destroyAllWindows()
        print("✅ Đã dừng")
        print("=" * 60)


if __name__ == '__main__':
    main()

