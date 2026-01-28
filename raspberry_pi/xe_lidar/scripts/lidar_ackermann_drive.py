#!/usr/bin/env python3
"""
Node điều khiển xe Ackermann tự lái chỉ dùng LiDAR
Tương tự project GitHub nhưng phù hợp với Ackermann steering

Tính năng:
- Wall following (bám tường)
- Obstacle avoidance (tránh vật cản)
- Điều khiển góc servo và tốc độ động cơ
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import numpy as np
import math


class LidarAckermannDrive(Node):
    def __init__(self):
        super().__init__('lidar_ackermann_drive')
        
        # Parameters
        self.declare_parameter('max_linear_speed', 0.3)  # Tốc độ tối đa (m/s)
        self.declare_parameter('max_angular_speed', 1.0)  # Tốc độ quay tối đa (rad/s)
        self.declare_parameter('safe_distance', 0.5)  # Khoảng cách an toàn (m)
        self.declare_parameter('front_angle_range', 90.0)  # Góc phía trước để kiểm tra (degrees)
        self.declare_parameter('wall_follow_distance', 0.4)  # Khoảng cách bám tường (m)
        self.declare_parameter('servo_center_angle', 100.0)  # Góc servo giữa (degrees)
        self.declare_parameter('servo_min_angle', 55.0)  # Góc servo tối thiểu (degrees)
        self.declare_parameter('servo_max_angle', 145.0)  # Góc servo tối đa (degrees)
        self.declare_parameter('servo_angle_smoothing', 0.7)  # Làm mượt góc servo
        
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.front_angle_range = self.get_parameter('front_angle_range').value
        self.wall_follow_distance = self.get_parameter('wall_follow_distance').value
        self.servo_center_angle = self.get_parameter('servo_center_angle').value
        self.servo_min_angle = self.get_parameter('servo_min_angle').value
        self.servo_max_angle = self.get_parameter('servo_max_angle').value
        self.servo_angle_smoothing = self.get_parameter('servo_angle_smoothing').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Đã subscribe topic /scan cho LiDAR')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_angle_pub = self.create_publisher(Float32, '/servo_angle_cmd', 10)
        
        # State variables
        self.latest_scan = None
        self.current_servo_angle = self.servo_center_angle
        
        # Timer để điều khiển
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('🚗 Lidar Ackermann Drive Node đã khởi động!')
        self.get_logger().info(f'   Safe distance: {self.safe_distance}m')
        self.get_logger().info(f'   Wall follow distance: {self.wall_follow_distance}m')
        self.get_logger().info(f'   Front angle range: {self.front_angle_range}°')
    
    def scan_callback(self, msg):
        """Callback xử lý dữ liệu LiDAR"""
        if self.latest_scan is None:
            self.get_logger().info('✅ Đã nhận được dữ liệu LiDAR lần đầu!')
        self.latest_scan = msg
    
    def get_front_scan_data(self, scan):
        """Lấy dữ liệu scan phía trước"""
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        front_angle_rad = math.radians(self.front_angle_range / 2)
        front_data = []
        
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if abs(angle) <= front_angle_rad:
                if not (np.isinf(ranges[i]) or np.isnan(ranges[i])):
                    if ranges[i] < scan.range_max and ranges[i] > scan.range_min:
                        front_data.append((angle, ranges[i]))
        
        return front_data
    
    def get_side_distances(self, scan):
        """Lấy khoảng cách bên trái và bên phải"""
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        left_distances = []
        right_distances = []
        
        # Góc 90 độ bên trái và phải
        left_angle = math.pi / 2  # 90 độ
        right_angle = -math.pi / 2  # -90 độ
        
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if not (np.isinf(ranges[i]) or np.isnan(ranges[i])):
                if ranges[i] < scan.range_max and ranges[i] > scan.range_min:
                    # Bên trái (góc 90 độ ± 30 độ)
                    if abs(angle - left_angle) < math.radians(30):
                        left_distances.append(ranges[i])
                    # Bên phải (góc -90 độ ± 30 độ)
                    if abs(angle - right_angle) < math.radians(30):
                        right_distances.append(ranges[i])
        
        left_avg = np.mean(left_distances) if left_distances else float('inf')
        right_avg = np.mean(right_distances) if right_distances else float('inf')
        
        return left_avg, right_avg
    
    def control_loop(self):
        """Vòng lặp điều khiển chính"""
        if self.latest_scan is None:
            # Chưa có dữ liệu LiDAR - chạy chậm
            cmd = Twist()
            cmd.linear.x = self.max_linear_speed * 0.3
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return
        
        cmd = Twist()
        scan = self.latest_scan
        
        # Lấy dữ liệu phía trước
        front_data = self.get_front_scan_data(scan)
        
        if not front_data:
            # Không có dữ liệu phía trước - đi thẳng
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = 0.0
            self.current_servo_angle = self.servo_center_angle
            self.servo_angle_pub.publish(Float32(data=self.current_servo_angle))
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Tìm vật cản gần nhất phía trước
        min_distance = min([item[1] for item in front_data])
        closest_obstacle = [item for item in front_data if item[1] == min_distance][0]
        obstacle_angle = closest_obstacle[0]
        obstacle_distance = closest_obstacle[1]
        
        # Lấy khoảng cách hai bên để wall following
        left_distance, right_distance = self.get_side_distances(scan)
        
        # Logic điều khiển
        if obstacle_distance < self.safe_distance:
            # Có vật cản phía trước - tránh vật cản
            if obstacle_angle < 0:
                # Vật cản bên trái -> rẽ phải
                target_servo_angle = self.servo_max_angle - 20.0  # Rẽ phải
                cmd.linear.x = self.max_linear_speed * 0.5
            else:
                # Vật cản bên phải -> rẽ trái
                target_servo_angle = self.servo_min_angle + 20.0  # Rẽ trái
                cmd.linear.x = self.max_linear_speed * 0.5
            
            self.get_logger().info(
                f'⚠️ Vật cản phía trước: {obstacle_distance*100:.0f}cm, '
                f'góc: {math.degrees(obstacle_angle):.1f}°, '
                f'servo: {target_servo_angle:.1f}°'
            )
        else:
            # Không có vật cản gần - Wall following
            # Điều chỉnh để giữ khoảng cách với tường hai bên
            error = left_distance - right_distance  # Dương = lệch phải, Âm = lệch trái
            
            # PID đơn giản cho wall following
            kp = 0.5
            desired_offset = error * kp
            
            # Chuyển đổi offset sang góc servo
            # error dương -> cần rẽ trái (giảm góc servo)
            # error âm -> cần rẽ phải (tăng góc servo)
            servo_range = (self.servo_max_angle - self.servo_min_angle) / 2.0
            normalized_error = np.clip(desired_offset / self.wall_follow_distance, -1.0, 1.0)
            target_servo_angle = self.servo_center_angle - normalized_error * servo_range * 0.3
            
            # Giới hạn góc servo
            target_servo_angle = max(self.servo_min_angle, min(self.servo_max_angle, target_servo_angle))
            
            # Tốc độ dựa trên khoảng cách vật cản
            if min_distance < self.safe_distance * 1.5:
                cmd.linear.x = self.max_linear_speed * 0.6
            else:
                cmd.linear.x = self.max_linear_speed
        
        # Làm mượt góc servo
        alpha = self.servo_angle_smoothing
        self.current_servo_angle = alpha * self.current_servo_angle + (1 - alpha) * target_servo_angle
        
        # Gửi lệnh
        cmd.angular.z = 0.0  # Không dùng angular, chỉ dùng góc servo
        self.servo_angle_pub.publish(Float32(data=self.current_servo_angle))
        self.cmd_vel_pub.publish(cmd)
        
        # Log định kỳ
        if hasattr(self, 'last_log_time'):
            from time import time
            if time() - self.last_log_time >= 2.0:
                self.get_logger().info(
                    f'📊 L: {left_distance*100:.0f}cm, R: {right_distance*100:.0f}cm, '
                    f'Front: {min_distance*100:.0f}cm, Servo: {self.current_servo_angle:.1f}°'
                )
                self.last_log_time = time()
        else:
            from time import time
            self.last_log_time = time()


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = LidarAckermannDrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Lỗi trong node: {str(e)}')
    finally:
        if node:
            try:
                if rclpy.ok():
                    cmd = Twist()
                    node.cmd_vel_pub.publish(cmd)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
