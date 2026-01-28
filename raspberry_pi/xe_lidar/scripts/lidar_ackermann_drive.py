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
from enum import Enum


class ObstacleAvoidanceState(Enum):
    """Trạng thái tránh vật cản"""
    NORMAL = 0          # Đi bình thường
    AVOIDING_LEFT = 1  # Đang tránh vật cản bằng cách rẽ trái (45°)
    AVOIDING_RIGHT = 2 # Đang tránh vật cản bằng cách rẽ phải (155°)
    RETURNING = 3      # Đang quay về đi thẳng sau khi tránh vật cản
    BACKING = 4        # Đang lùi lại để tránh vật cản


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
        self.smoothed_servo_angle_deg = self.servo_center_angle
        
        # Obstacle avoidance state machine
        self.obstacle_avoidance_state = ObstacleAvoidanceState.NORMAL
        self.obstacle_detected = False
        self.obstacle_direction = 0  # -1: trái, 1: phải, 0: giữa
        self.closest_obstacle_distance = float('inf')
        self.closest_obstacle_angle = 0.0
        self.left_obstacle_distance = float('inf')
        self.right_obstacle_distance = float('inf')
        self.obstacle_clear_count = 0
        self.obstacle_clear_threshold = 30  # Số lần kiểm tra không có vật cản để quay về
        self.avoidance_distance = 0.0
        self.avoidance_distance_threshold = 1.5  # Quãng đường tối thiểu để quay về (m)
        self.backing_distance = 0.0
        self.backing_distance_threshold = 0.3  # Quãng đường lùi lại tối thiểu (30cm)
        self.backing_start_time = None
        self.next_avoidance_state = ObstacleAvoidanceState.NORMAL  # State sẽ chuyển sau khi lùi lại
        self.next_avoidance_angle = self.servo_center_angle  # Góc sẽ rẽ sau khi lùi lại
        
        # Timing
        self.last_control_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                                 self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        
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
        # Xử lý dữ liệu LiDAR ngay khi nhận được
        self.process_lidar_data(msg)
    
    def process_lidar_data(self, scan):
        """Xử lý dữ liệu LiDAR để phát hiện vật cản - giống logic trong obstacle_avoidance.py"""
        if scan is None:
            return
        
        ranges = np.array(scan.ranges)
        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        
        # Chuyển đổi góc phía trước sang radians
        front_angle_rad = math.radians(self.front_angle_range / 2)
        
        # Tìm các điểm trong vùng phía trước
        num_points = len(ranges)
        front_indices = []
        left_indices = []
        right_indices = []
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            # Kiểm tra góc phía trước (từ -front_angle/2 đến +front_angle/2)
            if abs(angle) <= front_angle_rad:
                if not (np.isinf(ranges[i]) or np.isnan(ranges[i])):
                    if ranges[i] < scan.range_max and ranges[i] > scan.range_min:
                        front_indices.append((i, ranges[i], angle))
                        if angle < 0:
                            left_indices.append((i, ranges[i], angle))
                        else:
                            right_indices.append((i, ranges[i], angle))
        
        if not front_indices:
            self.obstacle_detected = False
            return
        
        # Tìm vật cản gần nhất phía trước và ở các góc độ khác nhau
        min_distance = min([item[1] for item in front_indices]) if front_indices else float('inf')
        if front_indices:
            closest_obstacle = [item for item in front_indices if item[1] == min_distance][0]
            self.closest_obstacle_distance = min_distance
            self.closest_obstacle_angle = closest_obstacle[2]
        else:
            self.closest_obstacle_distance = float('inf')
            self.closest_obstacle_angle = 0.0
        
        # Tìm vật cản gần nhất ở mỗi bên
        if left_indices:
            self.left_obstacle_distance = min([item[1] for item in left_indices])
        else:
            self.left_obstacle_distance = float('inf')
        
        if right_indices:
            self.right_obstacle_distance = min([item[1] for item in right_indices])
        else:
            self.right_obstacle_distance = float('inf')
        
        # Kiểm tra có vật cản không - tăng ngưỡng để phát hiện sớm hơn
        detection_distance = self.safe_distance * 1.5  # Phát hiện ở 60cm thay vì 40cm
        if min_distance < detection_distance:
            self.obstacle_detected = True
            # Xác định hướng vật cản dựa trên góc và khoảng cách
            obstacle_angle = self.closest_obstacle_angle
            
            # Kiểm tra vật cản ở cả hai bên
            left_obstacles = [item for item in front_indices if item[2] < 0 and item[1] < detection_distance]
            right_obstacles = [item for item in front_indices if item[2] > 0 and item[1] < detection_distance]
            
            if left_obstacles and right_obstacles:
                # Vật cản ở cả hai bên - chọn bên có khoảng cách xa hơn để rẽ
                left_min_dist = min([item[1] for item in left_obstacles])
                right_min_dist = min([item[1] for item in right_obstacles])
                if left_min_dist > right_min_dist:
                    self.obstacle_direction = -1  # Rẽ trái (vật cản bên phải xa hơn)
                else:
                    self.obstacle_direction = 1   # Rẽ phải (vật cản bên trái xa hơn)
            elif obstacle_angle < -0.1:  # Vật cản rõ ràng ở bên trái
                self.obstacle_direction = -1  # Vật cản bên trái -> rẽ phải
            elif obstacle_angle > 0.1:  # Vật cản rõ ràng ở bên phải
                self.obstacle_direction = 1   # Vật cản bên phải -> rẽ trái
            else:
                # Vật cản ở giữa - chọn bên có khoảng cách xa hơn
                if self.left_obstacle_distance > self.right_obstacle_distance:
                    self.obstacle_direction = -1  # Rẽ trái
                else:
                    self.obstacle_direction = 1   # Rẽ phải
        else:
            # Kiểm tra kỹ hơn: không có vật cản ở phía trước VÀ cả hai bên đều an toàn
            left_safe = all([item[1] >= self.safe_distance * 1.2 for item in left_indices]) if left_indices else True
            right_safe = all([item[1] >= self.safe_distance * 1.2 for item in right_indices]) if right_indices else True
            
            # Chỉ coi là không có vật cản khi cả phía trước và hai bên đều an toàn
            # Kiểm tra ở khoảng cách lớn hơn để đảm bảo an toàn
            clear_distance = self.safe_distance * 2.0  # Phải cách xa 80cm mới coi là an toàn
            if min_distance >= clear_distance and left_safe and right_safe:
                self.obstacle_detected = False
            else:
                # Vẫn còn vật cản gần
                self.obstacle_detected = True
    
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
        """Vòng lặp điều khiển chính - giống logic trong obstacle_avoidance.py"""
        cmd = Twist()
        
        # Tính delta thời gian cho tích lũy quãng đường
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        dt = max(0.01, current_time - self.last_control_time)
        self.last_control_time = current_time
        
        # Kiểm tra nếu chưa có dữ liệu LiDAR
        if self.latest_scan is None:
            # Chưa có dữ liệu LiDAR - chạy chậm và đi thẳng
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = 0.0
            self.current_servo_angle = self.servo_center_angle
            self.smoothed_servo_angle_deg = self.servo_center_angle
            self.servo_angle_pub.publish(Float32(data=self.current_servo_angle))
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Xử lý dữ liệu LiDAR (đã được gọi trong scan_callback, nhưng đảm bảo cập nhật)
        self.process_lidar_data(self.latest_scan)
        
        # ƯU TIÊN: Kiểm tra vật cản bằng LiDAR
        if self.obstacle_detected:
            # Có vật cản - kích hoạt chế độ tránh vật cản
            self.obstacle_clear_count = 0
            
            if self.obstacle_avoidance_state == ObstacleAvoidanceState.NORMAL:
                # Bắt đầu tránh vật cản - reset quãng đường
                self.avoidance_distance = 0.0
                
                # Kiểm tra nếu vật cản quá gần (< 20cm) -> lùi lại trước
                if self.closest_obstacle_distance < 0.2:
                    # Vật cản quá gần -> lùi lại trước
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    # Lưu hướng sẽ rẽ sau khi lùi lại
                    if self.obstacle_direction < 0:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        self.next_avoidance_angle = 155.0
                    elif self.obstacle_direction > 0:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT
                        self.next_avoidance_angle = 45.0
                    else:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        self.next_avoidance_angle = 155.0
                    
                    cmd.linear.x = -self.max_linear_speed * 0.5  # Lùi lại với tốc độ 50%
                    self.smoothed_servo_angle_deg = self.servo_center_angle  # Giữ thẳng khi lùi
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().info(f'⚠️ Vật cản quá gần ({self.closest_obstacle_distance*100:.0f}cm) - Lùi lại để tránh')
                elif self.obstacle_direction < 0:
                    # Vật cản bên trái -> rẽ phải (155°)
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                    avoid_servo_angle = 155.0
                    self.get_logger().info('⚠️ Vật cản bên trái - Rẽ phải 155° để tránh')
                    self.smoothed_servo_angle_deg = avoid_servo_angle
                    self.current_servo_angle = avoid_servo_angle
                    self.servo_angle_pub.publish(Float32(data=avoid_servo_angle))
                    cmd.linear.x = self.max_linear_speed * 0.6
                elif self.obstacle_direction > 0:
                    # Vật cản bên phải -> rẽ trái (45°)
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT
                    avoid_servo_angle = 45.0
                    self.get_logger().info('⚠️ Vật cản bên phải - Rẽ trái 45° để tránh')
                    self.smoothed_servo_angle_deg = avoid_servo_angle
                    self.current_servo_angle = avoid_servo_angle
                    self.servo_angle_pub.publish(Float32(data=avoid_servo_angle))
                    cmd.linear.x = self.max_linear_speed * 0.6
                else:
                    # Vật cản ở giữa -> lui lại và rẽ phải
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                    self.next_avoidance_angle = 155.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().info('⚠️ Vật cản phía trước - Lùi lại và sẽ rẽ phải 155°')
                
                cmd.angular.z = 0.0
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.BACKING:
                # Đang lùi lại để tránh vật cản
                if self.closest_obstacle_distance < 0.15:
                    # Vẫn quá gần - tiếp tục lùi lại
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.backing_distance += abs(cmd.linear.x) * dt
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    if int(current_time * 10) % 10 == 0:  # Log mỗi giây
                        self.get_logger().info(f'⬅️ Đang lùi lại ({self.backing_distance*100:.0f}cm) - Vật cản: {self.closest_obstacle_distance*100:.0f}cm')
                elif self.backing_distance < self.backing_distance_threshold:
                    # Chưa lùi đủ xa - tiếp tục lùi
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.backing_distance += abs(cmd.linear.x) * dt
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                else:
                    # Đã lùi đủ xa -> chuyển sang tránh vật cản
                    self.obstacle_avoidance_state = self.next_avoidance_state
                    self.smoothed_servo_angle_deg = self.next_avoidance_angle
                    self.current_servo_angle = self.next_avoidance_angle
                    self.servo_angle_pub.publish(Float32(data=self.next_avoidance_angle))
                    cmd.linear.x = self.max_linear_speed * 0.6
                    self.backing_distance = 0.0
                    self.get_logger().info(f'✅ Đã lùi đủ xa - Chuyển sang rẽ {self.next_avoidance_angle:.0f}° để tránh')
                cmd.angular.z = 0.0
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.AVOIDING_LEFT:
                # Đang tránh bằng cách rẽ trái - kiểm tra và điều chỉnh liên tục
                if self.left_obstacle_distance < 0.25:
                    # Có vật cản ở phía đang rẽ - lùi lại
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT  # Đổi hướng
                    self.next_avoidance_angle = 155.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().warn(f'⚠️ Vật cản bên trái khi đang rẽ trái ({self.left_obstacle_distance*100:.0f}cm) - Lùi lại và đổi hướng!')
                elif self.closest_obstacle_distance < 0.2:
                    # Vật cản quá gần phía trước - lùi lại
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT  # Giữ hướng
                    self.next_avoidance_angle = 45.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().warn(f'⚠️ Vật cản quá gần phía trước ({self.closest_obstacle_distance*100:.0f}cm) - Lùi lại!')
                else:
                    # Trong mức an toàn - tiếp tục chạy và quẹo
                    # Điều chỉnh góc rẽ dựa trên khoảng cách vật cản
                    if self.closest_obstacle_distance < 0.3:
                        target_servo_angle = 35.0  # Rẽ rất mạnh
                        cmd.linear.x = self.max_linear_speed * 0.4  # Giảm tốc độ
                    elif self.closest_obstacle_distance < 0.4:
                        target_servo_angle = 40.0  # Rẽ mạnh
                        cmd.linear.x = self.max_linear_speed * 0.5
                    elif self.closest_obstacle_distance < 0.5:
                        target_servo_angle = 42.0  # Rẽ vừa
                        cmd.linear.x = self.max_linear_speed * 0.6
                    else:
                        target_servo_angle = 45.0  # Rẽ bình thường
                        cmd.linear.x = self.max_linear_speed * 0.6
                    
                    # Làm mượt góc rẽ để tránh quẹo quá nhanh
                    alpha = 0.7  # Hệ số làm mượt cho góc rẽ tránh vật cản
                    self.smoothed_servo_angle_deg = alpha * self.smoothed_servo_angle_deg + (1 - alpha) * target_servo_angle
                    self.current_servo_angle = self.smoothed_servo_angle_deg
                    self.servo_angle_pub.publish(Float32(data=self.smoothed_servo_angle_deg))
                    # Tích lũy quãng đường đã đi
                    self.avoidance_distance += cmd.linear.x * dt
                cmd.angular.z = 0.0
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.AVOIDING_RIGHT:
                # Đang tránh bằng cách rẽ phải - kiểm tra và điều chỉnh liên tục
                if self.closest_obstacle_distance < 0.2:
                    # Vật cản quá gần - lùi lại
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT  # Giữ hướng
                    self.next_avoidance_angle = 155.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().warn(f'⚠️ Vật cản quá gần ({self.closest_obstacle_distance*100:.0f}cm) - Lùi lại!')
                elif self.right_obstacle_distance < 0.25:
                    # Vật cản quá gần ở phía đang rẽ - lùi lại và đổi hướng
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT  # Đổi hướng
                    self.next_avoidance_angle = 45.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().warn(f'⚠️ Vật cản bên phải quá gần ({self.right_obstacle_distance*100:.0f}cm) - Lùi lại và đổi hướng!')
                else:
                    # Trong mức an toàn - tiếp tục chạy và quẹo
                    # Điều chỉnh góc rẽ dựa trên khoảng cách vật cản
                    if self.closest_obstacle_distance < 0.3:
                        target_servo_angle = 165.0  # Rẽ rất mạnh
                        cmd.linear.x = self.max_linear_speed * 0.4  # Giảm tốc độ
                    elif self.closest_obstacle_distance < 0.4:
                        target_servo_angle = 160.0  # Rẽ mạnh
                        cmd.linear.x = self.max_linear_speed * 0.5
                    elif self.closest_obstacle_distance < 0.5:
                        target_servo_angle = 158.0  # Rẽ vừa
                        cmd.linear.x = self.max_linear_speed * 0.6
                    else:
                        target_servo_angle = 155.0  # Rẽ bình thường
                        cmd.linear.x = self.max_linear_speed * 0.6
                    
                    # Làm mượt góc rẽ để tránh quẹo quá nhanh
                    alpha = 0.7  # Hệ số làm mượt cho góc rẽ tránh vật cản
                    self.smoothed_servo_angle_deg = alpha * self.smoothed_servo_angle_deg + (1 - alpha) * target_servo_angle
                    self.current_servo_angle = self.smoothed_servo_angle_deg
                    self.servo_angle_pub.publish(Float32(data=self.smoothed_servo_angle_deg))
                    # Tích lũy quãng đường đã đi
                    self.avoidance_distance += cmd.linear.x * dt
                cmd.angular.z = 0.0
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.RETURNING:
                # Đang quay về đi thẳng nhưng lại gặp vật cản -> quay lại tránh
                # Kiểm tra nếu vật cản quá gần -> lùi lại trước
                if self.closest_obstacle_distance < 0.2:
                    # Vật cản quá gần -> lùi lại
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                    self.backing_distance = 0.0
                    self.backing_start_time = current_time
                    if self.obstacle_direction < 0:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        self.next_avoidance_angle = 155.0
                    elif self.obstacle_direction > 0:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT
                        self.next_avoidance_angle = 45.0
                    else:
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        self.next_avoidance_angle = 155.0
                    cmd.linear.x = -self.max_linear_speed * 0.5
                    self.smoothed_servo_angle_deg = self.servo_center_angle
                    self.current_servo_angle = self.servo_center_angle
                    self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                    self.get_logger().info('⚠️ Gặp vật cản khi quay về - Lùi lại để tránh')
                else:
                    # Reset và bắt đầu tránh lại
                    self.avoidance_distance = 0.0
                    if self.obstacle_direction < 0:
                        self.obstacle_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        avoid_servo_angle = 155.0
                        self.get_logger().info('⚠️ Gặp vật cản khi quay về - Rẽ phải 155° để tránh')
                    elif self.obstacle_direction > 0:
                        self.obstacle_avoidance_state = ObstacleAvoidanceState.AVOIDING_LEFT
                        avoid_servo_angle = 45.0
                        self.get_logger().info('⚠️ Gặp vật cản khi quay về - Rẽ trái 45° để tránh')
                    else:
                        self.obstacle_avoidance_state = ObstacleAvoidanceState.BACKING
                        self.backing_distance = 0.0
                        self.backing_start_time = current_time
                        self.next_avoidance_state = ObstacleAvoidanceState.AVOIDING_RIGHT
                        self.next_avoidance_angle = 155.0
                        cmd.linear.x = -self.max_linear_speed * 0.5
                        self.smoothed_servo_angle_deg = self.servo_center_angle
                        self.current_servo_angle = self.servo_center_angle
                        self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
                        self.get_logger().info('⚠️ Gặp vật cản khi quay về - Lùi lại và rẽ phải 155°')
                        cmd.angular.z = 0.0
                        self.cmd_vel_pub.publish(cmd)
                        return
                    
                    self.smoothed_servo_angle_deg = avoid_servo_angle
                    self.current_servo_angle = avoid_servo_angle
                    self.servo_angle_pub.publish(Float32(data=avoid_servo_angle))
                    cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = 0.0
        else:
            # Không có vật cản
            if self.obstacle_avoidance_state == ObstacleAvoidanceState.AVOIDING_LEFT:
                # Đã qua vật cản bên phải -> tiếp tục đi và kiểm tra điều kiện quay về
                self.obstacle_clear_count += 1
                # Tích lũy quãng đường đã đi
                self.avoidance_distance += cmd.linear.x * dt if cmd.linear.x > 0 else 0
                
                # Điều kiện quay về: đã đi đủ xa VÀ không có vật cản trong một khoảng thời gian
                if self.avoidance_distance >= self.avoidance_distance_threshold and \
                   self.obstacle_clear_count >= self.obstacle_clear_threshold:
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.RETURNING
                    self.get_logger().info(f'✅ Đã qua vật cản ({self.avoidance_distance:.2f}m) - Quay về đi thẳng (rẽ phải lại)')
                    self.obstacle_clear_count = 0
                    self.avoidance_distance = 0.0
                # Tiếp tục giữ góc 45° cho đến khi chuyển sang RETURNING
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = 0.0
                self.smoothed_servo_angle_deg = 45.0
                self.current_servo_angle = 45.0
                self.servo_angle_pub.publish(Float32(data=45.0))
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.AVOIDING_RIGHT:
                # Đã qua vật cản bên trái -> tiếp tục đi và kiểm tra điều kiện quay về
                self.obstacle_clear_count += 1
                # Tích lũy quãng đường đã đi
                self.avoidance_distance += cmd.linear.x * dt if cmd.linear.x > 0 else 0
                
                # Điều kiện quay về: đã đi đủ xa VÀ không có vật cản trong một khoảng thời gian
                if self.avoidance_distance >= self.avoidance_distance_threshold and \
                   self.obstacle_clear_count >= self.obstacle_clear_threshold:
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.RETURNING
                    self.get_logger().info(f'✅ Đã qua vật cản ({self.avoidance_distance:.2f}m) - Quay về đi thẳng (rẽ trái lại)')
                    self.obstacle_clear_count = 0
                    self.avoidance_distance = 0.0
                # Tiếp tục giữ góc 155° cho đến khi chuyển sang RETURNING
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = 0.0
                self.smoothed_servo_angle_deg = 155.0
                self.current_servo_angle = 155.0
                self.servo_angle_pub.publish(Float32(data=155.0))
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.BACKING:
                # Đang lùi lại nhưng không có vật cản nữa -> chuyển sang tránh vật cản
                # (Trường hợp này ít xảy ra nhưng cần xử lý)
                self.obstacle_avoidance_state = self.next_avoidance_state
                self.smoothed_servo_angle_deg = self.next_avoidance_angle
                self.current_servo_angle = self.next_avoidance_angle
                self.servo_angle_pub.publish(Float32(data=self.next_avoidance_angle))
                cmd.linear.x = self.max_linear_speed * 0.6
                self.backing_distance = 0.0
                self.get_logger().info(f'✅ Không còn vật cản khi lùi - Chuyển sang rẽ {self.next_avoidance_angle:.0f}°')
                cmd.angular.z = 0.0
            elif self.obstacle_avoidance_state == ObstacleAvoidanceState.RETURNING:
                # Đang quay về đi thẳng - tiếp tục đi thẳng
                self.obstacle_clear_count += 1
                # Tích lũy quãng đường đã đi khi quay về
                self.avoidance_distance += cmd.linear.x * dt if cmd.linear.x > 0 else 0
                
                # Đi một đoạn đủ dài trước khi về chế độ bình thường
                if self.obstacle_clear_count >= self.obstacle_clear_threshold and \
                   self.avoidance_distance >= self.avoidance_distance_threshold:
                    self.obstacle_avoidance_state = ObstacleAvoidanceState.NORMAL
                    self.get_logger().info('✅ Đã quay về đi thẳng - Chế độ bình thường')
                    self.obstacle_clear_count = 0
                    self.avoidance_distance = 0.0
                cmd.linear.x = self.max_linear_speed * 0.8  # Tốc độ khi đi thẳng
                cmd.angular.z = 0.0
                self.smoothed_servo_angle_deg = self.servo_center_angle
                self.current_servo_angle = self.servo_center_angle
                self.servo_angle_pub.publish(Float32(data=self.servo_center_angle))
            else:
                # NORMAL state - đi thẳng hoặc wall following
                # Lấy khoảng cách hai bên để wall following
                left_distance, right_distance = self.get_side_distances(self.latest_scan)
                
                # Wall following logic
                error = left_distance - right_distance  # Dương = lệch phải, Âm = lệch trái
                
                # PID đơn giản cho wall following
                kp = 0.5
                desired_offset = error * kp
                
                # Chuyển đổi offset sang góc servo
                servo_range = (self.servo_max_angle - self.servo_min_angle) / 2.0
                normalized_error = np.clip(desired_offset / self.wall_follow_distance, -1.0, 1.0)
                target_servo_angle = self.servo_center_angle - normalized_error * servo_range * 0.3
                
                # Giới hạn góc servo
                target_servo_angle = max(self.servo_min_angle, min(self.servo_max_angle, target_servo_angle))
                
                # Làm mượt góc servo
                alpha = self.servo_angle_smoothing
                self.smoothed_servo_angle_deg = alpha * self.smoothed_servo_angle_deg + (1 - alpha) * target_servo_angle
                self.current_servo_angle = self.smoothed_servo_angle_deg
                self.servo_angle_pub.publish(Float32(data=self.smoothed_servo_angle_deg))
                
                # Tốc độ khi đi thẳng
                cmd.linear.x = self.max_linear_speed * 0.8
                cmd.angular.z = 0.0
        
        # Gửi lệnh - đảm bảo luôn publish
        self.cmd_vel_pub.publish(cmd)
        
        # Debug: Log định kỳ
        if not hasattr(self, 'last_debug_time'):
            from time import time
            self.last_debug_time = time()
        else:
            from time import time
            if time() - self.last_debug_time >= 1.0:  # Log mỗi giây
                left_dist, right_dist = self.get_side_distances(self.latest_scan) if self.latest_scan else (float('inf'), float('inf'))
                self.get_logger().info(
                    f'🎮 State: {self.obstacle_avoidance_state.name}, '
                    f'Servo: {self.current_servo_angle:.1f}°, '
                    f'Speed: {cmd.linear.x:.2f}m/s, '
                    f'Front: {self.closest_obstacle_distance*100:.0f}cm, '
                    f'L: {left_dist*100:.0f}cm, R: {right_dist*100:.0f}cm'
                )
                self.last_debug_time = time()


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
