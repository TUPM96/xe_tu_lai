#!/usr/bin/env python3
"""
Node xử lý xe tự lái:
- Camera: Phát hiện vạch kẻ đường và điều chỉnh để đi giữa đường
- LiDAR: Phát hiện và tránh vật cản
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class AutonomousDrive(Node):
    def __init__(self):
        super().__init__('autonomous_drive')
        
        # Parameters
        # Note: use_sim_time is set by launch file, don't declare it here
        self.declare_parameter('min_distance', 0.5)  # Khoảng cách tối thiểu để dừng (m)
        self.declare_parameter('safe_distance', 0.8)  # Khoảng cách an toàn để tránh (m)
        self.declare_parameter('max_linear_speed', 0.3)  # Tốc độ tối đa (m/s)
        self.declare_parameter('max_angular_speed', 1.0)  # Tốc độ quay tối đa (rad/s)
        self.declare_parameter('front_angle_range', 60)  # Góc phía trước để kiểm tra (degrees)
        self.declare_parameter('use_camera', True)  # Sử dụng camera hay không
        self.declare_parameter('camera_topic', '/camera/image_raw')  # Topic camera
        
        self.min_distance = self.get_parameter('min_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.front_angle_range = self.get_parameter('front_angle_range').value
        self.use_camera = self.get_parameter('use_camera').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Đã subscribe topic /scan cho LiDAR')
        
        if self.use_camera:
            camera_topic = self.get_parameter('camera_topic').value
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                10
            )
            self.bridge = CvBridge()
            self.latest_image = None
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # State variables
        self.latest_scan = None
        self.obstacle_detected = False
        self.obstacle_direction = 0.0  # -1: trái, 0: giữa, 1: phải
        self.lane_center_offset = 0.0  # Offset từ giữa đường (-1 đến 1)
        self.lane_detected = False
        self.lidar_warning_count = 0  # Đếm số lần warning để tránh spam
        
        # Timer để xuất lệnh điều khiển
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Autonomous Drive Node đã khởi động!')
        self.get_logger().info(f'Camera: {self.use_camera}, LiDAR: Enabled')
        self.get_logger().info(f'Safe distance: {self.safe_distance}m')
    
    def scan_callback(self, msg):
        """Callback xử lý dữ liệu LiDAR để phát hiện vật cản"""
        if self.latest_scan is None:
            self.get_logger().info('Đã nhận được dữ liệu LiDAR lần đầu!')
        self.latest_scan = msg
        self.process_lidar_data(msg)
    
    def image_callback(self, msg):
        """Callback xử lý dữ liệu camera để phát hiện vạch kẻ đường"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            if self.use_camera:
                self.process_camera_lane_detection(cv_image)
        except Exception as e:
            self.get_logger().error(f'Lỗi xử lý ảnh: {str(e)}')
    
    def process_lidar_data(self, scan):
        """Xử lý dữ liệu LiDAR để phát hiện vật cản"""
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
        
        for i in range(num_points):
            angle = angle_min + i * angle_increment
            # Kiểm tra góc phía trước (từ -front_angle/2 đến +front_angle/2)
            if abs(angle) <= front_angle_rad:
                if not (np.isinf(ranges[i]) or np.isnan(ranges[i])):
                    if ranges[i] < scan.range_max and ranges[i] > scan.range_min:
                        front_indices.append((i, ranges[i], angle))
        
        if not front_indices:
            self.obstacle_detected = False
            return
        
        # Tìm vật cản gần nhất phía trước
        min_distance = min([item[1] for item in front_indices])
        closest_obstacle = [item for item in front_indices if item[1] == min_distance][0]
        
        # Kiểm tra có vật cản không
        if min_distance < self.safe_distance:
            self.obstacle_detected = True
            # Xác định hướng vật cản
            obstacle_angle = closest_obstacle[2]
            if obstacle_angle < 0:
                self.obstacle_direction = -1  # Vật cản bên trái
            else:
                self.obstacle_direction = 1  # Vật cản bên phải
            
            # Kiểm tra vật cản ở cả hai bên
            left_obstacles = [item for item in front_indices if item[2] < 0 and item[1] < self.safe_distance]
            right_obstacles = [item for item in front_indices if item[2] > 0 and item[1] < self.safe_distance]
            
            if left_obstacles and right_obstacles:
                # Vật cản ở cả hai bên, quay lại
                self.obstacle_direction = 0
        else:
            self.obstacle_detected = False
    
    def process_camera_lane_detection(self, image):
        """Xử lý camera để phát hiện 2 vạch trắng 2 bên đường và điều chỉnh đi giữa đường"""
        if image is None:
            return
        
        try:
            height, width = image.shape[:2]
            
            # Tạo vùng quan tâm (ROI) - phần dưới ảnh (vùng đường)
            roi_top = int(height * 0.4)  # Bắt đầu từ 40% chiều cao
            roi_bottom = height
            roi = image[roi_top:roi_bottom, :]
            
            # Chuyển sang HSV để dễ phát hiện màu trắng
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Tạo mask cho màu trắng (vạch kẻ đường)
            # Màu trắng trong HSV: V cao, S thấp
            lower_white = np.array([0, 0, 200])  # Ngưỡng dưới cho màu trắng
            upper_white = np.array([180, 30, 255])  # Ngưỡng trên cho màu trắng
            white_mask = cv2.inRange(hsv, lower_white, upper_white)
            
            # Áp dụng Gaussian blur để làm mịn
            blurred = cv2.GaussianBlur(white_mask, (5, 5), 0)
            
            # Phát hiện cạnh bằng Canny
            edges = cv2.Canny(blurred, 50, 150)
            
            # Phát hiện đường thẳng bằng HoughLinesP
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=30, 
                                   minLineLength=20, maxLineGap=15)
            
            if lines is None or len(lines) == 0:
                self.lane_detected = False
                self.lane_center_offset = 0.0
                return
            
            # Phân loại đường thẳng thành bên trái và bên phải
            left_lines = []
            right_lines = []
            center_x = width / 2
            
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Tính góc và điểm giữa của đường thẳng
                if x2 != x1:
                    slope = (y2 - y1) / (x2 - x1)
                    mid_x = (x1 + x2) / 2
                    
                    # Đường bên trái: slope âm và nằm bên trái màn hình
                    # Đường bên phải: slope dương và nằm bên phải màn hình
                    if slope < -0.2 and mid_x < center_x:  # Đường bên trái
                        left_lines.append(line[0])
                    elif slope > 0.2 and mid_x > center_x:  # Đường bên phải
                        right_lines.append(line[0])
            
            # Tính điểm trung bình của các đường ở dưới cùng của ROI
            left_x_points = []
            right_x_points = []
            roi_height = roi_bottom - roi_top
            
            # Lấy điểm ở dưới cùng (y lớn nhất) của mỗi đường
            for line in left_lines:
                x1, y1, x2, y2 = line
                # Lấy điểm có y lớn hơn (gần camera hơn)
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
            
            # Tính offset từ giữa đường
            if left_x_points and right_x_points:
                # Có cả 2 vạch kẻ đường - đi giữa
                left_x_avg = np.mean(left_x_points)
                right_x_avg = np.mean(right_x_points)
                lane_center = (left_x_avg + right_x_avg) / 2
                self.lane_center_offset = (lane_center - center_x) / (width / 2)  # Normalize về -1 đến 1
                self.lane_detected = True
                self.get_logger().debug(f'Phát hiện 2 vạch: trái={left_x_avg:.1f}, phải={right_x_avg:.1f}, center={lane_center:.1f}')
            elif left_x_points:
                # Chỉ có vạch bên trái, giả định vạch phải cách 2m (khoảng 200 pixel)
                left_x_avg = np.mean(left_x_points)
                lane_center = left_x_avg + 200  # Offset sang phải
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
                self.get_logger().debug(f'Chỉ phát hiện vạch trái: {left_x_avg:.1f}')
            elif right_x_points:
                # Chỉ có vạch bên phải, giả định vạch trái cách 2m
                right_x_avg = np.mean(right_x_points)
                lane_center = right_x_avg - 200  # Offset sang trái
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
                self.get_logger().debug(f'Chỉ phát hiện vạch phải: {right_x_avg:.1f}')
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0
                
        except Exception as e:
            self.get_logger().debug(f'Lỗi xử lý camera: {str(e)}')
            self.lane_detected = False
            self.lane_center_offset = 0.0
    
    def control_loop(self):
        """
        Vòng lặp điều khiển chính:
        - ƯU TIÊN CHÍNH: Camera để đi đúng tim đường (lane following)
        - ƯU TIÊN PHỤ: LiDAR chỉ để tránh vật cản khi cần thiết
        """
        cmd = Twist()
        
        # Xử lý trường hợp chưa có LiDAR (chạy chậm để an toàn)
        if self.latest_scan is None:
            if self.lidar_warning_count % 20 == 0:
                self.get_logger().warn('Chưa nhận được dữ liệu LiDAR, chạy chậm để an toàn...')
            self.lidar_warning_count += 1
            
            # Chạy chậm khi chưa có LiDAR (tốc độ 50% để an toàn)
            cmd.linear.x = self.max_linear_speed * 0.5
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Reset counter khi đã có dữ liệu
        if self.lidar_warning_count > 0:
            self.lidar_warning_count = 0
            self.get_logger().info('Đã nhận được dữ liệu LiDAR, chuyển sang chế độ tự động!')
        
        # ƯU TIÊN PHỤ: Kiểm tra vật cản bằng LiDAR (chỉ khi có vật cản mới can thiệp)
        if self.obstacle_detected:
            # Có vật cản, thực hiện tránh (tạm thời bỏ qua camera)
            if self.obstacle_direction == 0:
                # Vật cản ở giữa hoặc cả hai bên, lùi lại và quay
                cmd.linear.x = -self.max_linear_speed * 0.5
                cmd.angular.z = self.max_angular_speed * 0.8
                self.get_logger().info('⚠️ Vật cản phía trước - Lùi lại và quay phải')
            elif self.obstacle_direction < 0:
                # Vật cản bên trái, quay phải để tránh
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = -self.max_angular_speed * 0.7
                self.get_logger().info('⚠️ Vật cản bên trái - Quay phải để tránh')
            else:
                # Vật cản bên phải, quay trái để tránh
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = self.max_angular_speed * 0.7
                self.get_logger().info('⚠️ Vật cản bên phải - Quay trái để tránh')
        else:
            # KHÔNG có vật cản - ƯU TIÊN CHÍNH: Camera để đi đúng tim đường
            if self.use_camera and self.lane_detected:
                # Điều chỉnh để đi giữa đường dựa trên camera (đi đúng tim đường)
                cmd.linear.x = self.max_linear_speed
                # Điều chỉnh góc quay dựa trên offset từ giữa đường
                # offset > 0: lệch phải -> quay trái (angular > 0)
                # offset < 0: lệch trái -> quay phải (angular < 0)
                cmd.angular.z = -self.lane_center_offset * self.max_angular_speed * 0.8
                self.get_logger().debug(f'📷 Đi theo vạch kẻ đường (Camera), offset: {self.lane_center_offset:.2f}')
            else:
                # Không phát hiện được vạch kẻ đường, đi thẳng với tốc độ đầy đủ
                cmd.linear.x = self.max_linear_speed
                cmd.angular.z = 0.0
                if self.use_camera:
                    self.get_logger().debug('📷 Không phát hiện vạch kẻ đường, đi thẳng')
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = AutonomousDrive()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Lỗi trong node: {str(e)}')
    finally:
        # Dừng robot trước khi thoát (chỉ nếu context còn valid)
        if node:
            try:
                if rclpy.ok():
                    cmd = Twist()
                    node.cmd_vel_pub.publish(cmd)
            except Exception:
                pass  # Ignore errors during shutdown
            
            try:
                node.destroy_node()
            except Exception:
                pass  # Ignore errors during node destruction
        
        # Chỉ shutdown nếu context còn valid
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore errors if already shutdown


if __name__ == '__main__':
    main()
