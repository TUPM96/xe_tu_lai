#!/usr/bin/env python3
"""
Node xử lý xe tự lái với Ackermann Steering:
- Camera: Phát hiện vạch kẻ đường và điều chỉnh để đi giữa đường (Lane Following)
- LiDAR: Phát hiện và tránh vật cản (Obstacle Avoidance)
- Priority: LiDAR (safety) > Camera (navigation)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
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
        self.declare_parameter('max_linear_speed', 1.0)  # Tốc độ tối đa (m/s) - PWM 255
        self.declare_parameter('max_angular_speed', 1.0)  # Tốc độ quay tối đa (rad/s)
        self.declare_parameter('front_angle_range', 60)  # Góc phía trước để kiểm tra (degrees)
        self.declare_parameter('use_camera', True)  # Sử dụng camera hay không
        self.declare_parameter('use_lidar', True)   # Sử dụng LiDAR để tránh vật cản hay không
        self.declare_parameter('camera_topic', '/camera/image_raw')  # Topic camera
        self.declare_parameter('max_steer_angle', 0.5236)  # Góc lái tối đa (rad) ~30 degrees
        self.declare_parameter('debug_camera', False)  # Hiển thị debug camera output
        # Tham số điều khiển bám làn (P thuần)
        self.declare_parameter('kp', 0.5)
        # Tham số lane detection
        # Ngưỡng nhị phân cho vạch (tối trên nền sáng)
        self.declare_parameter('binary_threshold', 70)
        # Kích thước contour tối thiểu để coi là vạch
        self.declare_parameter('contour_min_height', 30)
        self.declare_parameter('contour_min_area', 500)
        # Tham số làm mượt (smoothing) để tránh phản ứng quá nhanh
        self.declare_parameter('lane_offset_smoothing', 0.7)  # 0.0=không smooth, 0.9=rất smooth
        self.declare_parameter('lane_dead_zone', 0.05)  # Vùng chết - bỏ qua offset nhỏ hơn giá trị này
        # Hệ số giảm tốc khi vào cua (0.0 - 1.0), ví dụ 0.5 = giảm còn 50% tốc độ khi đang đánh lái
        self.declare_parameter('cornering_speed_factor', 0.6)

        self.min_distance = self.get_parameter('min_distance').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.front_angle_range = self.get_parameter('front_angle_range').value
        self.use_camera = self.get_parameter('use_camera').value
        self.use_lidar = self.get_parameter('use_lidar').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.debug_camera = self.get_parameter('debug_camera').value
        self.kp = float(self.get_parameter('kp').value)
        self.binary_threshold = int(self.get_parameter('binary_threshold').value)
        self.contour_min_height = int(self.get_parameter('contour_min_height').value)
        self.contour_min_area = int(self.get_parameter('contour_min_area').value)
        self.lane_offset_smoothing = float(self.get_parameter('lane_offset_smoothing').value)
        self.lane_dead_zone = float(self.get_parameter('lane_dead_zone').value)
        self.cornering_speed_factor = float(self.get_parameter('cornering_speed_factor').value)
        self.last_control_time = float(self.get_clock().now().seconds_nanoseconds()[0])
        self.smoothed_lane_offset = 0.0  # Offset đã được làm mượt
        # Lưu contour lane của frame trước để tăng ổn định
        self.prev_left_contour = None
        self.prev_right_contour = None
        
        # Subscribers
        if self.use_lidar:
            self.scan_sub = self.create_subscription(
                LaserScan,
                '/scan',
                self.scan_callback,
                10
            )
            self.get_logger().info('Da subscribe topic /scan cho LiDAR')
        else:
            self.scan_sub = None
            self.get_logger().info('Bo qua LiDAR (use_lidar=false) - chi su dung camera de bam lan')
        
        if self.use_camera:
            camera_topic = self.get_parameter('camera_topic').value
            self.image_sub = self.create_subscription(
                Image,
                camera_topic,
                self.image_callback,
                10
            )
            self.get_logger().info(f'Da subscribe topic {camera_topic} cho Camera')
            self.latest_image = None
            
            # Publisher cho ảnh camera đã vẽ lane detection
            self.image_debug_pub = self.create_publisher(Image, '/camera/image_debug', 10)
        
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
        self.camera_received_count = 0  # Đếm số frame đã nhận từ camera
        self.last_lane_log_time = 0.0  # Thời gian log cuối cùng về lane
        
        # Timer để xuất lệnh điều khiển
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('Autonomous Drive Node da khoi dong!')
        self.get_logger().info(f'Camera: {self.use_camera}, LiDAR: Enabled')
        self.get_logger().info(f'Safe distance: {self.safe_distance}m')
    
    def scan_callback(self, msg):
        """Callback xu ly du lieu LiDAR de phat hien vat can"""
        if self.latest_scan is None:
            self.get_logger().info('Da nhan duoc du lieu LiDAR lan dau!')
        self.latest_scan = msg
        self.process_lidar_data(msg)
    
    def imgmsg_to_cv2(self, img_msg, encoding="bgr8"):
        """
        Convert ROS2 Image message sang OpenCV image (numpy array)
        KHÔNG CẦN cv_bridge!
        """
        if encoding == "bgr8" or encoding == "rgb8":
            # Convert bytes to numpy array
            dtype = np.uint8
            img_buf = np.frombuffer(img_msg.data, dtype=dtype)
            # Reshape to image dimensions
            if img_msg.height * img_msg.width * 3 == len(img_buf):
                img_buf = img_buf.reshape((img_msg.height, img_msg.width, 3))
                # BGR8 is default, RGB8 needs conversion
                if encoding == "rgb8":
                    img_buf = cv2.cvtColor(img_buf, cv2.COLOR_RGB2BGR)
                return img_buf
            else:
                raise ValueError(f"Image size mismatch: expected {img_msg.height * img_msg.width * 3}, got {len(img_buf)}")
        else:
            raise ValueError(f"Encoding {encoding} chưa được hỗ trợ")
    
    def image_callback(self, msg):
        """Callback xử lý dữ liệu camera để phát hiện vạch kẻ đường"""
        try:
            # Convert ROS Image message sang OpenCV image (KHÔNG CẦN cv_bridge)
            cv_image = self.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            self.camera_received_count += 1
            
            # Log khi nhận ảnh camera lần đầu
            if self.camera_received_count == 1:
                self.get_logger().info('Da nhan duoc anh camera lan dau!')
            
            if self.use_camera:
                self.process_camera_lane_detection(cv_image)
        except Exception as e:
            self.get_logger().error(f'Loi xu ly anh: {str(e)}')
    
    def process_lidar_data(self, scan):
        """Xử lý dữ liệu LiDAR để phát hiện vật cản"""
        if not self.use_lidar or scan is None:
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
    
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        """Convert OpenCV image sang ROS2 Image message"""
        img_msg = Image()
        img_msg.height, img_msg.width = cv_image.shape[:2]
        
        if encoding == "bgr8":
            img_msg.encoding = "bgr8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3
            img_msg.data = cv_image.tobytes()
        elif encoding == "rgb8":
            img_msg.encoding = "rgb8"
            img_msg.is_bigendian = 0
            img_msg.step = img_msg.width * 3
            img_msg.data = cv_image.tobytes()
        else:
            raise ValueError(f"Encoding {encoding} chua duoc ho tro")
        
        return img_msg
    
    def process_camera_lane_detection(self, image):
        """
        Xử lý camera theo kiểu "dò 2 line" bằng contour:
        - Tìm vạch trái/phải bằng threshold + morphology
        - Nối các điểm giữa (mid_points) để tạo đường giữa lane
        - Tính góc của đường giữa và ánh xạ thành offset điều khiển
        """
        if image is None:
            return

        try:
            height, width = image.shape[:2]
            image_with_lanes = image.copy()

            # Chuyển sang grayscale và làm mờ
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)

            # Ngưỡng nhị phân (vạch tối trên nền sáng) - có thể chỉnh qua binary_threshold
            _, mask = cv2.threshold(
                blur,
                self.binary_threshold,
                255,
                cv2.THRESH_BINARY_INV
            )

            # Đóng morphology để nối các đoạn đứt
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Tìm contour
            contours, _ = cv2.findContours(mask.copy(),
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

            left_contour = None
            right_contour = None
            center_x = width // 2

            if len(contours) > 0:
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    # Lọc bỏ nhiễu nhỏ bằng tham số contour_min_height/area
                    if h > self.contour_min_height and cv2.contourArea(contour) > self.contour_min_area:
                        cx = x + w // 2
                        if cx < center_x:
                            left_contour = contour
                        elif cx > center_x:
                            right_contour = contour

            # Nếu frame hiện tại mất vạch thì dùng lại contour frame trước
            if left_contour is None and self.prev_left_contour is not None:
                left_contour = self.prev_left_contour
            if right_contour is None and self.prev_right_contour is not None:
                right_contour = self.prev_right_contour

            self.prev_left_contour = left_contour
            self.prev_right_contour = right_contour

            mid_points = []
            angle_deg = 0.0
            huong = "Di thang"

            if left_contour is not None and right_contour is not None:
                # Vẽ 2 vạch
                cv2.drawContours(image_with_lanes, [left_contour], -1, (255, 0, 0), 2)
                cv2.drawContours(image_with_lanes, [right_contour], -1, (0, 0, 255), 2)

                left_points = left_contour[:, 0, :]
                right_points = right_contour[:, 0, :]
                left_points = sorted(left_points, key=lambda p: p[1])
                right_points = sorted(right_points, key=lambda p: p[1])

                lp_idx, rp_idx = 0, 0
                while lp_idx < len(left_points) and rp_idx < len(right_points):
                    lp = left_points[lp_idx]
                    rp = right_points[rp_idx]
                    # Ghép các điểm có cùng cao độ (y gần nhau)
                    if abs(lp[1] - rp[1]) < 5:
                        mid_x = (lp[0] + rp[0]) // 2
                        mid_y = lp[1]
                        mid_points.append((mid_x, mid_y))
                        lp_idx += 1
                        rp_idx += 1
                    elif lp[1] < rp[1]:
                        lp_idx += 1
                    else:
                        rp_idx += 1

                # Vẽ đường giữa (midline)
                for i in range(1, len(mid_points)):
                    cv2.line(image_with_lanes, mid_points[i - 1], mid_points[i],
                             (0, 255, 0), 2)

                if len(mid_points) > 2:
                    pt_start = mid_points[0]
                    pt_end = mid_points[-1]
                    cv2.circle(image_with_lanes, pt_start, 7, (0, 255, 255), -1)
                    cv2.circle(image_with_lanes, pt_end, 7, (0, 255, 255), -1)
                    cv2.line(image_with_lanes, pt_start, pt_end, (0, 255, 255), 3)

                    dx = pt_end[0] - pt_start[0]
                    dy = pt_end[1] - pt_start[1]
                    if dy != 0:
                        angle_deg = float(np.degrees(np.arctan2(-dx, dy)))
                    else:
                        angle_deg = 0.0

                    # Giảm bớt độ nhạy góc
                    angle_deg *= 0.7

                    # Ngưỡng để xác định rẽ hay đi thẳng
                    angle_threshold = 7.0
                    if angle_deg < -angle_threshold:
                        huong = "Re trai"
                    elif angle_deg > angle_threshold:
                        huong = "Re phai"
                    else:
                        huong = "Di thang"

                    # Ánh xạ góc sang offset chuẩn hóa [-1, 1] cho bộ điều khiển
                    MAX_VISUAL_ANGLE = 45.0
                    offset = np.clip(angle_deg / MAX_VISUAL_ANGLE, -1.0, 1.0)
                    self.lane_center_offset = offset
                    self.lane_detected = True
                else:
                    self.lane_detected = False
                    self.lane_center_offset = 0.0
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0

            # Bộ lọc mượt EMA cho offset
            alpha = self.lane_offset_smoothing
            self.smoothed_lane_offset = alpha * self.smoothed_lane_offset + (1 - alpha) * self.lane_center_offset

            # Text debug
            status_text = "Phat hien lan duong" if self.lane_detected else "Khong phat hien lan"
            offset_text = f"Raw: {self.lane_center_offset:.2f} | Smooth: {self.smoothed_lane_offset:.2f} | Angle:{angle_deg:.1f}"
            cv2.putText(image_with_lanes, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image_with_lanes, offset_text, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Mũi tên chỉ hướng dựa trên angle_deg (không dùng dead zone hiển thị)
            arrow_x = int(width / 2)
            arrow_y = int(height * 0.75)
            steering_offset = np.clip(angle_deg / 45.0, -1.0, 1.0)
            offset_pixels = int(steering_offset * width * 0.4)
            arrow_end_x = arrow_x + offset_pixels
            arrow_end_y = arrow_y - 60

            if abs(offset_pixels) > 3:
                cv2.arrowedLine(image_with_lanes,
                                (arrow_x, arrow_y),
                                (arrow_end_x, arrow_end_y),
                                (0, 255, 255), 5, tipLength=0.3)

            direction_text = huong
            if huong == "Di thang":
                direction_color = (0, 255, 0)
            else:
                direction_color = (255, 165, 0)
            cv2.putText(image_with_lanes, direction_text, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)

            # Publish ảnh debug
            if hasattr(self, 'image_debug_pub'):
                ros_image = self.cv2_to_imgmsg(image_with_lanes, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link_optical"
                self.image_debug_pub.publish(ros_image)

        except Exception as e:
            self.get_logger().debug(f'Loi xu ly camera: {str(e)}')
            self.lane_detected = False
            self.lane_center_offset = 0.0
    
    def control_loop(self):
        """
        Vòng lặp điều khiển chính cho Ackermann Steering:
        - ƯU TIÊN 1 (CAO - SAFETY): LiDAR để tránh vật cản
        - ƯU TIÊN 2 (THẤP - NAVIGATION): Camera để đi đúng làn đường (lane following)
        """
        cmd = Twist()

        # Tính delta thời gian cho PID (giả sử timer 0.1s, nhưng vẫn đo chính xác)
        now = self.get_clock().now().seconds_nanoseconds()[0]
        dt = max(0.01, now - getattr(self, "last_control_time", now))
        self.last_control_time = now
        
        # Nếu đang dùng LiDAR, xử lý trường hợp chưa có dữ liệu LiDAR (chạy chậm để an toàn)
        if self.use_lidar:
            if self.latest_scan is None:
                if self.lidar_warning_count % 20 == 0:
                    self.get_logger().warn('Chua nhan duoc du lieu LiDAR, chay cham de an toan...')
                self.lidar_warning_count += 1

                # Chay cham khi chua co LiDAR (toc do 50% de an toan)
                cmd.linear.x = self.max_linear_speed * 0.5
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return

            # Reset counter khi da co du lieu
            if self.lidar_warning_count > 0:
                self.lidar_warning_count = 0
                self.get_logger().info('Da nhan duoc du lieu LiDAR, chuyen sang che do tu dong!')
        
        # UU TIEN PHU: Kiem tra vat can bang LiDAR (chi khi co vat can moi can thiep)
        if self.use_lidar and self.obstacle_detected:
            # Co vat can, thuc hien tranh (tam thoi bo qua camera)
            if self.obstacle_direction == 0:
                # Vat can o giua hoac ca hai ben, lui lai va quay
                cmd.linear.x = -self.max_linear_speed * 0.5
                cmd.angular.z = self.max_angular_speed * 0.8
                self.get_logger().info('⚠️ Vat can phia truoc - Lui lai va quay phai')
            elif self.obstacle_direction < 0:
                # Vat can ben trai, quay phai de tranh
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = -self.max_angular_speed * 0.7
                self.get_logger().info('⚠️ Vat can ben trai - Quay phai de tranh')
            else:
                # Vat can ben phai, quay trai de tranh
                cmd.linear.x = self.max_linear_speed * 0.6
                cmd.angular.z = self.max_angular_speed * 0.7
                self.get_logger().info('⚠️ Vat can ben phai - Quay trai de tranh')
        else:
            # KHÔNG có vật cản (hoặc đã tắt LiDAR) - ƯU TIÊN: Camera để đi đúng làn đường
            if self.use_camera and self.lane_detected:
                # Điều chỉnh để đi giữa đường dựa trên camera (lane following)
                cmd.linear.x = self.max_linear_speed

                # Điều chỉnh góc quay dựa trên offset từ giữa đường (Ackermann steering)
                # QUAN TRỌNG:
                # - lane_center_offset > 0: xe lệch PHẢI → cần quay TRÁI (angular.z > 0)
                # - lane_center_offset < 0: xe lệch TRÁI → cần quay PHẢI (angular.z < 0)
                # - ROS2 convention: angular.z dương = quay trái, angular.z âm = quay phải

                # Điều khiển P thuần cho bám làn - dùng smoothed offset để tránh giật
                error = float(self.smoothed_lane_offset)
                # Áp dụng dead zone CHỈ cho điều khiển (không thay đổi smoothed gốc)
                if abs(error) < self.lane_dead_zone:
                    error = 0.0

                # Điều khiển P: angular ~ Kp * error (đã nhân max_angular_speed)
                # Đảo dấu để khớp với hướng servo/Arduino thực tế
                desired_angular = -self.kp * error * self.max_angular_speed

                # Giới hạn angular velocity theo max_steer_angle của Ackermann
                # Giả sử wheelbase = 0.4m, vận tốc max = 0.3 m/s
                # max_angular = v / (wheelbase / tan(max_steer_angle))
                # Nhưng để đơn giản, giới hạn bằng tỷ lệ của max_angular_speed
                max_angular_for_ackermann = self.max_angular_speed * 0.9  # 90% để an toàn
                cmd.angular.z = max(-max_angular_for_ackermann,
                                   min(max_angular_for_ackermann, desired_angular))

                # Giảm tốc độ khi đang vào cua (đang đánh lái)
                if abs(cmd.angular.z) > 0.01:
                    # cornering_speed_factor trong khoảng (0.0 - 1.0)
                    # Ví dụ: 0.6 = chạy 60% tốc độ khi vào cua
                    cmd.linear.x = self.max_linear_speed * self.cornering_speed_factor

                # Log định kỳ về lane detection (mỗi 2 giây)
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                if current_time - self.last_lane_log_time >= 2.0:
                    self.get_logger().info(
                        f'📷 Lane - Raw: {self.lane_center_offset:.2f}, '
                        f'Smooth: {self.smoothed_lane_offset:.2f}, '
                        f'Angular: {cmd.angular.z:.2f} rad/s'
                    )
                    self.last_lane_log_time = current_time
            else:
                # Khong phat hien duoc vach ke duong -> DUNG LAI (an toan hon la di thang)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if self.use_camera:
                    # Log định kỳ khi không phát hiện lane (mỗi 2 giây)
                    current_time = self.get_clock().now().seconds_nanoseconds()[0]
                    if current_time - self.last_lane_log_time >= 2.0:
                        self.get_logger().warn('📷 Khong phat hien lan duong - DUNG LAI')
                        self.last_lane_log_time = current_time
        
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
            node.get_logger().error(f'Loi trong node: {str(e)}')
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
