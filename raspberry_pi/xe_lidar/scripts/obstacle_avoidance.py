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
from std_msgs.msg import Float32


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
        # Tham số PID điều khiển bám làn
        self.declare_parameter('kp', 0.5)  # Proportional gain
        self.declare_parameter('ki', 0.0)   # Integral gain
        self.declare_parameter('kd', 0.1)  # Derivative gain
        # Tham số lane detection - C càng cao thì chỉ nhận màu đen hơn (loại bỏ xám)
        # Giảm giá trị mặc định để nhận được nhiều màu đen hơn
        self.declare_parameter('lane_threshold_c', 15)  # Giá trị C trong adaptive threshold (giảm từ 25 xuống 15)
        # Tham số làm mượt (smoothing) để tránh phản ứng quá nhanh
        self.declare_parameter('lane_offset_smoothing', 0.7)  # 0.0=không smooth, 0.9=rất smooth
        self.declare_parameter('lane_dead_zone', 0.05)  # Vùng chết - bỏ qua offset nhỏ hơn giá trị này
        # Hệ số giảm tốc khi vào cua (0.0 - 1.0), ví dụ 0.5 = giảm còn 50% tốc độ khi đang đánh lái
        self.declare_parameter('cornering_speed_factor', 0.6)
        # Hệ số tốc độ khi đi thẳng (0.0 - 1.0), ví dụ 0.7 = 70% tốc độ tối đa khi đi thẳng
        self.declare_parameter('straight_speed_factor', 1.0)
        # Hệ số tốc độ khi rẽ (0.0 - 1.0), ví dụ 0.4 = 40% tốc độ tối đa khi rẽ
        self.declare_parameter('turning_speed_factor', 0.4)
        # Tham số góc servo (degree) - giới hạn và góc giữa
        self.declare_parameter('servo_center_angle', 100.0)  # Góc giữa (đi thẳng)
        self.declare_parameter('servo_min_angle', 45.0)      # Góc tối thiểu (rẽ trái tối đa)
        self.declare_parameter('servo_max_angle', 155.0)     # Góc tối đa (rẽ phải tối đa)
        # Tham số làm mượt góc servo để tránh chuyển góc quá gấp
        self.declare_parameter('servo_angle_smoothing', 0.8)  # EMA filter cho góc servo (0.0-1.0)

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
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.lane_threshold_c = int(self.get_parameter('lane_threshold_c').value)
        self.lane_offset_smoothing = float(self.get_parameter('lane_offset_smoothing').value)
        self.lane_dead_zone = float(self.get_parameter('lane_dead_zone').value)
        self.cornering_speed_factor = float(self.get_parameter('cornering_speed_factor').value)
        self.straight_speed_factor = float(self.get_parameter('straight_speed_factor').value)
        self.turning_speed_factor = float(self.get_parameter('turning_speed_factor').value)
        self.servo_center_angle = float(self.get_parameter('servo_center_angle').value)
        self.servo_min_angle = float(self.get_parameter('servo_min_angle').value)
        self.servo_max_angle = float(self.get_parameter('servo_max_angle').value)
        self.servo_angle_smoothing = float(self.get_parameter('servo_angle_smoothing').value)
        
        # PID control variables
        self.pid_integral = 0.0
        self.pid_last_error = 0.0
        self.last_control_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                                 self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        self.smoothed_lane_offset = 0.0  # Offset đã được làm mượt
        
        # Servo angle smoothing
        self.last_servo_angle_deg = self.servo_center_angle  # Khởi tạo ở góc giữa
        self.smoothed_servo_angle_deg = self.servo_center_angle
        
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
        
        # Publisher lệnh vận tốc tới Arduino (linear, angular)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        # Publisher lệnh górc servo trực tiếp (đơn vị độ) tới Arduino qua arduino_bridge
        self.servo_angle_pub = self.create_publisher(Float32, '/servo_angle_cmd', 10)
        
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
        Xu ly camera de phat hien 2 vach DEN 2 ben duong bang CONTOUR DETECTION.
        Cach nay to den toan bo vung vach va tinh duong trung diem chinh xac hon.
        """
        if image is None:
            return

        try:
            height, width = image.shape[:2]
            image_with_lanes = image.copy()
            center_x = width / 2

            # Tao vung quan tam (ROI) - phan duoi anh (vung duong)
            roi_top = int(height * 0.4)  # Bat dau tu 40% chieu cao
            roi = image[roi_top:height, :]
            roi_height = height - roi_top

            # Chuyen sang Grayscale de phat hien vach den
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Ap dung Gaussian blur truoc de giam nhieu
            gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Dung Adaptive Threshold de tu dong dieu chinh theo anh sang
            # C cao hon = chi nhan mau den hon (loai bo xam)
            # Giảm C để nhận được nhiều màu đen hơn (bao gồm cả xám đen)
            black_mask = cv2.adaptiveThreshold(
                gray_blurred,
                255,
                cv2.ADAPTIVE_THRESH_MEAN_C,
                cv2.THRESH_BINARY_INV,
                blockSize=25,
                C=self.lane_threshold_c
            )
            
            # Thử thêm threshold đơn giản để bắt màu đen nếu adaptive threshold không đủ
            # Nếu adaptive threshold quá khó, dùng thêm simple threshold
            _, simple_thresh = cv2.threshold(gray_blurred, 80, 255, cv2.THRESH_BINARY_INV)
            # Kết hợp cả hai mask (OR operation)
            black_mask = cv2.bitwise_or(black_mask, simple_thresh)

            # Morphological operations de lam sach mask va noi cac vung gan nhau
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

            # Tim tat ca contours (vung den)
            contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Loc contours theo dien tich (loai bo nhieu nho) - giảm ngưỡng để nhận được vạch nhỏ hơn
            min_contour_area = 200  # pixels - giảm từ 500 xuống 200 để nhận được vạch nhỏ hơn
            valid_contours = [c for c in contours if cv2.contourArea(c) > min_contour_area]

            # Phan loai contours thanh ben trai va ben phai
            left_contours = []
            right_contours = []

            for contour in valid_contours:
                # Tinh centroid cua contour
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    # Phan loai theo vi tri x cua centroid
                    if cx < center_x * 0.8:  # Vung trai (< 40% width)
                        left_contours.append(contour)
                    elif cx > center_x * 1.2:  # Vung phai (> 60% width)
                        right_contours.append(contour)
                    # Bo qua vung giua (40% - 60%) de tranh nhieu

            # Ve contours len anh debug (xanh duong = trai, do = phai)
            if left_contours:
                cv2.drawContours(image_with_lanes,
                               [c + np.array([[0, roi_top]]) for c in left_contours],
                               -1, (255, 0, 0), cv2.FILLED)
            if right_contours:
                cv2.drawContours(image_with_lanes,
                               [c + np.array([[0, roi_top]]) for c in right_contours],
                               -1, (0, 0, 255), cv2.FILLED)

            # Tính toán lane center tại nhiều điểm dọc theo vạch kẻ đường để phản ánh độ cong
            # Chia ROI thành nhiều vùng ngang để tính midpoint tại mỗi vùng
            num_horizontal_zones = 5  # Chia thành 5 vùng từ dưới lên trên
            lane_centers = []  # Lưu các midpoint tại các vùng khác nhau
            lane_center = None  # Lane center cuối cùng để hiển thị
            
            # Tính midpoint tại mỗi vùng ngang
            for zone_idx in range(num_horizontal_zones):
                zone_top = int(roi_height * (zone_idx / num_horizontal_zones))
                zone_bottom = int(roi_height * ((zone_idx + 1) / num_horizontal_zones))
                
                left_x_in_zone = None
                right_x_in_zone = None
                
                # Tìm điểm x của vạch trái trong vùng này
                if left_contours:
                    all_left_points = np.vstack(left_contours)
                    zone_left_points = all_left_points[
                        (all_left_points[:, :, 1] >= zone_top) & 
                        (all_left_points[:, :, 1] < zone_bottom)
                    ]
                    if len(zone_left_points) > 0:
                        # Lấy trung bình x của các điểm trong vùng này
                        left_x_in_zone = np.mean(zone_left_points[:, 0])
                
                # Tìm điểm x của vạch phải trong vùng này
                if right_contours:
                    all_right_points = np.vstack(right_contours)
                    zone_right_points = all_right_points[
                        (all_right_points[:, :, 1] >= zone_top) & 
                        (all_right_points[:, :, 1] < zone_bottom)
                    ]
                    if len(zone_right_points) > 0:
                        right_x_in_zone = np.mean(zone_right_points[:, 0])
                
                # Tính midpoint tại vùng này nếu có cả 2 vạch
                if left_x_in_zone is not None and right_x_in_zone is not None:
                    zone_center = (left_x_in_zone + right_x_in_zone) / 2
                    zone_y = (zone_top + zone_bottom) / 2 + roi_top  # Y trong toàn bộ ảnh
                    lane_centers.append((zone_center, zone_y))
                elif left_x_in_zone is not None:
                    # Chỉ có vạch trái - ước tính
                    estimated_lane_width = 200  # pixels
                    zone_center = left_x_in_zone + estimated_lane_width
                    zone_y = (zone_top + zone_bottom) / 2 + roi_top
                    lane_centers.append((zone_center, zone_y))
                elif right_x_in_zone is not None:
                    # Chỉ có vạch phải - ước tính
                    estimated_lane_width = 200  # pixels
                    zone_center = right_x_in_zone - estimated_lane_width
                    zone_y = (zone_top + zone_bottom) / 2 + roi_top
                    lane_centers.append((zone_center, zone_y))
            
            # Tính lane center và offset từ các điểm đã tìm được
            if len(lane_centers) > 0:
                # Ưu tiên điểm ở giữa ROI (vùng 2 và 3) để tính offset chính xác hơn
                # Hoặc có thể dùng trung bình có trọng số (điểm gần camera có trọng số cao hơn)
                mid_zones = [i for i in range(len(lane_centers)) if i >= len(lane_centers) // 3 and i <= 2 * len(lane_centers) // 3]
                if mid_zones:
                    # Dùng các điểm ở giữa để tính offset
                    selected_centers = [lane_centers[i][0] for i in mid_zones]
                    lane_center = np.mean(selected_centers)
                else:
                    # Dùng tất cả các điểm
                    lane_center = np.mean([lc[0] for lc in lane_centers])
                
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
                
                # Vẽ đường midpoint cong lên ảnh để debug
                if len(lane_centers) > 1:
                    points = np.array([[int(lc[0]), int(lc[1])] for lc in lane_centers], np.int32)
                    cv2.polylines(image_with_lanes, [points], False, (0, 255, 255), 3)
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0

            # Chỉ vẽ đường trung điểm cong (polylines) - không vẽ đường thẳng
            # Đường trung điểm cong đã được vẽ ở trên bằng polylines màu vàng

            # Ap dung bo loc lam muot (exponential moving average)
            alpha = self.lane_offset_smoothing
            self.smoothed_lane_offset = alpha * self.smoothed_lane_offset + (1 - alpha) * self.lane_center_offset

            # Tính toán góc servo từ PID (sẽ được tính trong control_loop, nhưng cần để hiển thị)
            # Tạm thời dùng smoothed_lane_offset để hiển thị
            steering_offset = -self.smoothed_lane_offset
            
            # Ve text thong tin
            status_text = "LANE OK" if self.lane_detected else "NO LANE"
            left_text = f"L:{len(left_contours)}" if left_contours else "L:--"
            right_text = f"R:{len(right_contours)}" if right_contours else "R:--"
            offset_text = f"Raw:{self.lane_center_offset:.3f} Smooth:{self.smoothed_lane_offset:.3f}"
            servo_text = f"Servo: {self.smoothed_servo_angle_deg:.1f}°"
            contour_count_text = f"Contours: {len(valid_contours)} (L:{len(left_contours)} R:{len(right_contours)})"
            
            # Tính error để hiển thị
            if self.lane_detected:
                raw_error = -self.smoothed_lane_offset
                if abs(raw_error) < self.lane_dead_zone:
                    error_display = raw_error * (abs(raw_error) / self.lane_dead_zone) if self.lane_dead_zone > 0 else raw_error
                else:
                    error_display = raw_error
                error_text = f"Error: {error_display:.3f} (DeadZone: {self.lane_dead_zone:.2f})"
            else:
                error_text = "Error: --"

            cv2.putText(image_with_lanes, f"{status_text} | {left_text} | {right_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_with_lanes, offset_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_with_lanes, error_text, (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.putText(image_with_lanes, servo_text, (10, 120),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(image_with_lanes, contour_count_text, (10, 150),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            
            # Vẽ mask để debug (hiển thị ở góc trên bên phải)
            mask_display = cv2.resize(black_mask, (160, 120))
            mask_colored = cv2.cvtColor(mask_display, cv2.COLOR_GRAY2BGR)
            image_with_lanes[10:130, width-170:width-10] = mask_colored
            cv2.putText(image_with_lanes, "MASK", (width-165, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # Ve huong can di
            if self.lane_detected:
                # Tính toán góc servo từ offset để hiển thị hướng
                # Tính góc servo từ offset (sẽ được tính chính xác trong control_loop)
                servo_offset_from_center = self.smoothed_servo_angle_deg - self.servo_center_angle
                normalized_offset = servo_offset_from_center / ((self.servo_max_angle - self.servo_min_angle) / 2)
                
                # Tinh vi tri de ve mui ten chi huong
                arrow_x = int(center_x)
                arrow_y = int(height * 0.75)
                offset_pixels = int(normalized_offset * width * 0.3)
                arrow_end_x = arrow_x + offset_pixels
                arrow_end_y = arrow_y - 60

                # Ve mui ten chi huong (mau vang)
                if abs(offset_pixels) > 5:
                    cv2.arrowedLine(image_with_lanes,
                                   (arrow_x, arrow_y),
                                   (arrow_end_x, arrow_end_y),
                                   (0, 255, 255), 5, tipLength=0.3)

                # Text huong di - dựa trên góc servo thực tế
                # servo_offset_from_center < 0: góc nhỏ hơn center -> rẽ trái
                # servo_offset_from_center > 0: góc lớn hơn center -> rẽ phải
                if abs(servo_offset_from_center) < 2.0:  # Gần góc giữa
                    direction_text = "DI THANG"
                    direction_color = (0, 255, 0)
                elif servo_offset_from_center < 0:  # Góc < center -> rẽ trái
                    direction_text = f"RE TRAI ({abs(servo_offset_from_center):.1f}°)"
                    direction_color = (255, 165, 0)
                else:  # Góc > center -> rẽ phải
                    direction_text = f"RE PHAI ({servo_offset_from_center:.1f}°)"
                    direction_color = (255, 165, 0)

                cv2.putText(image_with_lanes, direction_text, (10, 180),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)
            else:
                cv2.putText(image_with_lanes, "KHONG THAY LANE", (10, 180),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Publish anh da ve
            if hasattr(self, 'image_debug_pub'):
                ros_image = self.cv2_to_imgmsg(image_with_lanes, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_link_optical"
                self.image_debug_pub.publish(ros_image)

        except Exception as e:
            self.get_logger().debug(f'Loi xu ly camera: {str(e)}')
            self.lane_detected = False
            self.lane_center_offset = 0.0
    
    def calculate_servo_angle_from_pid(self, error, dt):
        """
        Tính toán góc servo từ lane offset error bằng PID control
        
        Args:
            error: Lane offset error (-1.0 đến 1.0, âm = lệch trái, dương = lệch phải)
            dt: Delta time (seconds)
        
        Returns:
            Góc servo (degrees) trong giới hạn [servo_min_angle, servo_max_angle]
        """
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (với anti-windup)
        self.pid_integral += error * dt
        # Giới hạn integral để tránh windup
        max_integral = 1.0 / max(self.ki, 0.001) if self.ki > 0 else 10.0
        self.pid_integral = max(-max_integral, min(max_integral, self.pid_integral))
        i_term = self.ki * self.pid_integral
        
        # Derivative term
        d_error = (error - self.pid_last_error) / max(dt, 0.001)
        d_term = self.kd * d_error
        self.pid_last_error = error
        
        # Tổng PID output (normalized từ -1.0 đến 1.0)
        pid_output = p_term + i_term + d_term
        
        # Tăng độ nhạy để servo quay mạnh hơn
        # Nhân với hệ số để tăng phản ứng của servo
        pid_output = pid_output * 1.5  # Tăng 50% độ nhạy
        
        # Đảm bảo PID output có đủ độ lớn để điều khiển servo
        # Nếu error nhỏ nhưng không zero, vẫn cần có output nhỏ
        if abs(error) > 0.001 and abs(pid_output) < 0.01:
            # Tăng độ nhạy cho error nhỏ
            pid_output = pid_output * 10.0
        
        pid_output = max(-1.0, min(1.0, pid_output))  # Clamp
        
        # Chuyển đổi sang góc servo
        # pid_output = -1.0 -> servo_max_angle (rẽ trái tối đa)
        # pid_output = 0.0  -> servo_center_angle (đi thẳng)
        # pid_output = 1.0  -> servo_min_angle (rẽ phải tối đa)
        # Đảo dấu để phù hợp với hardware
        servo_range = (self.servo_max_angle - self.servo_min_angle) / 2.0
        servo_angle = self.servo_center_angle - pid_output * servo_range
        
        # Clamp vào giới hạn
        servo_angle = max(self.servo_min_angle, min(self.servo_max_angle, servo_angle))
        
        return servo_angle
    
    def control_loop(self):
        """
        Vòng lặp điều khiển chính cho Ackermann Steering:
        - ƯU TIÊN 1 (CAO - SAFETY): LiDAR để tránh vật cản
        - ƯU TIÊN 2 (THẤP - NAVIGATION): Camera để đi đúng làn đường (lane following)
        """
        cmd = Twist()

        # Tính delta thời gian cho PID (giả sử timer 0.1s, nhưng vẫn đo chính xác)
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        dt = max(0.01, current_time - self.last_control_time)
        self.last_control_time = current_time
        
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
            # KHÔNG có vật cản (hoặc đã tắt LiDAR) - Sử dụng PID để điều khiển góc servo
            if self.use_camera and self.lane_detected:
                # Tính toán error từ lane offset
                # smoothed_lane_offset: âm = lệch trái, dương = lệch phải
                # error cho PID: dương = cần rẽ phải, âm = cần rẽ trái
                raw_error = -self.smoothed_lane_offset  # Đảo dấu để phù hợp với PID
                
                # Áp dụng dead zone - nhưng KHÔNG zero error, chỉ giảm độ nhạy
                # Dead zone chỉ làm giảm error nhỏ, không loại bỏ hoàn toàn
                if abs(raw_error) < self.lane_dead_zone:
                    # Giảm error nhưng không zero để PID vẫn hoạt động
                    error = raw_error * (abs(raw_error) / self.lane_dead_zone) if self.lane_dead_zone > 0 else raw_error
                else:
                    error = raw_error
                
                # Tính góc servo từ PID
                target_servo_angle = self.calculate_servo_angle_from_pid(error, dt)
                
                # Làm mượt góc servo để tránh chuyển góc quá gấp
                alpha = self.servo_angle_smoothing
                self.smoothed_servo_angle_deg = alpha * self.smoothed_servo_angle_deg + (1 - alpha) * target_servo_angle
                
                # Gửi góc servo tới Arduino
                self.last_servo_angle_deg = self.smoothed_servo_angle_deg
                self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))
                
                # Tính tốc độ dựa trên góc lái (giảm tốc khi vào cua, giảm tốc khi đi thẳng)
                servo_offset_from_center = abs(self.smoothed_servo_angle_deg - self.servo_center_angle)
                max_servo_offset = (self.servo_max_angle - self.servo_min_angle) / 2.0
                
                # Nếu đi thẳng (góc servo gần center)
                if servo_offset_from_center < 3.0:  # Trong vùng 3 độ từ center
                    # Áp dụng hệ số tốc độ khi đi thẳng
                    speed_factor = self.straight_speed_factor
                else:
                    # Đang rẽ - tính hệ số giảm tốc dựa trên góc lái
                    # Interpolate giữa turning_speed_factor (khi rẽ nhiều) và straight_speed_factor (khi rẽ ít)
                    normalized_offset = servo_offset_from_center / max_servo_offset  # 0.0 đến 1.0
                    # Khi rẽ nhiều (normalized_offset = 1.0) -> dùng turning_speed_factor
                    # Khi rẽ ít (normalized_offset gần 0) -> dùng straight_speed_factor
                    speed_factor = self.turning_speed_factor + (self.straight_speed_factor - self.turning_speed_factor) * (1.0 - normalized_offset)
                    speed_factor = max(self.turning_speed_factor, min(self.straight_speed_factor, speed_factor))
                
                cmd.linear.x = self.max_linear_speed * speed_factor
                cmd.angular.z = 0.0  # Không dùng angular, chỉ dùng góc servo trực tiếp
                
                # Log định kỳ về lane detection (mỗi 2 giây)
                if current_time - self.last_lane_log_time >= 2.0:
                    servo_offset = self.smoothed_servo_angle_deg - self.servo_center_angle
                    if abs(servo_offset) < 2.0:
                        direction_str = "DI THANG"
                    elif servo_offset < 0:
                        direction_str = f"RE TRAI ({abs(servo_offset):.1f}°)"
                    else:
                        direction_str = f"RE PHAI ({servo_offset:.1f}°)"
                    
                    self.get_logger().info(
                        f'📷 Lane - RawOffset: {self.smoothed_lane_offset:.3f}, RawError: {raw_error:.3f}, '
                        f'Error: {error:.3f}, Servo: {self.smoothed_servo_angle_deg:.1f}°, '
                        f'{direction_str}, Speed: {cmd.linear.x:.2f} m/s'
                    )
                    self.last_lane_log_time = current_time
            else:
                # Không phát hiện được vạch kẻ đường -> DỪNG LẠI
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                # Đặt servo về góc giữa khi không thấy lane
                self.smoothed_servo_angle_deg = self.servo_center_angle
                self.last_servo_angle_deg = self.servo_center_angle
                self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))
                
                if self.use_camera:
                    # Log định kỳ khi không phát hiện lane (mỗi 2 giây)
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
