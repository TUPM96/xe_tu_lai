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
from enum import Enum


class TurnState(Enum):
    """Trạng thái của state machine cho việc rẽ"""
    IDLE = 0        # Đang chạy thẳng, sẵn sàng nhận lệnh rẽ mới
    TURNING = 1     # Đang thực hiện rẽ, bỏ qua lane detection cho đến khi hoàn thành


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
        # Tham số lane detection - C càng cao thì chỉ nhận màu đen hơn (loại bỏ xám)
        self.declare_parameter('lane_threshold_c', 25)  # Giá trị C trong adaptive threshold
        # Tham số làm mượt (smoothing) để tránh phản ứng quá nhanh
        self.declare_parameter('lane_offset_smoothing', 0.7)  # 0.0=không smooth, 0.9=rất smooth
        self.declare_parameter('lane_dead_zone', 0.05)  # Vùng chết - bỏ qua offset nhỏ hơn giá trị này
        # Hệ số giảm tốc khi vào cua (0.0 - 1.0), ví dụ 0.5 = giảm còn 50% tốc độ khi đang đánh lái
        self.declare_parameter('cornering_speed_factor', 0.6)
        # Tham số điều khiển rẽ kiểu "bật công tắc"
        self.declare_parameter('turn_trigger_angular', 0.3)   # chưa dùng cho góc trực tiếp, giữ lại nếu cần
        self.declare_parameter('turn_command_angular', 0.5)   # chưa dùng khi điều khiển theo góc servo
        # Tham số góc servo (degree) - dùng khi xuất lệnh góc trực tiếp từ Python
        self.declare_parameter('servo_center_angle', 100.0)
        self.declare_parameter('servo_left_angle', 45.0)
        self.declare_parameter('servo_right_angle', 155.0)

        # Tham số state machine cho việc rẽ (hard-coded turn)
        self.declare_parameter('turn_distance', 0.5)        # Khoảng cách rẽ (m) - chạy 50cm rồi mới xét tiếp
        self.declare_parameter('turn_speed', 0.2)           # Tốc độ khi rẽ (m/s)
        self.declare_parameter('turn_trigger_threshold', 0.3)  # Ngưỡng offset để kích hoạt rẽ (0.0-1.0)
        self.declare_parameter('straight_speed_factor', 0.8)  # Hệ số tốc độ khi đi thẳng (0.0-1.0)

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
        self.lane_threshold_c = int(self.get_parameter('lane_threshold_c').value)
        self.lane_offset_smoothing = float(self.get_parameter('lane_offset_smoothing').value)
        self.lane_dead_zone = float(self.get_parameter('lane_dead_zone').value)
        self.cornering_speed_factor = float(self.get_parameter('cornering_speed_factor').value)
        self.turn_trigger_angular = float(self.get_parameter('turn_trigger_angular').value)
        self.turn_command_angular = float(self.get_parameter('turn_command_angular').value)
        self.servo_center_angle = float(self.get_parameter('servo_center_angle').value)
        self.servo_left_angle = float(self.get_parameter('servo_left_angle').value)
        self.servo_right_angle = float(self.get_parameter('servo_right_angle').value)
        self.turn_distance = float(self.get_parameter('turn_distance').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.turn_trigger_threshold = float(self.get_parameter('turn_trigger_threshold').value)
        self.straight_speed_factor = float(self.get_parameter('straight_speed_factor').value)
        self.last_servo_angle_deg = 0.0
        self.last_control_time = float(self.get_clock().now().seconds_nanoseconds()[0])
        self.smoothed_lane_offset = 0.0  # Offset đã được làm mượt

        # State machine cho việc rẽ
        self.turn_state = TurnState.IDLE
        self.turn_start_time = 0.0
        self.turn_duration = self.turn_distance / self.turn_speed  # Thời gian rẽ = quãng đường / tốc độ
        self.turn_direction = 0  # -1: trái, 0: thẳng, 1: phải
        self.turn_servo_angle = self.servo_center_angle  # Góc servo hiện tại khi rẽ
        
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
            black_mask = cv2.adaptiveThreshold(
                gray_blurred,
                255,
                cv2.ADAPTIVE_THRESH_MEAN_C,
                cv2.THRESH_BINARY_INV,
                blockSize=25,
                C=self.lane_threshold_c
            )

            # Morphological operations de lam sach mask va noi cac vung gan nhau
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
            black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

            # Tim tat ca contours (vung den)
            contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Loc contours theo dien tich (loai bo nhieu nho)
            min_contour_area = 500  # pixels
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

            # Tim diem duoi cung (gan camera nhat) cua moi ben
            left_bottom_x = None
            right_bottom_x = None

            # Tim diem duoi cung cua vach trai
            if left_contours:
                all_left_points = np.vstack(left_contours)
                # Tim diem co y lon nhat (gan duoi nhat trong ROI)
                bottom_y = np.max(all_left_points[:, :, 1])
                # Lay cac diem o hang duoi cung (trong khoang 20 pixel)
                bottom_points = all_left_points[all_left_points[:, :, 1] >= bottom_y - 20]
                if len(bottom_points) > 0:
                    # Lay trung binh x cua cac diem duoi cung
                    left_bottom_x = np.mean(bottom_points[:, 0])

            # Tim diem duoi cung cua vach phai
            if right_contours:
                all_right_points = np.vstack(right_contours)
                bottom_y = np.max(all_right_points[:, :, 1])
                bottom_points = all_right_points[all_right_points[:, :, 1] >= bottom_y - 20]
                if len(bottom_points) > 0:
                    right_bottom_x = np.mean(bottom_points[:, 0])

            # Tinh lane center va offset
            lane_center = None
            if left_bottom_x is not None and right_bottom_x is not None:
                # Co ca 2 vach - tinh trung diem
                lane_center = (left_bottom_x + right_bottom_x) / 2
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif left_bottom_x is not None:
                # Chi co vach trai - uoc tinh lane center
                lane_center = left_bottom_x + 200  # Gia su lane rong 400px
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif right_bottom_x is not None:
                # Chi co vach phai - uoc tinh lane center
                lane_center = right_bottom_x - 200
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0

            # Ve duong giua man hinh (xanh la) va lane center (vang)
            cv2.line(image_with_lanes, (int(center_x), height),
                    (int(center_x), roi_top), (0, 255, 0), 2)

            if lane_center is not None:
                cv2.line(image_with_lanes, (int(lane_center), height),
                        (int(lane_center), roi_top), (0, 255, 255), 3)

            # Ap dung bo loc lam muot (exponential moving average)
            alpha = self.lane_offset_smoothing
            self.smoothed_lane_offset = alpha * self.smoothed_lane_offset + (1 - alpha) * self.lane_center_offset

            # Ve text thong tin
            status_text = "LANE OK" if self.lane_detected else "NO LANE"
            left_text = f"L:{left_bottom_x:.0f}" if left_bottom_x else "L:--"
            right_text = f"R:{right_bottom_x:.0f}" if right_bottom_x else "R:--"
            offset_text = f"Raw:{self.lane_center_offset:.2f} Smooth:{self.smoothed_lane_offset:.2f}"

            cv2.putText(image_with_lanes, f"{status_text} | {left_text} | {right_text}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image_with_lanes, offset_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Ve huong can di
            if self.lane_detected:
                # Tinh vi tri de ve mui ten chi huong
                arrow_x = int(center_x)
                arrow_y = int(height * 0.75)

                # Dao dau de khop voi huong steering thuc te
                steering_offset = -self.smoothed_lane_offset
                offset_pixels = int(steering_offset * width * 0.4)
                arrow_end_x = arrow_x + offset_pixels
                arrow_end_y = arrow_y - 60

                # Ve mui ten chi huong (mau vang)
                if abs(offset_pixels) > 5:
                    cv2.arrowedLine(image_with_lanes,
                                   (arrow_x, arrow_y),
                                   (arrow_end_x, arrow_end_y),
                                   (0, 255, 255), 5, tipLength=0.3)

                # Text huong di
                if abs(steering_offset) < 0.02:
                    direction_text = "DI THANG"
                    direction_color = (0, 255, 0)
                elif steering_offset > 0:
                    direction_text = f"RE TRAI ({abs(steering_offset):.2f})"
                    direction_color = (255, 165, 0)
                else:
                    direction_text = f"RE PHAI ({abs(steering_offset):.2f})"
                    direction_color = (255, 165, 0)

                cv2.putText(image_with_lanes, direction_text, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)
            else:
                cv2.putText(image_with_lanes, "KHONG THAY LANE", (10, 90),
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
            # KHÔNG có vật cản (hoặc đã tắt LiDAR) - Sử dụng STATE MACHINE để điều khiển rẽ
            current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                           self.get_clock().now().seconds_nanoseconds()[1] / 1e9

            # ==================== STATE MACHINE CHO VIỆC RẼ ====================
            if self.turn_state == TurnState.TURNING:
                # Đang trong trạng thái RẼ - tiếp tục rẽ cho đến khi hoàn thành
                elapsed = current_time - self.turn_start_time

                if elapsed >= self.turn_duration:
                    # Đã rẽ xong - quay về IDLE
                    self.turn_state = TurnState.IDLE
                    self.get_logger().info(
                        f'✅ Hoàn thành rẽ {"trái" if self.turn_direction < 0 else "phải"} '
                        f'sau {elapsed:.2f}s ({self.turn_distance}m)'
                    )
                    # Về servo giữa
                    self.turn_servo_angle = self.servo_center_angle
                    cmd.linear.x = self.max_linear_speed
                else:
                    # Vẫn đang rẽ - giữ nguyên góc servo và tốc độ chậm
                    cmd.linear.x = self.turn_speed
                    remaining = self.turn_duration - elapsed

                    # Log tiến trình rẽ (mỗi 0.5 giây)
                    if int(elapsed * 2) != int((elapsed - dt) * 2):
                        self.get_logger().info(
                            f'🔄 Đang rẽ {"trái" if self.turn_direction < 0 else "phải"}: '
                            f'{elapsed:.1f}s/{self.turn_duration:.1f}s, '
                            f'còn {remaining:.1f}s, servo={self.turn_servo_angle:.0f}°'
                        )

                # Gửi góc servo cố định trong suốt quá trình rẽ
                cmd.angular.z = 0.0
                self.last_servo_angle_deg = float(self.turn_servo_angle)
                self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))

            else:
                # STATE = IDLE - xét xem có cần rẽ không
                if self.use_camera and self.lane_detected:
                    # Dùng RAW offset để trigger rẽ (phản ứng nhanh hơn)
                    raw_error = float(self.lane_center_offset)

                    # Kiểm tra có cần kích hoạt rẽ không (offset vượt ngưỡng)
                    if abs(raw_error) >= self.turn_trigger_threshold:
                        # BẮT ĐẦU RẼ - chuyển sang trạng thái TURNING
                        self.turn_state = TurnState.TURNING
                        self.turn_start_time = current_time

                        if raw_error > 0.0:
                            # Lệch sang phải -> rẽ trái để về giữa
                            self.turn_direction = -1
                            self.turn_servo_angle = self.servo_left_angle
                        else:
                            # Lệch sang trái -> rẽ phải để về giữa
                            self.turn_direction = 1
                            self.turn_servo_angle = self.servo_right_angle

                        self.get_logger().info(
                            f'🚗 Bắt đầu rẽ {"trái" if self.turn_direction < 0 else "phải"}: '
                            f'raw_offset={raw_error:.2f}, servo={self.turn_servo_angle:.0f}°, '
                            f'duration={self.turn_duration:.2f}s'
                        )

                        # Gửi lệnh rẽ
                        cmd.linear.x = self.turn_speed
                        cmd.angular.z = 0.0
                        self.last_servo_angle_deg = float(self.turn_servo_angle)
                        self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))

                    elif abs(raw_error) < self.lane_dead_zone:
                        # Đi thẳng - không cần rẽ (áp dụng straight_speed_factor)
                        cmd.linear.x = self.max_linear_speed * self.straight_speed_factor
                        cmd.angular.z = 0.0
                        self.last_servo_angle_deg = float(self.servo_center_angle)
                        self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))

                    else:
                        # Offset nhỏ - điều chỉnh nhẹ (giữa dead_zone và trigger_threshold)
                        # Vẫn chạy thẳng với tốc độ giảm nhẹ hơn nữa, servo về giữa
                        cmd.linear.x = self.max_linear_speed * self.straight_speed_factor * 0.8
                        cmd.angular.z = 0.0
                        self.last_servo_angle_deg = float(self.servo_center_angle)
                        self.servo_angle_pub.publish(Float32(data=self.last_servo_angle_deg))

                    # Log định kỳ về lane detection (mỗi 2 giây)
                    if current_time - self.last_lane_log_time >= 2.0:
                        state_str = "TURNING" if self.turn_state == TurnState.TURNING else "IDLE"
                        self.get_logger().info(
                            f'📷 [{state_str}] Lane - Raw: {raw_error:.2f}, '
                            f'Trigger: {self.turn_trigger_threshold}, '
                            f'ServoCmd: {self.last_servo_angle_deg:.1f}°'
                        )
                        self.last_lane_log_time = current_time
                else:
                    # Không phát hiện được vạch kẻ đường -> DỪNG LẠI (an toàn hơn là đi thẳng)
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
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
