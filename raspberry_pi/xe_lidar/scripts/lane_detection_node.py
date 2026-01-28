#!/usr/bin/env python3
"""
Node xử lý Lane Detection riêng biệt:
- Nhận ảnh từ camera
- Phát hiện vạch kẻ đường (2 vạch đen 2 bên)
- Publish kết quả lane detection và ảnh debug
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Bool
import cv2
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection')

        # Parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('debug_camera', True)
        self.declare_parameter('roi_top_percent', 0.4)  # ROI bat dau tu % chieu cao
        self.declare_parameter('lane_width_pixels', 200)  # Khoang cach uoc tinh giua 2 vach (pixels)
        self.declare_parameter('use_full_image', False)  # Dung full anh (khong crop ROI)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.debug_camera = self.get_parameter('debug_camera').value
        self.roi_top_percent = self.get_parameter('roi_top_percent').value
        self.lane_width_pixels = self.get_parameter('lane_width_pixels').value
        self.use_full_image = self.get_parameter('use_full_image').value

        # Subscriber - Camera
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Da subscribe topic {self.camera_topic}')

        # Publishers
        self.image_debug_pub = self.create_publisher(Image, '/lane_detection/image_debug', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/lane_detection/camera_info', 10)
        self.offset_pub = self.create_publisher(Float32, '/lane_detection/offset', 10)
        self.detected_pub = self.create_publisher(Bool, '/lane_detection/detected', 10)

        # State variables
        self.lane_center_offset = 0.0
        self.lane_detected = False
        self.frame_count = 0

        self.get_logger().info('Lane Detection Node da khoi dong!')
        self.get_logger().info(f'Debug camera: {self.debug_camera}')
        self.get_logger().info(f'Use full image: {self.use_full_image}')

    def imgmsg_to_cv2(self, img_msg, encoding="bgr8"):
        """Convert ROS2 Image message sang OpenCV image (numpy array)"""
        if encoding == "bgr8" or encoding == "rgb8":
            dtype = np.uint8
            img_buf = np.frombuffer(img_msg.data, dtype=dtype)
            if img_msg.height * img_msg.width * 3 == len(img_buf):
                img_buf = img_buf.reshape((img_msg.height, img_msg.width, 3))
                if encoding == "rgb8":
                    img_buf = cv2.cvtColor(img_buf, cv2.COLOR_RGB2BGR)
                return img_buf
            else:
                raise ValueError(f"Image size mismatch: expected {img_msg.height * img_msg.width * 3}, got {len(img_buf)}")
        else:
            raise ValueError(f"Encoding {encoding} chua duoc ho tro")

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

    def _make_camera_info(self, width, height, stamp, frame_id="camera_link_optical"):
        """Tạo CameraInfo cho RViz / rqt_image_view (cùng stamp với image_debug)."""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width = width
        info.height = height
        fx = fy = width / (2.0 * np.tan(np.pi / 6.0))
        cx, cy = width / 2.0, height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def image_callback(self, msg):
        """Callback xu ly anh tu camera"""
        try:
            cv_image = self.imgmsg_to_cv2(msg, "bgr8")
            self.frame_count += 1

            if self.frame_count == 1:
                self.get_logger().info('Da nhan duoc anh camera lan dau!')

            self.process_lane_detection(cv_image)

        except Exception as e:
            self.get_logger().error(f'Loi xu ly anh: {str(e)}')

    def process_lane_detection(self, image):
        """Xu ly camera de phat hien 2 vach DEN 2 ben duong"""
        if image is None:
            return

        try:
            height, width = image.shape[:2]
            image_with_lanes = image.copy()

            # Tao vung quan tam (ROI)
            if self.use_full_image:
                # Dung full anh (khong crop)
                roi_top = 0
                roi_bottom = height
                roi = image.copy()
            else:
                # Crop phan duoi anh (vung duong)
                roi_top = int(height * self.roi_top_percent)
                roi_bottom = height
                roi = image[roi_top:roi_bottom, :]

            # Chuyen sang Grayscale de phat hien vach den
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

            # Ap dung Gaussian blur truoc de giam nhieu
            gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Dung Adaptive Threshold de tu dong dieu chinh theo anh sang
            black_mask = cv2.adaptiveThreshold(
                gray_blurred,
                255,
                cv2.ADAPTIVE_THRESH_MEAN_C,
                cv2.THRESH_BINARY_INV,
                blockSize=25,
                C=10
            )

            # Ap dung them mot lan blur de lam min mask
            blurred = cv2.GaussianBlur(black_mask, (5, 5), 0)

            # Phat hien canh bang Canny
            edges = cv2.Canny(blurred, 50, 150)

            # Phat hien duong thang bang HoughLinesP (tang threshold/minLen de bo doan ngan, nhiêu)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40,
                                   minLineLength=60, maxLineGap=25)

            # Chi nhan 2 vach: 1 ben trai, 1 ben phai. Vung giua bo qua.
            center_x = width / 2
            left_zone_max = width * 0.42   # Trai: mid_x phai < 42% man hinh
            right_zone_min = width * 0.58  # Phai: mid_x phai > 58% man hinh

            left_candidates = []   # (bottom_x, line)
            right_candidates = []

            if lines is not None and len(lines) > 0:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    mid_x = (x1 + x2) / 2
                    bottom_x = x1 if y1 > y2 else x2  # Diem gan camera hon (y lon hon)

                    if abs(x2 - x1) < 1:
                        slope = 999
                    else:
                        slope = (y2 - y1) / (x2 - x1)

                    # Chi nhan duong gan thang dung, va nam dung vung trai/phai
                    if abs(slope) > 0.5 or abs(slope) == 999:
                        if mid_x < left_zone_max:
                            left_candidates.append((bottom_x, line[0]))
                        elif mid_x > right_zone_min:
                            right_candidates.append((bottom_x, line[0]))
                        # else: bo qua vung giua (0.42 -> 0.58)

            # Chi lay 1 vach trai (xa trai nhat = bottom_x nho nhat), 1 vach phai (xa phai nhat = bottom_x lon nhat)
            left_x_points = []
            right_x_points = []
            best_left_line = None
            best_right_line = None

            if left_candidates:
                best_left_line = min(left_candidates, key=lambda t: t[0])  # bottom_x nho nhat
                x1, y1, x2, y2 = best_left_line[1]
                left_x_points.append(x1 if y1 > y2 else x2)
            if right_candidates:
                best_right_line = max(right_candidates, key=lambda t: t[0])  # bottom_x lon nhat
                x1, y1, x2, y2 = best_right_line[1]
                right_x_points.append(x1 if y1 > y2 else x2)

            # Keo dai va ve het ca duong (tu dau ROI xuong day anh) - to het ca line
            roi_h = roi_bottom - roi_top

            def extend_and_draw(line_data, color):
                """Tu (x1,y1),(x2,y2) trong ROI, keo dai line tu y=0 den y=roi_h, ve len anh."""
                if line_data is None:
                    return
                x1, y1, x2, y2 = line_data[1]
                if abs(x2 - x1) < 1:
                    # Thang dung: x co dinh
                    x = int((x1 + x2) / 2)
                    pt1 = (max(0, min(x, width - 1)), roi_top)
                    pt2 = (max(0, min(x, width - 1)), height - 1)
                else:
                    slope = (y2 - y1) / (x2 - x1)
                    # x = x1 + (y - y1) / slope
                    x_at_top = x1 - y1 / slope
                    x_at_bot = x1 + (roi_h - 1 - y1) / slope
                    pt1 = (int(np.clip(x_at_top, 0, width - 1)), roi_top)
                    pt2 = (int(np.clip(x_at_bot, 0, width - 1)), height - 1)
                cv2.line(image_with_lanes, pt1, pt2, color, 3)

            extend_and_draw(best_left_line, (255, 0, 0))
            extend_and_draw(best_right_line, (0, 0, 255))

            # Tinh offset tu giua duong
            lane_center = None
            if left_x_points and right_x_points:
                left_x_avg = np.mean(left_x_points)
                right_x_avg = np.mean(right_x_points)
                lane_center = (left_x_avg + right_x_avg) / 2
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif left_x_points:
                left_x_avg = np.mean(left_x_points)
                lane_center = left_x_avg + self.lane_width_pixels
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            elif right_x_points:
                right_x_avg = np.mean(right_x_points)
                lane_center = right_x_avg - self.lane_width_pixels
                self.lane_center_offset = (lane_center - center_x) / (width / 2)
                self.lane_detected = True
            else:
                self.lane_detected = False
                self.lane_center_offset = 0.0

            # Ve thong tin len anh
            center_y_bottom = height
            center_y_top = roi_top

            # Ve duong giua man hinh (xanh la)
            cv2.line(image_with_lanes, (int(center_x), center_y_bottom),
                    (int(center_x), center_y_top), (0, 255, 0), 2)

            if lane_center is not None:
                # Ve duong giua lan (vang)
                cv2.line(image_with_lanes, (int(lane_center), center_y_bottom),
                        (int(lane_center), center_y_top), (0, 255, 255), 3)

            # Ve text thong tin
            status_text = "Phat hien lan duong" if self.lane_detected else "Khong phat hien lan"
            offset_text = f"Offset: {self.lane_center_offset:.2f}"
            cv2.putText(image_with_lanes, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image_with_lanes, offset_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Ve huong can di
            if self.lane_detected:
                arrow_x = int(center_x)
                arrow_y = int(height * 0.75)
                offset_pixels = int(self.lane_center_offset * width * 0.4)
                arrow_end_x = arrow_x + offset_pixels
                arrow_end_y = arrow_y - 60

                if abs(offset_pixels) > 5:
                    cv2.arrowedLine(image_with_lanes,
                                   (arrow_x, arrow_y),
                                   (arrow_end_x, arrow_end_y),
                                   (0, 255, 255), 5, tipLength=0.3)

                if abs(self.lane_center_offset) < 0.1:
                    direction_text = "Di thang"
                    direction_color = (0, 255, 0)
                elif self.lane_center_offset > 0:
                    direction_text = f"Re trai ({abs(self.lane_center_offset):.2f})"
                    direction_color = (255, 165, 0)
                else:
                    direction_text = f"Re phai ({abs(self.lane_center_offset):.2f})"
                    direction_color = (255, 165, 0)

                cv2.putText(image_with_lanes, direction_text, (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)
            else:
                cv2.putText(image_with_lanes, "Khong xac dinh duoc huong", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Publish ket qua
            # 1. Publish offset
            offset_msg = Float32()
            offset_msg.data = float(self.lane_center_offset)
            self.offset_pub.publish(offset_msg)

            # 2. Publish detected flag
            detected_msg = Bool()
            detected_msg.data = self.lane_detected
            self.detected_pub.publish(detected_msg)

            # 3. Publish anh debug va camera_info (RViz/rqt_image_view can /lane_detection/camera_info)
            if self.debug_camera:
                current_time = self.get_clock().now().to_msg()
                ros_image = self.cv2_to_imgmsg(image_with_lanes, "bgr8")
                ros_image.header.stamp = current_time
                ros_image.header.frame_id = "camera_link_optical"
                self.image_debug_pub.publish(ros_image)
                cam_info = self._make_camera_info(
                    image_with_lanes.shape[1], image_with_lanes.shape[0], current_time
                )
                self.camera_info_pub.publish(cam_info)

            # Log dinh ky (moi 30 frames ~ 1 giay)
            if self.frame_count % 30 == 0:
                if self.lane_detected:
                    self.get_logger().info(f'Lane detected - Offset: {self.lane_center_offset:.2f}')
                else:
                    self.get_logger().info('Lane NOT detected')

        except Exception as e:
            self.get_logger().debug(f'Loi xu ly lane detection: {str(e)}')
            self.lane_detected = False
            self.lane_center_offset = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Loi trong node: {str(e)}')
    finally:
        if node:
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
